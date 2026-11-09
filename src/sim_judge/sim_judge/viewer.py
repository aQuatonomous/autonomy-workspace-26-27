"""Course ground truth and light camera feeds for Foxglove (or RViz) while the sim runs.

  ros2 run sim_judge viewer [--answer-key <course>.answer_key.json]

Publishes, for watching a run from another machine through foxglove_bridge (9_foxglove.sh):
  /sim/course                     every course element from the answer key (MarkerArray, latched)
  /sim/boat_truth                 the boat's true pose from Gazebo (PoseStamped)
  /sim/view/cameraN/compressed    JPEG copies of /cameraN/image_raw at a few Hz (raw images are
                                  ~25 MB/s per camera, too much for Wi-Fi or Tailscale)
  TF map -> sim_world             so the course lines up with the stack's map frame
  TF base_link -> rslidar         so the raw Airy cloud can be drawn next to the boat

The stack's map frame starts at the boat's first GPS fix; sim_world is the Gazebo world (east,
north). The offset between them is estimated from the stack's /boat_pose_stamped and the true
pose, and starts at the course's start position (where the first fix is taken) until then.
The ground truth is for watching only; the stack never subscribes to it.
"""

import argparse
import json
import os
import sys
import time

KIND_HEIGHT = {'sur_mark': 1.2, 'indicator_buoy': 1.0, 'light_buoy': 1.0, 'dock_indicator': 0.4, 'a2': 0.6,
               'a0': 0.4}
COLORS = {
    'red': (0.9, 0.1, 0.1), 'green': (0.1, 0.8, 0.2), 'black': (0.15, 0.15, 0.15),
    'yellow': (1.0, 0.85, 0.1), 'blue': (0.1, 0.4, 1.0), 'white': (0.95, 0.95, 0.95), None: (0.7, 0.7, 0.7),
}


def course_shapes(key):
    """Plain shapes for every answer-key element, in the Gazebo world frame (x east, y north).

    Each shape is a dict with type 'cylinder' | 'outline' | 'cubes' | 'text' | 'line', a name, a
    colour (r, g, b) and the geometry for that type."""
    shapes = []
    b = key.get('boundary') or []
    if b:
        shapes.append({'type': 'outline', 'name': 'boundary', 'color': (1.0, 1.0, 1.0),
                       'points': [tuple(p) for p in b]})
    by_id = {}
    for e in key.get('elements', []):
        by_id[e['id']] = e
        kind, color = e.get('kind'), COLORS.get(e.get('color'), COLORS[None])
        if kind in KIND_HEIGHT:
            shapes.append({'type': 'cylinder', 'name': e['name'], 'color': color,
                           'x': e['x'], 'y': e['y'], 'radius': e.get('radius') or 0.2,
                           'height': KIND_HEIGHT[kind]})
        elif kind == 'vessel' and e.get('footprint'):
            shapes.append({'type': 'outline', 'name': e['name'], 'color': color,
                           'points': [tuple(p) for p in e['footprint']]})
        elif kind == 'dock' and e.get('cubes'):
            shapes.append({'type': 'cubes', 'name': e['name'], 'color': (0.55, 0.55, 0.6),
                           'size': e.get('cube_size', 0.5), 'points': [tuple(p) for p in e['cubes']]})
        elif kind == 'banner':
            shapes.append({'type': 'text', 'name': e['name'], 'color': (1.0, 1.0, 1.0),
                           'x': e['x'], 'y': e['y'], 'z': 1.0, 'text': str(e.get('number', '?'))})
    for g in key.get('gates', []):
        r, gr = by_id.get(g.get('red_id')), by_id.get(g.get('green_id'))
        if r and gr:
            shapes.append({'type': 'line', 'name': g['label'], 'color': (0.3, 0.7, 1.0),
                           'points': [(r['x'], r['y']), (gr['x'], gr['y'])]})
            shapes.append({'type': 'text', 'name': g['label'] + '_label', 'color': (0.3, 0.7, 1.0),
                           'x': (r['x'] + gr['x']) / 2, 'y': (r['y'] + gr['y']) / 2, 'z': 1.5,
                           'text': g['label']})
    return shapes


class OffsetEstimate:
    """Mean of (true world position - stack map position) over the last samples."""

    def __init__(self, initial, keep=100):
        self.initial, self.keep, self.samples = tuple(initial), keep, []

    def add(self, world_xy, map_xy):
        self.samples.append((world_xy[0] - map_xy[0], world_xy[1] - map_xy[1]))
        del self.samples[:-self.keep]

    def value(self):
        if not self.samples:
            return self.initial
        n = len(self.samples)
        return (sum(s[0] for s in self.samples) / n, sum(s[1] for s in self.samples) / n)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--answer-key', default=os.environ.get('SIM_ANSWER_KEY', ''))
    ap.add_argument('--cameras', nargs='*', default=['camera0', 'camera1', 'camera2'])
    ap.add_argument('--camera-rate', type=float, default=5.0, help='JPEG frames per second per camera')
    ap.add_argument('--jpeg-quality', type=int, default=70)
    args, ros_args = ap.parse_known_args(argv)
    if not args.answer_key or not os.path.isfile(args.answer_key):
        print('No answer key; select a course with select_course.sh first.', file=sys.stderr)
        return 1
    key = json.load(open(args.answer_key))

    import rclpy
    from rclpy.executors import ExternalShutdownException
    from geometry_msgs.msg import PoseStamped, TransformStamped, Point
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
    from sensor_msgs.msg import CompressedImage, Image
    from std_msgs.msg import ColorRGBA
    from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
    from visualization_msgs.msg import Marker, MarkerArray

    try:
        import cv2
        import numpy as np
    except ImportError:
        cv2 = None

    rclpy.init(args=['sim_viewer'] + ros_args)
    node = Node('sim_viewer')
    log = node.get_logger()
    start = key.get('start') or {}
    offset = OffsetEstimate((start.get('x', 0.0), start.get('y', 0.0)))
    truth = {'xy': None, 't': 0.0}

    def color(c, a=1.0):
        return ColorRGBA(r=float(c[0]), g=float(c[1]), b=float(c[2]), a=a)

    markers = MarkerArray()
    for i, s in enumerate(course_shapes(key)):
        m = Marker()
        m.header.frame_id = 'sim_world'
        m.ns, m.id, m.action, m.frame_locked = 'course', i, Marker.ADD, True
        m.pose.orientation.w = 1.0
        m.color = color(s['color'])
        if s['type'] == 'cylinder':
            m.type = Marker.CYLINDER
            m.pose.position.x, m.pose.position.y, m.pose.position.z = float(s['x']), float(s['y']), s['height'] / 2
            m.scale.x = m.scale.y = 2 * float(s['radius'])
            m.scale.z = float(s['height'])
        elif s['type'] in ('outline', 'line'):
            m.type = Marker.LINE_STRIP
            pts = s['points'] + (s['points'][:1] if s['type'] == 'outline' else [])
            m.points = [Point(x=float(x), y=float(y), z=0.05) for x, y in pts]
            m.scale.x = 0.08
        elif s['type'] == 'cubes':
            m.type = Marker.CUBE_LIST
            size = float(s['size'])
            m.points = [Point(x=float(x), y=float(y), z=size / 2) for x, y in s['points']]
            m.scale.x = m.scale.y = m.scale.z = size
        else:
            m.type = Marker.TEXT_VIEW_FACING
            m.pose.position.x, m.pose.position.y, m.pose.position.z = float(s['x']), float(s['y']), float(s['z'])
            m.scale.z = 0.6
            m.text = s['text']
        markers.markers.append(m)
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_publisher(MarkerArray, '/sim/course', latched).publish(markers)

    static_tf = StaticTransformBroadcaster(node)
    lidar = TransformStamped()
    lidar.header.stamp = node.get_clock().now().to_msg()
    lidar.header.frame_id, lidar.child_frame_id = 'base_link', 'rslidar'
    lidar.transform.rotation.w = 1.0
    static_tf.sendTransform(lidar)

    tf = TransformBroadcaster(node)
    truth_pub = node.create_publisher(PoseStamped, '/sim/boat_truth', 10)

    def on_truth(msg):
        p = msg.pose.pose
        truth['xy'], truth['t'] = (p.position.x, p.position.y), time.monotonic()
        out = PoseStamped()
        out.header.stamp = node.get_clock().now().to_msg()
        out.header.frame_id = 'sim_world'
        out.pose = p
        truth_pub.publish(out)

    def on_stack_pose(msg):
        if truth['xy'] is not None and time.monotonic() - truth['t'] < 0.3:
            offset.add(truth['xy'], (msg.pose.position.x, msg.pose.position.y))

    def send_offset():
        ox, oy = offset.value()
        t = TransformStamped()
        t.header.stamp = node.get_clock().now().to_msg()
        t.header.frame_id, t.child_frame_id = 'map', 'sim_world'
        t.transform.translation.x, t.transform.translation.y = -ox, -oy
        t.transform.rotation.w = 1.0
        tf.sendTransform(t)

    node.create_subscription(Odometry, '/sim/ground_truth/odom', on_truth, qos_profile_sensor_data)
    node.create_subscription(PoseStamped, '/boat_pose_stamped', on_stack_pose, 10)
    node.create_timer(0.2, send_offset)

    if cv2 is None:
        log.warning('python3-opencv is missing; no compressed camera feeds')
    else:
        period = 1.0 / max(args.camera_rate, 0.1)
        for cam in args.cameras:
            pub = node.create_publisher(CompressedImage, f'/sim/view/{cam}/compressed', 2)
            last = {'t': 0.0}

            def on_image(msg, pub=pub, last=last):
                now = time.monotonic()
                if now - last['t'] < period or msg.encoding not in ('rgb8', 'bgr8'):
                    return
                last['t'] = now
                img = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)[:, :msg.width * 3]
                img = img.reshape(msg.height, msg.width, 3)
                if msg.encoding == 'rgb8':
                    img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
                ok, jpg = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, args.jpeg_quality])
                if ok:
                    out = CompressedImage(header=msg.header, format='jpeg', data=jpg.tobytes())
                    pub.publish(out)

            node.create_subscription(Image, f'/{cam}/image_raw', on_image, qos_profile_sensor_data)

    log.info(f'Course {os.path.basename(args.answer_key)}: {len(markers.markers)} markers on /sim/course; '
             f'true pose on /sim/boat_truth; cameras on /sim/view/<camera>/compressed')
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
