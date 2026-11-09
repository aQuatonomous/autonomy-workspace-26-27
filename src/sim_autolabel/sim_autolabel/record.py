"""Records labelled camera frames from the sim for CV training.

  ros2 run sim_autolabel record [--answer-key <course>.answer_key.json] [--out DIR]

Needs a course selected with `select_course.sh --labels`: each boat camera then has a
bounding-box camera twin with the boxes of every labelled course element in view on
/autolabel/<camera>, and a plain camera twin with the clean image on /autolabel/<camera>_rgb (the
box camera's own image has the boxes drawn in), both with the same sim time stamp.
Boxes cover only the visible part of an element (occluded elements get none). Boxes with a label
that is not in the course's table (the gz-waves water surface carries a changing one) are ignored.

Writes one session folder: session.json (course, weather, label names) and per camera
<camera>/<stamp>.jpg + <stamp>.json (boxes in pixels with sim label names, boat pose). Turn
sessions into a YOLO dataset with `ros2 run sim_autolabel export`. Reads Gazebo over
gz-transport, so no ROS bridge is needed. Ctrl+C stops and prints what was saved.
"""

import argparse
import collections
import datetime
import json
import math
import os
import signal
import sys
import threading
import time

CAMERAS = ('camera0', 'camera1', 'camera2')


def stamp_ns(header):
    return header.stamp.sec * 1_000_000_000 + header.stamp.nsec


def frame_record(camera, stamp, width, height, boxes, label_names, boat=None):
    """The per-frame JSON. boxes: (label_id, x0, y0, x1, y1) in pixels."""
    out = []
    for label, x0, y0, x1, y1 in boxes:
        out.append({'label': label_names.get(label, f'label_{label}'), 'label_id': label,
                    'x0': round(x0, 1), 'y0': round(y0, 1), 'x1': round(x1, 1), 'y1': round(y1, 1)})
    return {'camera': camera, 'stamp': round(stamp, 4), 'width': width, 'height': height,
            'boat': boat, 'boxes': out}


class Pairer:
    """Matches each camera's image and boxes messages by stamp (the sensor publishes both per frame)."""

    def __init__(self, on_pair, keep=16):
        self._pending = collections.OrderedDict()
        self._on_pair = on_pair
        self._keep = keep
        self._lock = threading.Lock()

    def add(self, camera, stamp, kind, msg):
        with self._lock:
            entry = self._pending.setdefault((camera, stamp), {})
            entry[kind] = msg
            if 'image' not in entry or 'boxes' not in entry:
                while len(self._pending) > self._keep:
                    self._pending.popitem(last=False)
                return
            del self._pending[(camera, stamp)]
        self._on_pair(camera, stamp, entry['image'], entry['boxes'])


class Recorder:
    def __init__(self, out_dir, label_names, empty_every, max_frames):
        self.out_dir = out_dir
        self.label_names = label_names
        self.empty_every = empty_every
        self.max_frames = max_frames
        self.boat = None
        self.saved = 0
        self.empty_seen = 0
        self.ignored = 0
        self.counts = collections.Counter()
        self.done = threading.Event()
        self._lock = threading.Lock()

    def on_odom(self, msg):
        q = msg.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.boat = {'x': round(msg.pose.position.x, 3), 'y': round(msg.pose.position.y, 3), 'yaw': round(yaw, 4)}

    def on_frame(self, camera, stamp, image_msg, boxes_msg):
        import cv2
        import numpy as np

        boxes = [(b.label, b.box.min_corner.x, b.box.min_corner.y, b.box.max_corner.x, b.box.max_corner.y)
                 for b in boxes_msg.annotated_box if b.label in self.label_names]
        with self._lock:
            self.ignored += len(boxes_msg.annotated_box) - len(boxes)
            if self.done.is_set():
                return
            if not boxes:
                # Frames with nothing in view teach the model what background looks like; keep a few.
                self.empty_seen += 1
                if self.empty_every <= 0 or self.empty_seen % self.empty_every:
                    return
            self.saved += 1
            if self.max_frames and self.saved >= self.max_frames:
                self.done.set()
            for b in boxes:
                self.counts[self.label_names[b[0]]] += 1
            boat = self.boat
        w, h = image_msg.width, image_msg.height
        step = image_msg.step or w * 3
        rgb = np.frombuffer(image_msg.data, dtype=np.uint8).reshape(h, step)[:, :w * 3].reshape(h, w, 3)
        d = os.path.join(self.out_dir, camera)
        os.makedirs(d, exist_ok=True)
        name = f'{stamp:019d}'
        cv2.imwrite(os.path.join(d, name + '.jpg'), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR),
                    [cv2.IMWRITE_JPEG_QUALITY, 95])
        rec = frame_record(camera, stamp * 1e-9, w, h, boxes, self.label_names, boat)
        with open(os.path.join(d, name + '.json'), 'w', newline='\n') as f:
            json.dump(rec, f)


def _parse_args(argv):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--answer-key', default=os.environ.get('SIM_ANSWER_KEY', ''),
                    help='answer key JSON from generate_course.py (default: $SIM_ANSWER_KEY)')
    ap.add_argument('--out', default=None,
                    help='session folder (default: $SIM_DATASETS_DIR/<time>_seed<N>)')
    ap.add_argument('--cameras', nargs='+', default=list(CAMERAS))
    ap.add_argument('--boat', default='ourboat', help='Gazebo model name of the team boat')
    ap.add_argument('--empty-every', type=int, default=10,
                    help='keep 1 in N frames with no labelled element in view (0 = none)')
    ap.add_argument('--max-frames', type=int, default=0, help='stop after this many frames (0 = until Ctrl+C)')
    return ap.parse_args(argv)


def main(argv=None):
    args = _parse_args(sys.argv[1:] if argv is None else argv)
    if not args.answer_key or not os.path.isfile(args.answer_key):
        print('No answer key. Select a course with select_course.sh --labels (or pass --answer-key).', file=sys.stderr)
        return 1
    with open(args.answer_key, encoding='utf-8') as f:
        key = json.load(f)
    if 'labels' not in key:
        print(f'{args.answer_key} has no label table; regenerate the course with the current generate_course.py.',
              file=sys.stderr)
        return 1
    label_names = {v: k for k, v in key['labels'].items()}

    started = datetime.datetime.now()
    out = args.out or os.path.join(
        os.environ.get('SIM_DATASETS_DIR', os.path.expanduser('~/.aquatonomous_sim/datasets')),
        f'{started:%Y-%m-%d_%H-%M-%S}_seed{key.get("seed", "x")}')
    os.makedirs(out, exist_ok=True)
    with open(os.path.join(out, 'session.json'), 'w', newline='\n') as f:
        json.dump({'started': started.isoformat(timespec='seconds'), 'answer_key': os.path.abspath(args.answer_key),
                   'seed': key.get('seed'), 'world': key.get('world'), 'waves': key.get('waves'),
                   'weather': key.get('weather'), 'camera': os.environ.get('SIM_CAMERA'),
                   'cameras': args.cameras, 'labels': key['labels']}, f, indent=2)
        f.write('\n')

    from gz.msgs10.annotated_axis_aligned_2d_box_v_pb2 import AnnotatedAxisAligned2DBox_V
    from gz.msgs10.image_pb2 import Image
    from gz.msgs10.odometry_pb2 import Odometry
    from gz.transport13 import Node

    rec = Recorder(out, label_names, args.empty_every, args.max_frames)
    pairer = Pairer(rec.on_frame)
    node = Node()
    if not node.subscribe(Odometry, f'/model/{args.boat}/odometry', rec.on_odom):
        print(f'could not subscribe to /model/{args.boat}/odometry', file=sys.stderr)
    for cam in args.cameras:
        topic = f'/autolabel/{cam}'
        ok = node.subscribe(Image, topic + '_rgb',
                            lambda m, c=cam: pairer.add(c, stamp_ns(m.header), 'image', m))
        ok &= node.subscribe(AnnotatedAxisAligned2DBox_V, topic,
                             lambda m, c=cam: pairer.add(c, stamp_ns(m.header), 'boxes', m))
        if not ok:
            print(f'could not subscribe to {topic}', file=sys.stderr)
            return 1

    signal.signal(signal.SIGINT, lambda *_: rec.done.set())
    signal.signal(signal.SIGTERM, lambda *_: rec.done.set())
    print(f'Recording {", ".join(args.cameras)} into {out} (Ctrl+C to stop)')
    last = -1
    while not rec.done.wait(5.0):
        if rec.saved == 0 and last == 0:
            print('  no frames yet: is the course selected with --labels and Gazebo running (not paused)?')
        elif rec.saved != last:
            print(f'  {rec.saved} frames, {sum(rec.counts.values())} boxes')
        last = rec.saved
    time.sleep(0.2)  # let an in-flight frame finish writing
    print(f'Saved {rec.saved} frames to {out}')
    for name, n in sorted(rec.counts.items()):
        print(f'  {name:24s} {n}')
    if rec.ignored:
        print(f'  ({rec.ignored} boxes with labels outside the course table ignored, e.g. the water)')
    return 0


if __name__ == '__main__':
    sys.exit(main())
