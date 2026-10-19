import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from std_msgs.msg import String, Int32, Float64
from geometry_msgs.msg import TwistStamped, Twist, Vector3
from sensor_msgs.msg import NavSatFix
from mavros_msgs.msg import State 
from message_node_msgs.msg import ObjectDetection, Dock
from sound_signal_msgs.msg import SoundSignalWithFreq

from datetime import datetime, timezone
from google.protobuf.timestamp_pb2 import Timestamp
from google.protobuf import json_format
from .report_pb2 import *
from math import sqrt
import socket
import struct

# MAVROS sensor topics use BEST_EFFORT QoS; default RELIABLE would not receive them
_QOS_SENSOR = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
)


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.team_id="QUEU"
        self.vehicle_id="The Frontenac"
        self.seq = 0 #counter of current message number
        self.cur_task = TaskType.TASK_UNKNOWN
        self.state = RobotState.STATE_UNKNOWN
        self.position = LatLng(latitude=0,longitude=0)
        self.speed = 0.0
        self.heading = 0.0 
        self.message_pub = self.create_publisher(String, 'gs_message_send', 10)
        self.task_sub = self.create_subscription(
            Int32,
            'cur_task',
            self.task_callback,
            10)
        self.state_sub = self.create_subscription(
            State, #mavros_msgs/State.msg
            'mavros/state',
            self.state_callback,
            10)
        self.pos_sub = self.create_subscription(
                NavSatFix,
                'mavros/global_position/global',
                self.pos_callback,
                _QOS_SENSOR)
        self.vel_sub = self.create_subscription(
                TwistStamped,
                'mavros/global_position/gp_vel',
                self.vel_callback,
                _QOS_SENSOR)
        self.heading_sub = self.create_subscription(
                Float64,
                'mavros/global_position/compass_hdg',
                self.heading_callback,
                _QOS_SENSOR)
        self.task_sub  # prevent unused variable warning
        self.state_sub
        self.pos_sub
        self.vel_sub
        self.heading_sub
        self.heartbeat_timer = self.create_timer(1, self.heartbeat_callback)
        self.gate_pass_sub = self.create_subscription(
                String,
                'messages/gate_pass',
                self.gate_pass_callback,
                10)
        self.object_detected_sub = self.create_subscription(
                ObjectDetection,
                'messages/object_detected',
                self.object_detected_callback,
                10)
        self.object_delivery_sub = self.create_subscription(
                Int32,
                'messages/object_delivered',
                self.object_delivered_callback,
                10)
        self.docking_sub = self.create_subscription(
                Dock,
                'messages/docking',
                self.docking_callback,
                10)
        self.patrol_boat_sub = self.create_subscription(
                String,
                'messages/patrol_boat',
                self.patrol_boat_callback,
                10)
        self.sound_signal_sub = self.create_subscription(
                SoundSignalWithFreq,
                'sound_signal_interupt_freq',
                self.sound_signal_callback,
                10)

    #############################
    #     Ingress Callbacks     #
    #############################

    def task_callback(self, msg):
        match msg.data:
            case 1:
                self.cur_task = TaskType.TASK_ENTRY_EXIT
            case 2:
                self.cur_task = TaskType.TASK_NAV_CHANNEL
            case 3:
                self.cur_task = TaskType.TASK_SPEED_CHALLENGE
            case 4:
                self.cur_task = TaskType.TASK_OBJECT_DELIVERY
            case 5:
                self.cur_task = TaskType.TASK_DOCKING
            case 6:
                self.cur_task = TaskType.TASK_SOUND_SIGNAL
            case _:
                self.cur_task = TaskType.TASK_UNKNOWN

    def state_callback(self, msg):
        if (msg.guided):
            self.state = RobotState.STATE_AUTO
        elif (msg.manual_input):
            self.state = RobotState.STATE_MANUAL
        else:
            self.state = RobotState.STATE_UNKNOWN

    def pos_callback(self, msg):
        self.position = LatLng(latitude=msg.latitude, longitude=msg.longitude)

    def vel_callback(self, msg):
        v = msg.twist.linear #Twist is a Twist, linear is a Vector3
        self.speed = sqrt( v.x * v.x + v.y * v.y )

    def heading_callback(self, msg):
        self.heading = msg.data

    ############################
    #     Egress Callbacks     #
    ############################

    def message_send(self, report):
        self.seq += 1 # increment sequence count
        ts = Timestamp() 
        ts.FromDatetime(datetime.now(timezone.utc))
        report.sent_at.CopyFrom(ts)
        payload = report.SerializeToString()
        
        #payload = json_format.MessageToJson(report)
        #print(payload)

        #msg = String()
        #msg.data = payload
        #self.message_pub.publish(msg)
        # Header($R) + 1-byte length + payload + footer(!!)
        frame = b'$R' + struct.pack("!B", len(payload)) + payload + b'!!'
        
        try:
            with socket.create_connection(("10.10.10.1", 50000)) as s: #localhost for temp server
            #with socket.create_connection(("127.0.0.1", 50000)) as s: #localhost for temp server

                s.sendall(frame)
        except:
                self.get_logger().warning('Failed to connect to heartbeat message server')

    def heartbeat_callback(self):
        # Build message,
        report = Report(
            team_id=self.team_id,
            vehicle_id=self.vehicle_id,
            seq=self.seq,
            heartbeat=Heartbeat(
                state=self.state,
                position=self.position,
                spd_mps=self.speed,
                heading_deg=self.heading,
                current_task=self.cur_task
            ),
        )
        self.message_send(report)

    def gate_pass_callback(self, msg):
        # Build message,
        gate_type = GateType.GATE_UNKNOWN
        match msg.data:
            case "start":
                gate_type = GateType.GATE_ENTRY
            case "end":
                gate_type = GateType.GATE_EXIT
            case "speed_start":
                gate_type = GateType.GATE_SPEED_START
            case "speed_end":
                gate_type = GateType.GATE_SPEED_END
            case _:
                self.get_logger().warning('Malformed gate type in gate message: "%s"' % msg.data)

        report = Report(
            team_id=self.team_id,
            vehicle_id=self.vehicle_id,
            seq=self.seq,
            gate_pass=GatePass(
                type=gate_type,
                position=self.position
            ),
        )

        self.message_send(report)

    def object_detected_callback(self, msg):
        # Build message,
        obj_typ = ObjectType.OBJECT_UNKNOWN
        match msg.object_type:
            case "boat":
                obj_typ = ObjectType.OBJECT_BOAT
            case "light":
                obj_typ = ObjectType.OBJECT_LIGHT_BEACON
            case "buoy":
                obj_typ = ObjectType.OBJECT_BUOY
            case _:
                self.get_logger().warning('Malformed object type in object detected message: "%s"' % msg.object_type)

        col = Color.COLOR_UNKNOWN
        match msg.colour:
            case "red":
                col = Color.COLOR_RED
            case "green":
                col = Color.COLOR_GREEN
            case "yellow":
                col = Color.COLOR_YELLOW
            case "black":
                col = Color.COLOR_BLACK
            case _:
                self.get_logger().warning('Malformed colour in object detected message: "%s"' % msg.colour)
            
        report = Report(
            team_id=self.team_id,
            vehicle_id=self.vehicle_id,
            seq=self.seq,
            object_detected=ObjectDetected(
                object_type = obj_typ,
                color = col,
                position = LatLng(latitude=msg.latitude, longitude=msg.longitude),
                object_id = msg.object_id,
                task_context = self.cur_task,
            ),
        )

        self.message_send(report)

    def object_delivered_callback(self, msg):
        # Build message,
        report = Report(
            team_id=self.team_id,
            vehicle_id=self.vehicle_id,
            seq=self.seq,
            object_delivery=ObjectDelivery(
                vessel_color = Color.COLOR_YELLOW,
                position=self.position,
                delivery_type = DeliveryType.DELIVERY_WATER
            ),
        )

        self.message_send(report)
    
    def docking_callback(self, msg):
        # Build message,
        report = Report(
            team_id=self.team_id,
            vehicle_id=self.vehicle_id,
            seq=self.seq,
            docking=Docking(
                dock = msg.dock,
                slip = msg.slip
            ),
        )

        self.message_send(report)

    def patrol_boat_callback(self, msg):
        """Send PatrolBoat report to RoboCommand (STOPPING=1 or RESUMING=2)."""
        action = msg.data.strip().upper()
        if action == 'STOPPING':
            action_type = 1  # PATROL_BOAT_ACTION_STOPPING
        elif action == 'RESUMING':
            action_type = 2  # PATROL_BOAT_ACTION_RESUMING
        else:
            self.get_logger().warning('Unknown patrol_boat action: "%s"' % msg.data)
            return
        self._send_patrol_boat_report(action_type)

    def _send_patrol_boat_report(self, patrol_boat_action_type: int):
        """Build and send a Report with body=PatrolBoat (wire format, report_pb2 has no PatrolBoat)."""
        def varint_bytes(n: int) -> bytes:
            out = []
            while n > 0x7F:
                out.append((n & 0x7F) | 0x80)
                n >>= 7
            out.append(n & 0x7F)
            return bytes(out)

        def tag(field_num: int, wire_type: int) -> bytes:
            return varint_bytes((field_num << 3) | wire_type)

        def encode_string(s: str) -> bytes:
            b = s.encode('utf-8')
            return tag(1, 2) + varint_bytes(len(b)) + b

        now = datetime.now(timezone.utc)
        ts_seconds = int(now.timestamp())
        ts_nanos = now.microsecond * 1000
        # Timestamp: field 1 seconds, field 2 nanos
        ts_payload = tag(1, 0) + varint_bytes(ts_seconds) + tag(2, 0) + varint_bytes(ts_nanos)
        # PatrolBoat: field 1 patrol_boat_action_type (enum)
        patrol_payload = tag(1, 0) + varint_bytes(patrol_boat_action_type)
        # Report: 1 team_id, 2 vehicle_id, 3 seq, 4 sent_at, 16 patrol_boat
        self.seq += 1
        t_id = self.team_id.encode('utf-8')
        v_id = self.vehicle_id.encode('utf-8')
        payload = (
            tag(1, 2) + varint_bytes(len(t_id)) + t_id
            + tag(2, 2) + varint_bytes(len(v_id)) + v_id
            + tag(3, 0) + varint_bytes(self.seq)
            + tag(4, 2) + varint_bytes(len(ts_payload)) + ts_payload
            + tag(16, 2) + varint_bytes(len(patrol_payload)) + patrol_payload
        )
        frame = b'$R' + struct.pack("!B", len(payload)) + payload + b'!!'
        try:
            # Send to the same endpoint as other reports (local RoboCommand/test server)
            with socket.create_connection(("127.0.0.1", 50000)) as s:
                s.sendall(frame)
        except Exception:
            self.get_logger().warning('Failed to send patrol boat report to RoboCommand')

    def sound_signal_callback(self, msg):
        # Build message,
        sig_t = SignalType.SIGNAL_UNKNOWN
        ass_t = TaskType.TASK_UNKNOWN
        match msg.signal:
            case 1:
                sig_t = SignalType.SIGNAL_ONE_BLAST
                ass_t = TaskType.TASK_NAV_CHANNEL
            case 2: 
                sig_t = SignalType.SIGNAL_TWO_BLAST
                ass_t = TaskType.TASK_DOCKING
        
        report = Report(
            team_id=self.team_id,
            vehicle_id=self.vehicle_id,
            seq=self.seq,
            sound_signal=SoundSignal(
                signal_type = sig_t,
                frequency_hz = msg.freq,
                assigned_task = ass_t
            ),
        )

        self.message_send(report)






def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    try:
        rclpy.spin(minimal_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        minimal_subscriber.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass  # context may already be shut down by signal handler


if __name__ == '__main__':
    main()
