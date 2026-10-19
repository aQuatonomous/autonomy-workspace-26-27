"""Send RoboCommand reports to a judge (or any RoboCommand server) to check the link.

  ros2 run sim_judge send_test_reports --answer-key <course>.answer_key.json [--seconds 5]

Sends 1 Hz heartbeats at the course start position plus one GatePass, ObjectDetected and Docking
report built from the answer key, using the same framing as message_node.
"""

import argparse
import json
import socket
import time

from google.protobuf.timestamp_pb2 import Timestamp

from . import report_compat as pb
from .robocommand import encode_frame


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--host', default='127.0.0.1')
    ap.add_argument('--port', type=int, default=50000)
    ap.add_argument('--answer-key', required=True)
    ap.add_argument('--seconds', type=int, default=5)
    args = ap.parse_args(argv)
    key = json.load(open(args.answer_key, encoding='utf-8'))
    start = key['start']
    pos = pb.LatLng(latitude=start['lat'], longitude=start['lon'])
    seq = 0

    def send(**body):
        nonlocal seq
        ts = Timestamp()
        ts.GetCurrentTime()
        r = pb.Report(team_id='TEST', vehicle_id='sim', seq=seq, sent_at=ts, **body)
        seq += 1
        with socket.create_connection((args.host, args.port), timeout=2.0) as s:
            s.sendall(encode_frame(r))

    el = {e['id']: e for e in key['elements']}
    for i in range(args.seconds):
        send(heartbeat=pb.Heartbeat(state=pb.RobotState.STATE_AUTO, position=pos, spd_mps=0.0,
                                    heading_deg=start['compass_heading_deg'],
                                    current_task=pb.TaskType.TASK_ENTRY_EXIT))
        if i == 1:
            send(gate_pass=pb.GatePass(type=pb.GateType.GATE_ENTRY, position=pos))
        if i == 2 and '2' in key['tasks']:
            e = el[key['tasks']['2']['indicators'][0]['id']]
            send(object_detected=pb.ObjectDetected(
                object_type=pb.ObjectType.OBJECT_LIGHT_BEACON,
                color=pb.Color.COLOR_RED if e['color'] == 'red' else pb.Color.COLOR_GREEN,
                position=pb.LatLng(latitude=e['lat'], longitude=e['lon']), object_id=1,
                task_context=pb.TaskType.TASK_NAV_CHANNEL))
        if i == 3 and '5' in key['tasks']:
            c = key['tasks']['5']['correct'][0]
            send(docking=pb.Docking(dock=c['dock'], slip=str(c['slip'])))
        time.sleep(1.0)
    print(f'sent {seq} reports to {args.host}:{args.port}')


if __name__ == '__main__':
    main()
