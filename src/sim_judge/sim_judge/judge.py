"""RoboBoat 2026 sim judge: RoboCommand stand-in + referee + live scoreboard.

  ros2 run sim_judge judge --answer-key <course>.answer_key.json [--port 50000]

Ground truth comes straight from Gazebo (gz-transport, /model/<boat>/odometry), so no ROS bridge
is needed. Point message_node at this server with robocommand_host:=127.0.0.1. Ctrl+C ends the
run and writes <report-dir>/<timestamp>_seed<N>.json and .md.
"""

import argparse
import datetime
import json
import math
import os
import signal
import sys
import threading
import time

from .referee import INTERPRETATIONS, Referee, render
from .robocommand import RoboCommandServer, to_dict


def _default_answer_key():
    return os.environ.get('SIM_ANSWER_KEY', '')


def _parse_args(argv):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--answer-key', default=_default_answer_key(),
                    help='answer key JSON from generate_course.py (default: $SIM_ANSWER_KEY)')
    ap.add_argument('--host', default='0.0.0.0')
    ap.add_argument('--port', type=int, default=int(os.environ.get('ROBOCOMMAND_PORT', 50000)))
    ap.add_argument('--boat', default='ourboat', help='Gazebo model name of the team boat')
    ap.add_argument('--odom-topic', default=None, help='default /model/<boat>/odometry')
    ap.add_argument('--boat-length', type=float, default=1.2, help='hull footprint for strikes (m)')
    ap.add_argument('--boat-width', type=float, default=0.9)
    ap.add_argument('--report-tol', type=float, default=3.0, help='object report position tolerance (m)')
    ap.add_argument('--gate-report-tol', type=float, default=10.0, help='GatePass position tolerance (m)')
    ap.add_argument('--dock-hold', type=float, default=3.0, help='seconds inside a slip that count as docked')
    ap.add_argument('--duration', type=float, default=0.0, help='end the run after this many sim seconds (0 = never)')
    ap.add_argument('--report-dir', default=os.environ.get('SIM_RUNS_DIR', os.path.expanduser('~/.aquatonomous_sim/runs')))
    ap.add_argument('--refresh', type=float, default=1.0, help='scoreboard refresh period (s)')
    ap.add_argument('--plain', action='store_true', help='append scoreboards instead of redrawing the screen')
    return ap.parse_args(argv)


class GroundTruth:
    """Subscribes to the boat odometry over gz-transport and forwards poses to the referee."""

    def __init__(self, topic, callback):
        from gz.msgs10.odometry_pb2 import Odometry
        from gz.transport13 import Node
        self._node = Node()
        self._cb = callback
        self.count = 0
        if not self._node.subscribe(Odometry, topic, self._on_msg):
            raise RuntimeError(f'could not subscribe to {topic}')

    def _on_msg(self, msg):
        self.count += 1
        st = msg.header.stamp
        q = msg.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self._cb(st.sec + st.nsec * 1e-9, msg.pose.position.x, msg.pose.position.y, yaw)


def _markdown(score, key, args, started_wall):
    lines = [f'# RoboBoat 2026 sim run: seed {key.get("seed")}', '',
             f'- World: `{key.get("world")}`; started {started_wall}',
             f'- Total: **{score["total"]:.0f} / {score["max_total"]}** '
             f'(strikes {score["strikes"]["count"]}, {score["strikes"]["penalty"]} points)',
             f'- Qualifying criteria met: **{"yes" if score["qualifying"]["qualified"] else "no"}**',
             f'- Sim time {score["run"]["elapsed"]:.1f} s'
             + (f'; terminated at the boundary at {score["run"]["terminated"]:.1f} s' if score['run']['terminated'] else ''),
             '']
    for task, items in score['tasks'].items():
        lines += [f'## Task {task}: {key["tasks"][task].get("name", "")}', '', '| Item | Points | Detail |', '|---|---|---|']
        for i in items:
            lines.append(f'| {i["desc"]} | {i["points"]:.0f} / {i["max"]} | {i["detail"]} |')
        lines.append('')
    if score['strikes']['list']:
        lines += ['## Strikes', ''] + [f'- {s["t"]:.1f} s: {s["name"]}' for s in score['strikes']['list']] + ['']
    hb = score['heartbeat']
    lines += ['## Communications', '',
              f'- Heartbeats: {hb["count"]}' + (f', mean interval {hb["mean_interval"]:.2f} s, max gap {hb["max_gap"]:.1f} s'
                                                if hb['mean_interval'] else '') + (' (OK)' if hb['ok'] else ' (NOT OK)'),
              f'- Frames: {score["protocol"]["frames"]}, errors: {score["protocol"]["errors"]}, '
              f'rate violations: {score["protocol"]["rate_violations"]}', '']
    if score['warnings']:
        lines += ['## Warnings', ''] + [f'- {w}' for w in score['warnings']] + ['']
    lines += ['## Judging interpretations', ''] + [f'- {t}' for t in INTERPRETATIONS] + ['']
    return '\n'.join(lines)


def main(argv=None):
    args = _parse_args(sys.argv[1:] if argv is None else argv)
    if not args.answer_key or not os.path.isfile(args.answer_key):
        sys.exit(f'answer key not found: {args.answer_key!r} (pass --answer-key or set SIM_ANSWER_KEY)')
    with open(args.answer_key, encoding='utf-8') as f:
        key = json.load(f)

    ref = Referee(key, boat_length=args.boat_length, boat_width=args.boat_width, report_tol=args.report_tol,
                  gate_report_tol=args.gate_report_tol, dock_hold=args.dock_hold)
    lock = threading.Lock()
    log = []

    def on_pose(t, x, y, yaw):
        with lock:
            ref.on_pose(t, x, y, yaw)

    def on_report(wall, report, peer):
        with lock:
            ref.on_report(wall, report, peer)
            log.append({'wall': wall, 'sim_t': ref.t, 'peer': peer, 'report': to_dict(report)})

    def on_error(wall, message, peer):
        with lock:
            ref.on_error(wall, message, peer)

    server = RoboCommandServer(args.host, args.port, on_report, on_error)
    server.start()
    topic = args.odom_topic or f'/model/{args.boat}/odometry'
    truth = GroundTruth(topic, on_pose)
    started_wall = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')
    print(f'RoboCommand stand-in listening on {args.host}:{args.port}; ground truth from {topic}', flush=True)

    stop = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, lambda *_: stop.set())
    tty = sys.stdout.isatty() and not args.plain
    last_plain = 0.0
    while not stop.is_set():
        stop.wait(args.refresh)
        with lock:
            score = ref.score(time.time())
            text = render(score, key)
            done = ref.terminated is not None or (
                args.duration > 0 and ref.run_start is not None and ref.t - ref.run_start >= args.duration)
        if truth.count == 0:
            text += f'\n(no odometry yet on {topic}; is Gazebo running with the course world?)'
        if tty:
            sys.stdout.write('\x1b[H\x1b[2J' + text + '\n')
            sys.stdout.flush()
        elif time.time() - last_plain >= 10.0 or done:
            print(text + '\n', flush=True)
            last_plain = time.time()
        if done:
            break

    server.stop()
    with lock:
        score = ref.score(time.time())
        events = list(ref.events)
    os.makedirs(args.report_dir, exist_ok=True)
    stem = os.path.join(args.report_dir, datetime.datetime.now().strftime('%Y%m%d-%H%M%S') + f'_seed{key.get("seed")}')
    with open(stem + '.json', 'w', encoding='utf-8') as f:
        json.dump({'answer_key': os.path.abspath(args.answer_key), 'seed': key.get('seed'), 'score': score,
                   'events': [{'t': t, 'event': e} for t, e in events], 'reports': log,
                   'interpretations': INTERPRETATIONS}, f, indent=1, default=str)
    with open(stem + '.md', 'w', encoding='utf-8') as f:
        f.write(_markdown(score, key, args, started_wall))
    print(render(score, key))
    print(f'\nRun report: {stem}.md (details in {stem}.json)')


if __name__ == '__main__':
    main()
