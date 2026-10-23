#!/usr/bin/env python3
"""Offline re-analysis of a recorded run. No ROS, no Gazebo, no waiting.

The sim runs at 0.5-1.1x realtime, so a single course takes minutes and a parameter sweep done
by re-running it would take a day. The evaluator writes every detection it saw to JSONL precisely
so that the expensive part -- driving the course -- is paid once and the cheap part -- deciding
what the numbers mean -- can be repeated as often as you like.

Three things it does:

  analyse_run.py summarise RUN.jsonl
      What was actually recorded: detection counts, CV score distribution, staleness, coverage.
      Read this before trusting any metric derived from the file.

  analyse_run.py fit-confusion RUN.jsonl [RUN2.jsonl ...] -o confusion_prior.yaml
      Replace the hand-built confusion prior with one measured from real detections. Fit on one
      set of seeds and validate on another: a matrix fitted and validated on seed 0 tells you
      only that seed 0 is memorisable.

  analyse_run.py replay RUN.jsonl [--set existence.p_detect=0.9 ...]
      Re-run the scoring maths over the recorded detections with different tunables and print
      the resulting existence/position metrics. This is the sweep loop.

Caveat worth stating plainly: replay re-scores recorded *detections*, so it can answer "would a
different filter have done better on this data" but not "would a different filter have seen
different things". Anything upstream of /fused_buoys is frozen.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
from typing import Dict, Iterable, List, Optional, Tuple

from .config_io import load_confusion_matrix, load_scorer_config, save_confusion_matrix
from .core import metrics
from .core.association import Measurement
from .core.classes import UNKNOWN_ID, class_name
from .core.config import ScorerConfig
from .core.confusion import matrix_from_counts
from .core.scorer import ObjectMap
from .core.visibility import BoatPose


# ---------------------------------------------------------------------------------------------
# Reading
# ---------------------------------------------------------------------------------------------

def read_records(path: str) -> List[Dict]:
    records = []
    with open(path, 'r', encoding='utf-8') as handle:
        for lineno, line in enumerate(handle, 1):
            line = line.strip()
            if not line:
                continue
            try:
                records.append(json.loads(line))
            except json.JSONDecodeError:
                # A run killed mid-write leaves a torn final line. That is expected and not
                # worth discarding the whole file over, but anything else is a real problem.
                print(f'{path}:{lineno}: unparseable line, skipped', file=sys.stderr)
    return records


def split(records: Iterable[Dict]) -> Tuple[Optional[Dict], List[Dict], List[Dict]]:
    """Separate the log into (meta, detections, frames).

    The resolved frame offset is folded into meta if the run wrote one. It is logged at
    shutdown rather than at startup because it is *measured* against Gazebo odometry during the
    run, and the assumed value from the answer key is only the fallback.
    """
    meta, detections, frames = None, [], []
    resolved_offset = None
    for r in records:
        kind = r.get('type')
        if kind == 'meta':
            meta = r
        elif kind == 'detection':
            detections.append(r)
        elif kind == 'frame':
            frames.append(r)
        elif kind == 'offset':
            resolved_offset = r
    if meta is not None and resolved_offset is not None:
        meta = dict(meta)
        meta['resolved_offset'] = resolved_offset
    return meta, detections, frames


def offset_of(meta: Dict) -> Tuple[float, float]:
    """The map<->answer-key translation this run actually used."""
    resolved = (meta or {}).get('resolved_offset')
    if resolved and resolved.get('estimated'):
        return (float(resolved['estimated'][0]), float(resolved['estimated'][1]))
    start = (meta or {}).get('start') or {}
    return (float(start.get('x', 0.0)), float(start.get('y', 0.0)))


# ---------------------------------------------------------------------------------------------
# summarise
# ---------------------------------------------------------------------------------------------

def cmd_summarise(args) -> int:
    for path in args.jsonl:
        meta, detections, frames = split(read_records(path))
        print(f'\n=== {os.path.basename(path)}')
        if meta is None:
            print('  no meta record: this file was not written by truth_eval_node')
            continue
        print(f'  seed {meta.get("seed")}  world {meta.get("world")}  '
              f'{len(meta.get("truth_elements") or [])} truth elements')
        print(f'  {len(frames)} logged frames, {len(detections)} logged detections')
        if not detections:
            print('  NOTHING TO ANALYSE: no detections were recorded.')
            continue

        classed = [d for d in detections if d.get('class_id') not in (None, UNKNOWN_ID)]
        scores = [d['class_score'] for d in classed if d.get('class_score') is not None]
        ages = [d['cv_age_s'] for d in detections if d.get('cv_age_s') is not None]
        near = [d for d in detections if d.get('truth_id') is not None]

        print(f'  carrying a CV class: {len(classed)} ({len(classed) / len(detections):.1%})')
        if scores:
            q = metrics.percentiles(scores, (50, 90, 95))
            print(f'  CV score p50/p90/p95: {q["p50"]:.3f} / {q["p90"]:.3f} / {q["p95"]:.3f}'
                  f'  max {max(scores):.3f}')
            if max(scores) < 0.3:
                print('  WARNING: no detection ever scored above 0.3. The class posterior '
                      'received essentially no evidence; high entropy downstream is correct.')
        if ages:
            q = metrics.percentiles(ages, (50, 95))
            stale = sum(1 for a in ages if a > 0.75) / len(ages)
            print(f'  CV age p50/p95: {q["p50"]:.2f} s / {q["p95"]:.2f} s, stale {stale:.1%}')
        else:
            print('  CV age: not recorded (FusedBuoy predates class_stamp)')
        print(f'  detections near a truth element: {len(near)} '
              f'({len(near) / len(detections):.1%}); the rest are clutter or off-course')

        counts = confusion_counts(detections)
        if counts:
            print('  observed class given truth (top rows):')
            for true_id, row in sorted(counts.items(),
                                       key=lambda kv: -sum(kv[1].values()))[:6]:
                total = sum(row.values())
                best = sorted(row.items(), key=lambda kv: -kv[1])[:3]
                detail = ', '.join(f'{class_name(int(c))} {n / total:.0%}' for c, n in best)
                print(f'    {class_name(int(true_id)):<18} n={int(total):<5} {detail}')
    return 0


# ---------------------------------------------------------------------------------------------
# fit-confusion
# ---------------------------------------------------------------------------------------------

def confusion_counts(detections: Iterable[Dict]) -> Dict[int, Dict[int, float]]:
    """Tally (true class -> observed class) from detections matched to a truth element.

    Detections with no truth element nearby are excluded rather than filed under "unknown":
    they are clutter, and a clutter return says nothing about how the detector confuses classes.
    """
    counts: Dict[int, Dict[int, float]] = {}
    for d in detections:
        true_id = d.get('truth_class_id')
        obs_id = d.get('class_id')
        if true_id is None or obs_id is None or obs_id == UNKNOWN_ID:
            continue
        row = counts.setdefault(int(true_id), {})
        row[int(obs_id)] = row.get(int(obs_id), 0.0) + 1.0
    return counts


def cmd_fit_confusion(args) -> int:
    merged: Dict[int, Dict[int, float]] = {}
    seeds = []
    total = 0
    for path in args.jsonl:
        meta, detections, _ = split(read_records(path))
        if meta is not None:
            seeds.append(meta.get('seed'))
        for true_id, row in confusion_counts(detections).items():
            dst = merged.setdefault(true_id, {})
            for obs_id, n in row.items():
                dst[obs_id] = dst.get(obs_id, 0.0) + n
                total += n

    if total < args.min_samples:
        print(f'only {int(total)} usable detections across {len(args.jsonl)} file(s); '
              f'refusing to fit a confusion matrix from fewer than {args.min_samples}. '
              'A matrix fitted on a handful of frames is a worse prior than the hand-built one, '
              'because it carries false authority.', file=sys.stderr)
        return 1

    matrix = matrix_from_counts(merged, prior_strength=args.prior_strength)
    note = (f'Fitted from {int(total)} matched detections across seed(s) '
            f'{sorted(s for s in seeds if s is not None)}.\n'
            f'Smoothed toward the hand-built prior with {args.prior_strength} pseudo-counts '
            'per row.\n'
            'Validate on a seed that is NOT in that list before trusting it.')
    save_confusion_matrix(args.output, matrix, note=note)
    print(f'wrote {args.output} from {int(total)} detections, seeds '
          f'{sorted(s for s in seeds if s is not None)}')

    for true_id in sorted(merged):
        row = merged[true_id]
        n = sum(row.values())
        correct = row.get(true_id, 0.0) / n if n else float('nan')
        print(f'  {class_name(true_id):<18} n={int(n):<6} P(correct)={correct:.2f}')
    return 0


# ---------------------------------------------------------------------------------------------
# replay
# ---------------------------------------------------------------------------------------------

def _apply_overrides(config: ScorerConfig, overrides: List[str]) -> ScorerConfig:
    data = config.to_dict()
    for item in overrides:
        if '=' not in item:
            raise SystemExit(f'--set expects section.key=value, got {item!r}')
        path, value = item.split('=', 1)
        parts = path.split('.')
        if len(parts) != 2:
            raise SystemExit(f'--set expects section.key=value, got {path!r}')
        section, key = parts
        if section not in data or key not in data[section]:
            raise SystemExit(f'unknown tunable {path!r}')
        # Preserve the declared type: prune_after_expected_misses is a count, and silently
        # turning it into 12.0 would be a confusing thing to find in a sweep log.
        try:
            data[section][key] = int(value) if isinstance(data[section][key], int) \
                else float(value)
        except ValueError:
            raise SystemExit(f'{path}: {value!r} is not a number')
    return ScorerConfig.from_dict(data)


def replay(records: List[Dict], config: ScorerConfig, confusion) -> Tuple[ObjectMap, Dict]:
    """Re-drive ObjectMap over the recorded detections.

    Detections are grouped by their recorded timestamp into frames, because the existence filter
    distinguishes "saw nothing here this frame" from "did not look", and feeding detections one
    at a time would turn every co-visible object into an expected miss.
    """
    meta, detections, frames = split(records)

    # Boat pose comes from the frame records; detections carry only map-frame points.
    poses = sorted(((f['t'], f['boat']) for f in frames if f.get('boat')), key=lambda kv: kv[0])
    if not poses:
        raise SystemExit('no boat poses in the log; cannot replay (frame records had boat=null)')

    def pose_at(t: float) -> BoatPose:
        best = min(poses, key=lambda kv: abs(kv[0] - t))
        b = best[1]
        return BoatPose(east=float(b['east']), north=float(b['north']),
                        yaw_rad=float(b['yaw']), stamp=t)

    by_time: Dict[float, List[Dict]] = {}
    for d in detections:
        by_time.setdefault(round(float(d['t']), 3), []).append(d)

    omap = ObjectMap(config, confusion)
    for t in sorted(by_time):
        pose = pose_at(t)
        measurements = []
        for d in by_time[t]:
            de, dn = d['east'] - pose.east, d['north'] - pose.north
            measurements.append(Measurement(
                east=float(d['east']), north=float(d['north']),
                range_m=float(d.get('range_m') or math.hypot(de, dn)),
                global_bearing_rad=math.atan2(dn, de),
                class_id=int(d.get('class_id') or UNKNOWN_ID),
                class_score=float(d.get('class_score') or 0.0),
                track_id=int(d.get('track_id') or 0),
                stamp=t,
                cv_age_s=d.get('cv_age_s'),
            ))
        omap.update(measurements, pose, now=t)
    return omap, (meta or {})


def cmd_replay(args) -> int:
    records = read_records(args.jsonl[0])
    config = _apply_overrides(load_scorer_config(args.config), args.set or [])
    confusion = load_confusion_matrix(args.confusion)
    omap, meta = replay(records, config, confusion)

    truth = meta.get('truth_elements') or []
    if not truth:
        print('no truth elements in the log; reporting scorer-internal numbers only')
        print(f'  objects={len(omap)} stats={omap.stats}')
        return 0

    # The log already stores truth in the map frame, as the evaluator resolved it.
    ox, oy = offset_of(meta)
    truth_xy = [(t['x'] - ox, t['y'] - oy) for t in truth]

    snapshot = omap.snapshot()
    scored_xy = [(o['east'], o['north']) for o in snapshot]
    scored_p = [o['p_exist'] for o in snapshot]
    match = metrics.match_to_truth(scored_xy, truth_xy, args.match_distance)
    matched = {i for i, _ in match.matches}
    outcomes = [1 if i in matched else 0 for i in range(len(snapshot))]

    errors, nees_values = [], []
    for si, ti in match.matches:
        o = snapshot[si]
        de = o['east'] - truth_xy[ti][0]
        dn = o['north'] - truth_xy[ti][1]
        errors.append(math.hypot(de, dn))
        nees_values.append(metrics.nees(de, dn, o['position_cov']))

    overrides = ' '.join(args.set or []) or '(defaults)'
    print(f'\nreplay {os.path.basename(args.jsonl[0])}  {overrides}')
    print(f'  objects {len(snapshot)}  matched {len(match.matches)}  '
          f'false positives {len(match.unmatched_scored)}  missed {len(match.unmatched_truth)}')
    print(f'  ECE {metrics.expected_calibration_error(scored_p, outcomes):.3f}  '
          f'Brier {metrics.brier_score(scored_p, outcomes):.3f}')
    q = metrics.percentiles(errors, (50, 90))
    n = metrics.nees_summary(nees_values)
    print(f'  position p50 {q["p50"]:.2f} m  p90 {q["p90"]:.2f} m  '
          f'NEES {n["mean"]:.2f} ({n["verdict"]})')
    print(f'  stats {omap.stats}')
    return 0


# ---------------------------------------------------------------------------------------------

def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(
        prog='analyse_run', description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest='command', required=True)

    p = sub.add_parser('summarise', help='what is actually in the log')
    p.add_argument('jsonl', nargs='+')
    p.set_defaults(func=cmd_summarise)

    p = sub.add_parser('fit-confusion', help='fit a confusion matrix from matched detections')
    p.add_argument('jsonl', nargs='+')
    p.add_argument('-o', '--output', required=True)
    p.add_argument('--prior-strength', type=float, default=2.0,
                   help='pseudo-counts per row pulling the fit toward the hand-built prior')
    p.add_argument('--min-samples', type=int, default=200,
                   help='refuse to fit from fewer matched detections than this')
    p.set_defaults(func=cmd_fit_confusion)

    p = sub.add_parser('replay', help='re-score the recorded detections with other tunables')
    p.add_argument('jsonl', nargs=1)
    p.add_argument('--config', default=None, help='scorer config YAML (default: built-in)')
    p.add_argument('--confusion', default=None, help='confusion prior YAML (default: built-in)')
    p.add_argument('--set', action='append', metavar='section.key=value',
                   help='override one tunable; repeatable')
    p.add_argument('--match-distance', type=float, default=2.0)
    p.set_defaults(func=cmd_replay)

    return ap


def main(argv=None) -> int:
    args = build_parser().parse_args(sys.argv[1:] if argv is None else argv)
    return args.func(args)


if __name__ == '__main__':
    raise SystemExit(main())
