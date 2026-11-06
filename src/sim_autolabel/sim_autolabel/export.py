"""Turns recorded sim sessions into an Ultralytics YOLO dataset for one of the CV team's models.

  ros2 run sim_autolabel export SESSION [SESSION ...] --out DIR [--dataset main|numbers]
  python3 -m sim_autolabel.export ...      # same, without ROS (needs numpy, opencv, pyyaml)

SESSION is a folder written by `record`, or any folder containing several. The class list and the
sim-label -> class mapping come from classes.yaml (--classes to use another). Writes the layout
Roboflow exports and imports for YOLO: DIR/{train,valid}/{images,labels} and DIR/data.yaml, so
DIR can be dropped into a Roboflow project (or sent with `upload`) or passed straight to
`yolo train data=DIR/data.yaml`; plus DIR/export.json (sources, mapping, per-class counts).

Boxes are filtered by their size in the model's input (the dataset's `imgsz`, the TensorRT
engine's letterboxed square): one under --min-box-px there is too small to learn from.

Train/val split: consecutive frames are nearly identical, so frames are split in blocks of
--val-block sim seconds per session and camera rather than one by one; a block goes to val
with probability --val-fraction (deterministic, so re-exporting gives the same split).
--preview N also draws the boxes on N images into DIR/preview to check the labels by eye.
"""

import argparse
import collections
import glob
import json
import os
import shutil
import sys
import zlib

import yaml

CLASSES_YAML = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'classes.yaml')
SPLITS = {'train': 'train', 'val': 'valid'}   # split -> folder (Roboflow's names)
OUTPUTS = tuple(SPLITS.values()) + ('preview',)


def load_dataset(path, name):
    with open(path, encoding='utf-8') as f:
        cfg = yaml.safe_load(f)
    datasets = cfg.get('datasets', {})
    if name not in datasets:
        raise SystemExit(f'{path} has no dataset "{name}" (has: {", ".join(datasets)})')
    ds = datasets[name]
    classes = [str(c) for c in ds['classes']]
    mapping = {str(k): str(v) for k, v in ds['from_sim'].items()}
    unknown = sorted(set(mapping.values()) - set(classes))
    if unknown:
        raise SystemExit(f'{path}: dataset "{name}" maps to classes not in its class list: {unknown}')
    return classes, mapping, int(ds.get('imgsz', 640))


def find_sessions(paths):
    sessions = []
    for p in paths:
        if os.path.isfile(os.path.join(p, 'session.json')):
            sessions.append(os.path.abspath(p))
        else:
            sessions += sorted(os.path.dirname(os.path.abspath(s))
                               for s in glob.glob(os.path.join(p, '**', 'session.json'), recursive=True))
    return sorted(set(sessions))


def frames(session):
    for path in sorted(glob.glob(os.path.join(session, '*', '*.json'))):
        image = path[:-5] + '.jpg'
        if os.path.isfile(image):
            with open(path, encoding='utf-8') as f:
                yield image, json.load(f)


def is_val(session, camera, stamp, block, fraction):
    key = f'{os.path.basename(session)}/{camera}/{int(stamp // block)}'
    return zlib.crc32(key.encode()) / 2 ** 32 < fraction


def yolo_lines(frame, classes, mapping, imgsz, min_px, dropped):
    """YOLO label lines (class cx cy w h, normalized) for one frame's boxes."""
    w, h = frame['width'], frame['height']
    scale = imgsz / max(w, h)   # letterbox scale into the model input
    lines, kept = [], []
    for b in frame['boxes']:
        cls = mapping.get(b['label'])
        if cls is None:
            dropped[f'{b["label"]} (not in this dataset)'] += 1
            continue
        x0, y0 = max(0.0, b['x0']), max(0.0, b['y0'])
        x1, y1 = min(float(w), b['x1']), min(float(h), b['y1'])
        if min(x1 - x0, y1 - y0) * scale < min_px:
            dropped[f'{b["label"]} (under {min_px} px at {imgsz})'] += 1
            continue
        lines.append(f'{classes.index(cls)} {(x0 + x1) / 2 / w:.6f} {(y0 + y1) / 2 / h:.6f} '
                     f'{(x1 - x0) / w:.6f} {(y1 - y0) / h:.6f}')
        kept.append((cls, x0, y0, x1, y1))
    return lines, kept


def draw_preview(src, dst, kept):
    import cv2
    img = cv2.imread(src)
    for cls, x0, y0, x1, y1 in kept:
        cv2.rectangle(img, (int(x0), int(y0)), (int(x1), int(y1)), (0, 255, 255), 1)
        cv2.putText(img, cls, (int(x0), max(10, int(y0) - 3)), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)
    cv2.imwrite(dst, img)


def export(sessions, out, classes, mapping, imgsz=640, min_px=3, val_fraction=0.15, val_block=30.0, preview=0):
    for folder in SPLITS.values():
        os.makedirs(os.path.join(out, folder, 'images'), exist_ok=True)
        os.makedirs(os.path.join(out, folder, 'labels'), exist_ok=True)
    if preview:
        os.makedirs(os.path.join(out, 'preview'), exist_ok=True)
    counts = {'train': collections.Counter(), 'val': collections.Counter()}
    images = collections.Counter()
    dropped = collections.Counter()
    previews = 0
    for session in sessions:
        sname = os.path.basename(session)
        for src, frame in frames(session):
            split = 'val' if is_val(session, frame['camera'], frame['stamp'], val_block, val_fraction) else 'train'
            lines, kept = yolo_lines(frame, classes, mapping, imgsz, min_px, dropped)
            name = f'{sname}_{frame["camera"]}_{os.path.basename(src)[:-4]}'
            shutil.copyfile(src, os.path.join(out, SPLITS[split], 'images', name + '.jpg'))
            with open(os.path.join(out, SPLITS[split], 'labels', name + '.txt'), 'w', newline='\n') as f:
                f.write(''.join(line + '\n' for line in lines))
            images[split] += 1
            counts[split].update(k[0] for k in kept)
            if kept and previews < preview:
                draw_preview(src, os.path.join(out, 'preview', name + '.jpg'), kept)
                previews += 1
    with open(os.path.join(out, 'data.yaml'), 'w', newline='\n') as f:
        yaml.safe_dump({'path': os.path.abspath(out), 'train': 'train/images', 'val': 'valid/images',
                        'nc': len(classes), 'names': classes}, f, sort_keys=False)
    summary = {'sessions': sessions, 'classes': classes, 'from_sim': mapping, 'imgsz': imgsz, 'min_box_px': min_px,
               'val_fraction': val_fraction, 'val_block_s': val_block, 'images': dict(images),
               'boxes': {s: dict(c) for s, c in counts.items()}, 'dropped': dict(dropped)}
    with open(os.path.join(out, 'export.json'), 'w', newline='\n') as f:
        json.dump(summary, f, indent=2)
        f.write('\n')
    return summary


def _parse_args(argv):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('sessions', nargs='+', help='session folders from record, or folders containing them')
    ap.add_argument('--out', required=True, help='dataset folder to write')
    ap.add_argument('--dataset', default='main', help='dataset in classes.yaml (default main)')
    ap.add_argument('--classes', default=CLASSES_YAML, help='class mapping YAML (default: the packaged classes.yaml)')
    ap.add_argument('--min-box-px', type=float, default=3.0,
                    help="drop boxes narrower or shorter than this in the model input (px at the dataset's imgsz)")
    ap.add_argument('--val-fraction', type=float, default=0.15)
    ap.add_argument('--val-block', type=float, default=30.0, help='sim seconds per train/val block')
    ap.add_argument('--preview', type=int, default=0, help='draw the boxes on this many images into DIR/preview')
    ap.add_argument('--force', action='store_true', help='replace an existing export in DIR')
    return ap.parse_args(argv)


def main(argv=None):
    args = _parse_args(sys.argv[1:] if argv is None else argv)
    classes, mapping, imgsz = load_dataset(args.classes, args.dataset)
    sessions = find_sessions(args.sessions)
    if not sessions:
        print(f'No recorded sessions (session.json) under {", ".join(args.sessions)}', file=sys.stderr)
        return 1
    if os.path.exists(os.path.join(args.out, 'data.yaml')):
        if not args.force:
            print(f'{args.out} already holds an export; pass --force to replace it.', file=sys.stderr)
            return 1
        for d in OUTPUTS:
            shutil.rmtree(os.path.join(args.out, d), ignore_errors=True)
    s = export(sessions, args.out, classes, mapping, imgsz, args.min_box_px, args.val_fraction, args.val_block,
               args.preview)
    print(f'Exported {len(sessions)} session(s) to {args.out} (dataset {args.dataset})')
    print(f'  images: {s["images"].get("train", 0)} train, {s["images"].get("val", 0)} val')
    print(f'  {"class":20s} {"train":>7s} {"val":>7s}')
    for c in classes:
        print(f'  {c:20s} {s["boxes"]["train"].get(c, 0):7d} {s["boxes"]["val"].get(c, 0):7d}')
    for reason, n in sorted(s['dropped'].items()):
        print(f'  dropped {n:6d}  {reason}')
    if s['images'].get('train') and not s['images'].get('val'):
        print('  warning: no validation images; record longer or raise --val-fraction', file=sys.stderr)
    print(f'  Roboflow: ros2 run sim_autolabel upload {args.out} --workspace WS --project PROJECT')
    print(f'  or train directly: yolo train data={os.path.join(os.path.abspath(args.out), "data.yaml")} imgsz={imgsz}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
