"""Uploads an export to the CV team's Roboflow project, labels included.

  ros2 run sim_autolabel upload DIR --workspace WS --project PROJECT [--limit 20]
  python3 -m sim_autolabel.upload ...      # same, without ROS (needs `pip install roboflow`)

DIR is a folder written by `export`. The API key comes from $ROBOFLOW_API_KEY (Roboflow:
Settings > API Keys). Images keep their train/valid split, go into one upload batch (--batch,
default "sim <DIR name>") and are tagged `sim` and `sim_seed<N>`, so the CV team can filter the
sim images in or out when they generate a dataset version.

The project must already use the dataset's class names (classes.yaml): a different spelling
would make new classes in the project, so the upload stops unless --new-classes is given.
Try a few images first with --limit and look at them in Roboflow before sending everything.
"""

import argparse
import glob
import os
import re
import sys

import yaml

from sim_autolabel.export import SPLITS


def items(dataset):
    """(split folder, image, label file) for every image in an export."""
    for folder in SPLITS.values():
        for image in sorted(glob.glob(os.path.join(dataset, folder, 'images', '*.jpg'))):
            label = os.path.join(dataset, folder, 'labels', os.path.basename(image)[:-4] + '.txt')
            if os.path.isfile(label):
                yield folder, image, label


def image_tags(image, extra):
    m = re.search(r'_seed(\d+)_', os.path.basename(image))
    return ['sim'] + ([f'sim_seed{m.group(1)}'] if m else []) + list(extra)


def missing_classes(project_classes, classes):
    """Dataset classes the Roboflow project does not have yet (none if the project is still empty)."""
    have = set(project_classes or ())
    return [c for c in classes if c not in have] if have else []


def _parse_args(argv):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('dataset', help='folder written by export')
    ap.add_argument('--workspace', required=True, help='Roboflow workspace id (from the project URL)')
    ap.add_argument('--project', required=True, help='Roboflow project id (from the project URL)')
    ap.add_argument('--batch', default=None, help='upload batch name (default "sim <DIR name>")')
    ap.add_argument('--tag', nargs='*', default=[], help='extra tags for every image')
    ap.add_argument('--limit', type=int, default=0, help='upload only this many images (0 = all)')
    ap.add_argument('--new-classes', action='store_true',
                    help='upload even if the project lacks some of the dataset classes')
    ap.add_argument('--retries', type=int, default=3)
    return ap.parse_args(argv)


def main(argv=None):
    args = _parse_args(sys.argv[1:] if argv is None else argv)
    data_yaml = os.path.join(args.dataset, 'data.yaml')
    if not os.path.isfile(data_yaml):
        print(f'{args.dataset} is not an export (no data.yaml); run `export` first.', file=sys.stderr)
        return 1
    with open(data_yaml, encoding='utf-8') as f:
        classes = [str(c) for c in yaml.safe_load(f)['names']]
    todo = list(items(args.dataset))
    if args.limit:
        todo = todo[:args.limit]
    if not todo:
        print(f'No images under {args.dataset}', file=sys.stderr)
        return 1
    api_key = os.environ.get('ROBOFLOW_API_KEY')
    if not api_key:
        print('Set ROBOFLOW_API_KEY (Roboflow: Settings > API Keys).', file=sys.stderr)
        return 1
    try:
        from roboflow import Roboflow
    except ImportError:
        print('Needs the Roboflow client: pip install roboflow', file=sys.stderr)
        return 1

    project = Roboflow(api_key=api_key).workspace(args.workspace).project(args.project)
    missing = missing_classes(getattr(project, 'classes', None), classes)
    if missing and not args.new_classes:
        print(f'Project {args.project} has no class {", ".join(missing)} (it has: {", ".join(project.classes)}).\n'
              'Fix the names in classes.yaml and export again, or pass --new-classes to create them.',
              file=sys.stderr)
        return 1

    batch = args.batch or f'sim {os.path.basename(os.path.abspath(args.dataset))}'
    done, failed = 0, []
    for i, (folder, image, label) in enumerate(todo, 1):
        try:
            project.single_upload(image_path=image, annotation_path=label, annotation_labelmap=data_yaml,
                                  split=folder, batch_name=batch, tag_names=image_tags(image, args.tag),
                                  num_retry_uploads=args.retries)
            done += 1
        except Exception as e:  # noqa: BLE001 - keep going, report at the end
            failed.append((image, str(e)))
            if not done and len(failed) >= 5:
                print(f'First {len(failed)} uploads failed, stopping: {e}', file=sys.stderr)
                break
        if i % 50 == 0:
            print(f'  {i}/{len(todo)} ({len(failed)} failed)')
    print(f'Uploaded {done} of {len(todo)} images to {args.workspace}/{args.project}, batch "{batch}"')
    for image, err in failed[:10]:
        print(f'  failed {os.path.basename(image)}: {err}', file=sys.stderr)
    return 0 if not failed else 1


if __name__ == '__main__':
    sys.exit(main())
