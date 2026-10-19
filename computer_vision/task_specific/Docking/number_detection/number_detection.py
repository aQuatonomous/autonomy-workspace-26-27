import argparse
from pathlib import Path

import cv2
from ultralytics import YOLO


def main():
    parser = argparse.ArgumentParser(description="Run the trained number detector (YOLO) on an image.")
    parser.add_argument("--model", default="best.pt", help="Path to model weights (.pt)")
    parser.add_argument("--source", default="test_numbers.png", help="Input image path")
    parser.add_argument("--conf", type=float, default=0.25, help="Confidence threshold")
    parser.add_argument("--imgsz", type=int, default=960, help="Inference image size")
    parser.add_argument("--save-dir", default="runs_number_detection", help="Directory to save predictions")
    parser.add_argument("--show", action="store_true", help="Show annotated result (requires GUI/display)")
    args = parser.parse_args()

    model_path = Path(args.model)
    if not model_path.exists():
        raise SystemExit(f"Model not found: {model_path}")

    yolo = YOLO(str(model_path))
    results = yolo.predict(
        source=args.source,
        conf=args.conf,
        imgsz=args.imgsz,
        save=True,
        project=args.save_dir,
        name="predict",
        exist_ok=True,
        verbose=False,
    )

    for r in results:
        names = r.names or {}
        print(f"\nSource: {r.path}")
        if r.boxes is None or len(r.boxes) == 0:
            print("  (no detections)")
            continue
        for b in r.boxes:
            cls_id = int(b.cls.item())
            conf = float(b.conf.item())
            xyxy = [float(x) for x in b.xyxy.squeeze(0).tolist()]
            label = names.get(cls_id, str(cls_id))
            print(f"  {label}: conf={conf:.3f}, xyxy={xyxy}")

        if args.show:
            annotated = r.plot()
            cv2.imshow("prediction", annotated)
            cv2.waitKey(0)
            cv2.destroyAllWindows()

if __name__ == "__main__":
    main()