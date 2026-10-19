#!/usr/bin/env python3
"""
Process video with colour indicator buoy detector.
Runs the detector on each frame and creates an output video with annotations.
"""

import cv2
import numpy as np
from pathlib import Path
import argparse
import sys
from colour_indicator_buoy_detector import classify_colour_indicator_buoy

def process_video(input_path: str, output_path: str = None, skip_frames: int = 0):
    """
    Process video with colour indicator buoy detector.
    
    Args:
        input_path: Path to input video
        output_path: Path to output video (default: input_name_processed.mp4)
        skip_frames: Process every Nth frame (0 = process all frames)
    """
    input_path = Path(input_path)
    if not input_path.exists():
        print(f"Error: Input video not found: {input_path}")
        return False
    
    if output_path is None:
        output_path = input_path.parent / f"{input_path.stem}_processed.mp4"
    else:
        output_path = Path(output_path)
    
    print(f"Processing video: {input_path}")
    print(f"Output will be saved to: {output_path}")
    
    # Open input video
    cap = cv2.VideoCapture(str(input_path))
    if not cap.isOpened():
        print(f"Error: Could not open video: {input_path}")
        return False
    
    # Get video properties
    fps = int(cap.get(cv2.CAP_PROP_FPS))
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    
    print(f"Video properties: {width}x{height}, {fps} FPS, {total_frames} frames")
    
    # Setup video writer
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out = cv2.VideoWriter(str(output_path), fourcc, fps, (width, height))
    
    if not out.isOpened():
        print(f"Error: Could not create output video writer")
        cap.release()
        return False
    
    frame_count = 0
    processed_count = 0
    detection_count = 0
    
    print(f"Processing frames...")
    
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                break
            
            frame_count += 1
            
            # Skip frames if specified
            if skip_frames > 0 and (frame_count - 1) % (skip_frames + 1) != 0:
                # Write original frame for skipped frames
                out.write(frame)
                continue
            
            processed_count += 1
            
            # Run colour indicator buoy detector
            try:
                annotated_frame, info = classify_colour_indicator_buoy(
                    frame,
                    conf_threshold=0.6,  # Increased diamond confidence threshold
                    max_black_brightness=230,  # Updated for lighter diamonds with glare
                    roi_conf_threshold=0.6,
                    buoy_conf_threshold=0.3,
                    white_blob_expansion=2.0,  # Reduced from 4.0 for tighter bounding boxes
                    min_white_brightness=100,
                    min_white_blob_score=0.15,
                )
                
                buoys = info.get("buoys", [])
                if buoys:
                    detection_count += 1
                
                # Add frame info overlay
                cv2.putText(
                    annotated_frame,
                    f"Frame: {frame_count}/{total_frames} | Buoys: {len(buoys)}",
                    (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (255, 255, 255),
                    2,
                    cv2.LINE_AA,
                )
                
                # Add detection summary
                if buoys:
                    for i, buoy in enumerate(buoys):
                        state = buoy["indicator_state"]
                        conf = buoy["indicator_conf"]
                        buoy_conf = buoy["buoy_confidence"]
                        
                        text = f"Buoy {i+1}: {state.upper()} (conf:{conf:.2f}, buoy:{buoy_conf:.2f})"
                        cv2.putText(
                            annotated_frame,
                            text,
                            (10, 60 + i * 25),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.5,
                            (0, 255, 0) if state == "green" else (0, 0, 255) if state == "red" else (255, 255, 255),
                            1,
                            cv2.LINE_AA,
                        )
                
                out.write(annotated_frame)
                
            except Exception as e:
                print(f"Error processing frame {frame_count}: {e}")
                # Write original frame on error
                out.write(frame)
            
            # Progress update
            if frame_count % 100 == 0:
                progress = (frame_count / total_frames) * 100
                print(f"Progress: {frame_count}/{total_frames} frames ({progress:.1f}%) - "
                      f"Processed: {processed_count}, Detections: {detection_count}")
    
    except KeyboardInterrupt:
        print("\nProcessing interrupted by user")
    
    finally:
        # Cleanup
        cap.release()
        out.release()
        
        print(f"\nProcessing complete!")
        print(f"Total frames: {frame_count}")
        print(f"Processed frames: {processed_count}")
        print(f"Frames with detections: {detection_count}")
        print(f"Detection rate: {(detection_count/processed_count)*100:.1f}%")
        print(f"Output saved to: {output_path}")
        
        return True

def main():
    parser = argparse.ArgumentParser(
        description="Process video with colour indicator buoy detector"
    )
    parser.add_argument(
        "input_video",
        help="Path to input video file"
    )
    parser.add_argument(
        "--output", "-o",
        help="Path to output video file (default: input_name_processed.mp4)"
    )
    parser.add_argument(
        "--skip-frames", "-s",
        type=int,
        default=0,
        help="Process every Nth frame (0 = process all frames, 1 = every other frame, etc.)"
    )
    
    args = parser.parse_args()
    
    success = process_video(args.input_video, args.output, args.skip_frames)
    return 0 if success else 1

if __name__ == "__main__":
    sys.exit(main())