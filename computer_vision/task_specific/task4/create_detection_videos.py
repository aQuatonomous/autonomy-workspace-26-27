#!/usr/bin/env python3
"""
Create detection videos for Task 4 supply drop detection.
Only creates output videos with detection overlays - no debug images.
"""

import cv2
import numpy as np
import os
from pathlib import Path
import argparse
from typing import List, Dict, Tuple
import time
import tempfile
from task4_simplified_pipeline import SimplifiedTask4Pipeline, SimplifiedConfig

class DetectionVideoCreator:
    """Creates detection videos without debug outputs"""
    
    def __init__(self, config: SimplifiedConfig = None):
        # Disable debug image saving
        self.config = config or SimplifiedConfig()
        self.config.SAVE_DEBUG_IMAGES = False
        self.config.DEBUG = False
        self.pipeline = SimplifiedTask4Pipeline(self.config)
    
    def create_detection_video(self, video_path: str, output_path: str, 
                              frame_skip: int = 5, max_frames: int = None) -> Dict:
        """
        Create a detection video with overlays
        
        Args:
            video_path: Path to input video
            output_path: Path for output detection video
            frame_skip: Process every Nth frame (default: 5)
            max_frames: Maximum frames to process (None = all)
        
        Returns:
            Dictionary with processing results
        """
        video_path = Path(video_path)
        output_path = Path(output_path)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        
        print(f"\nProcessing video: {video_path.name}")
        
        # Open input video
        cap = cv2.VideoCapture(str(video_path))
        if not cap.isOpened():
            raise ValueError(f"Could not open video: {video_path}")
        
        # Get video properties
        fps = cap.get(cv2.CAP_PROP_FPS)
        width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        frame_count = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
        duration = frame_count / fps if fps > 0 else 0
        
        print(f"  Input: {width}x{height}, {fps:.1f} FPS, {duration:.1f}s ({frame_count} frames)")
        print(f"  Processing every {frame_skip} frames")
        
        # Create output video writer
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        out = cv2.VideoWriter(str(output_path), fourcc, fps, (width, height))
        
        if not out.isOpened():
            raise ValueError(f"Could not create output video: {output_path}")
        
        # Processing stats
        results = {
            'video_name': video_path.name,
            'output_path': str(output_path),
            'fps': fps,
            'total_frames': frame_count,
            'duration': duration,
            'processed_frames': 0,
            'total_detections': 0,
            'processing_time': 0
        }
        
        frame_idx = 0
        processed_count = 0
        detection_cache = {}  # Cache detections for frames we process
        start_time = time.time()
        
        try:
            while True:
                ret, frame = cap.read()
                if not ret:
                    break
                
                # Check max_frames limit
                if max_frames and processed_count >= max_frames:
                    # Still need to write remaining frames without processing
                    out.write(frame)
                    frame_idx += 1
                    continue
                
                # Flip frame vertically to correct orientation (WhatsApp videos are often upside down)
                frame_flipped = cv2.flip(frame, 0)
                
                # Process detection on this frame if it's a skip frame
                detections = []
                if frame_idx % frame_skip == 0:
                    try:
                        # Create temporary file for this frame (use flipped frame for detection)
                        with tempfile.NamedTemporaryFile(suffix='.jpg', delete=False) as temp_file:
                            temp_path = temp_file.name
                            cv2.imwrite(temp_path, frame_flipped)
                        
                        # Create temporary output directory (won't be used since debug is off)
                        with tempfile.TemporaryDirectory() as temp_dir:
                            detections = self.pipeline.process_image(temp_path, temp_dir)
                        
                        # Clean up temp file
                        os.unlink(temp_path)
                        
                        detection_cache[frame_idx] = detections
                        processed_count += 1
                        results['total_detections'] += len(detections)
                        
                        if len(detections) > 0:
                            timestamp = frame_idx / fps if fps > 0 else frame_idx
                            print(f"    Frame {frame_idx} (t={timestamp:.1f}s): {len(detections)} detections")
                        
                    except Exception as e:
                        print(f"    Error processing frame {frame_idx}: {e}")
                        detection_cache[frame_idx] = []
                
                # Use cached detections if available, otherwise use empty list
                if frame_idx in detection_cache:
                    frame_detections = detection_cache[frame_idx]
                else:
                    # Find the most recent processed frame's detections
                    frame_detections = []
                    for i in range(frame_idx, -1, -1):
                        if i in detection_cache:
                            frame_detections = detection_cache[i]
                            break
                
                # Draw detections on flipped frame
                if frame_detections:
                    frame_flipped = self._draw_detections_on_frame(frame_flipped, frame_detections, frame_idx, fps)
                
                # Write flipped frame to output video
                out.write(frame_flipped)
                frame_idx += 1
                
        finally:
            cap.release()
            out.release()
        
        results['processed_frames'] = processed_count
        results['processing_time'] = time.time() - start_time
        
        print(f"  Completed: {processed_count} frames processed, {results['total_detections']} total detections")
        print(f"  Processing time: {results['processing_time']:.1f}s")
        print(f"  Output saved: {output_path}")
        
        return results
    
    def _draw_detections_on_frame(self, frame: np.ndarray, detections: List[Dict], 
                                 frame_idx: int, fps: float) -> np.ndarray:
        """Draw detection bounding boxes and info on frame"""
        # Draw detections
        for detection in detections:
            x1, y1, x2, y2 = detection['bbox']
            color = detection['color']
            confidence = detection['confidence']
            has_shape = detection['has_shape']
            
            # Choose color
            if color == 'yellow':
                draw_color = (0, 255, 255)  # Yellow in BGR
            else:  # black
                draw_color = (255, 255, 255)  # White in BGR
            
            # Draw bounding box
            thickness = 4 if has_shape else 2
            cv2.rectangle(frame, (x1, y1), (x2, y2), draw_color, thickness)
            
            # Draw label
            label = f"{color.upper()}"
            if has_shape:
                shape_type = "TRI" if color == 'yellow' else "CROSS"
                label += f"+{shape_type}"
            label += f" {confidence:.2f}"
            
            # Label background
            (label_w, label_h), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2)
            cv2.rectangle(frame, (x1, y1-label_h-10), (x1+label_w, y1), draw_color, -1)
            cv2.putText(frame, label, (x1, y1-5), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2)
        
        # Add frame info
        timestamp = frame_idx / fps if fps > 0 else frame_idx
        det_count = len(detections)
        info_text = f"t={timestamp:.1f}s | detections={det_count}"
        
        # Info background
        (info_w, info_h), _ = cv2.getTextSize(info_text, cv2.FONT_HERSHEY_SIMPLEX, 0.8, 2)
        cv2.rectangle(frame, (10, 10), (20+info_w, 20+info_h), (0, 0, 0), -1)
        cv2.putText(frame, info_text, (15, 15+info_h), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        
        return frame

def main():
    parser = argparse.ArgumentParser(description="Create detection videos for Task 4")
    parser.add_argument("--input", "-i", 
                       default="videos",
                       help="Input directory containing videos or single video file")
    parser.add_argument("--output", "-o",
                       default="detection_videos",
                       help="Output directory for detection videos")
    parser.add_argument("--frame-skip", "-f", type=int, default=5,
                       help="Process every Nth frame (default: 5)")
    parser.add_argument("--max-frames", "-m", type=int,
                       help="Maximum frames to process per video")
    parser.add_argument("--single", "-s",
                       help="Process single video file")
    
    args = parser.parse_args()
    
    creator = DetectionVideoCreator()
    
    try:
        if args.single:
            # Process single video
            video_path = Path(args.single)
            output_path = Path(args.output) / f"{video_path.stem}_detections.mp4"
            
            results = creator.create_detection_video(
                str(video_path), str(output_path), 
                args.frame_skip, args.max_frames
            )
            
        else:
            # Process all videos in directory
            input_path = Path(args.input)
            
            if not input_path.exists():
                raise ValueError(f"Input path does not exist: {input_path}")
            
            # Find video files
            video_extensions = {'.mp4', '.avi', '.mov', '.mkv', '.wmv'}
            if input_path.is_file():
                video_files = [input_path]
            else:
                video_files = [f for f in input_path.iterdir() 
                              if f.suffix.lower() in video_extensions]
            
            if not video_files:
                raise ValueError(f"No video files found in {input_path}")
            
            print(f"Found {len(video_files)} videos to process")
            
            output_dir = Path(args.output)
            output_dir.mkdir(parents=True, exist_ok=True)
            
            all_results = {}
            
            for video_file in sorted(video_files):
                output_path = output_dir / f"{video_file.stem}_detections.mp4"
                
                try:
                    results = creator.create_detection_video(
                        str(video_file), str(output_path),
                        args.frame_skip, args.max_frames
                    )
                    all_results[video_file.name] = results
                        
                except Exception as e:
                    print(f"Error processing {video_file.name}: {e}")
                    all_results[video_file.name] = {'error': str(e)}
            
            # Overall summary
            print(f"\n{'='*60}")
            print("DETECTION VIDEO CREATION SUMMARY")
            print(f"{'='*60}")
            
            total_detections = 0
            successful_videos = 0
            
            for video_name, results in all_results.items():
                if 'error' in results:
                    print(f"ERROR {video_name}: {results['error']}")
                else:
                    successful_videos += 1
                    detections = results['total_detections']
                    total_detections += detections
                    duration = results['processing_time']
                    
                    print(f"SUCCESS {video_name}: {detections} detections ({duration:.1f}s)")
                    print(f"   Output: {Path(results['output_path']).name}")
            
            print(f"\nSummary: {successful_videos}/{len(video_files)} videos processed successfully")
            print(f"Total detections: {total_detections}")
            print(f"Detection videos saved to: {args.output}")
    
    except Exception as e:
        print(f"Error: {e}")
        return 1
    
    return 0

if __name__ == "__main__":
    exit(main())