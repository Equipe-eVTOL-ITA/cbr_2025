#!/usr/bin/env python3
"""
HSV Color Debug Tool for Landing Pad Detection

This script helps debug and tune HSV color ranges for landing pad detection.
It provides visual feedback for blue square detection and yellow circle/cross detection.

Usage:
    python3 base_detector_debug.py [--params /path/to/params.yaml] [--image /path/to/image.jpg]
"""

import cv2
import numpy as np
import matplotlib.pyplot as plt
import argparse
import yaml
import os
from typing import Dict, Tuple, List


class BaseDetectorDebug:
    """Debug tool for landing pad HSV color tuning"""
    
    def __init__(self, params_file=None):
        self.script_dir = os.path.dirname(os.path.realpath(__file__))
        
        # Default parameters (same as in base_detector.py)
        self.params = {
            'blue_lower_h': 100, 'blue_lower_s': 120, 'blue_lower_v': 100,
            'blue_upper_h': 130, 'blue_upper_s': 255, 'blue_upper_v': 255,
            'yellow_lower_h': 20, 'yellow_lower_s': 100, 'yellow_lower_v': 100,
            'yellow_upper_h': 30, 'yellow_upper_s': 255, 'yellow_upper_v': 255,
            'morph_kernel_size': 5,
            'morph_iterations': 2,
            'min_pad_area': 5000,
            'max_pad_area': 50000,
            'min_aspect_ratio': 0.7,
            'max_aspect_ratio': 1.3
        }
        
        # Load parameters from file if provided
        if params_file and os.path.exists(params_file):
            self.load_params_from_file(params_file)
    
    def load_params_from_file(self, params_file: str):
        """Load parameters from YAML file"""
        try:
            with open(params_file, 'r') as f:
                yaml_data = yaml.safe_load(f)
                
            if 'base_detector' in yaml_data and 'ros__parameters' in yaml_data['base_detector']:
                detector_params = yaml_data['base_detector']['ros__parameters']
                
                # Update parameters
                for key, value in detector_params.items():
                    if key in self.params:
                        self.params[key] = value
                        
                print(f"Loaded parameters from {params_file}")
            else:
                print(f"No base_detector parameters found in {params_file}")
                
        except Exception as e:
            print(f"Error loading parameters: {e}")
    
    def create_color_ranges(self) -> Dict:
        """Create color ranges from parameters"""
        return {
            'blue': {
                'lower': np.array([self.params['blue_lower_h'], 
                                 self.params['blue_lower_s'], 
                                 self.params['blue_lower_v']]),
                'upper': np.array([self.params['blue_upper_h'], 
                                 self.params['blue_upper_s'], 
                                 self.params['blue_upper_v']])
            },
            'yellow': {
                'lower': np.array([self.params['yellow_lower_h'], 
                                 self.params['yellow_lower_s'], 
                                 self.params['yellow_lower_v']]),
                'upper': np.array([self.params['yellow_upper_h'], 
                                 self.params['yellow_upper_s'], 
                                 self.params['yellow_upper_v']])
            }
        }
    
    def segment_color(self, hsv_image: np.ndarray, color_name: str) -> np.ndarray:
        """Segment a specific color from HSV image"""
        color_ranges = self.create_color_ranges()
        
        if color_name not in color_ranges:
            return np.zeros(hsv_image.shape[:2], dtype=np.uint8)
        
        color_range = color_ranges[color_name]
        mask = cv2.inRange(hsv_image, color_range['lower'], color_range['upper'])
        
        # Apply morphological operations
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, 
                                         (self.params['morph_kernel_size'], 
                                          self.params['morph_kernel_size']))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, 
                               iterations=self.params['morph_iterations'])
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
        
        return mask
    
    def find_landing_pad_candidates(self, image: np.ndarray) -> Tuple[List[Dict], np.ndarray]:
        """Find landing pad candidates in image"""
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        debug_image = image.copy()
        
        # Segment blue squares
        blue_mask = self.segment_color(hsv, 'blue')
        yellow_mask = self.segment_color(hsv, 'yellow')
        
        # Find contours
        blue_contours, _ = cv2.findContours(blue_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        candidates = []
        
        for i, contour in enumerate(blue_contours):
            x, y, w, h = cv2.boundingRect(contour)
            area = w * h
            aspect_ratio = w / h
            
            # Check constraints
            area_valid = self.params['min_pad_area'] <= area <= self.params['max_pad_area']
            aspect_valid = self.params['min_aspect_ratio'] <= aspect_ratio <= self.params['max_aspect_ratio']
            
            # Color for drawing
            color = (0, 255, 0) if (area_valid and aspect_valid) else (0, 0, 255)
            
            # Draw rectangle
            cv2.rectangle(debug_image, (x, y), (x+w, y+h), color, 2)
            
            # Add text
            status = "VALID" if (area_valid and aspect_valid) else "INVALID"
            cv2.putText(debug_image, f"{status} {i}", (x, y-10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
            cv2.putText(debug_image, f"A:{area:.0f} AR:{aspect_ratio:.2f}", (x, y+h+20), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
            
            # Extract yellow ROI
            roi_yellow = yellow_mask[y:y+h, x:x+w] if y+h <= yellow_mask.shape[0] and x+w <= yellow_mask.shape[1] else None
            
            candidates.append({
                'bbox': (x, y, w, h),
                'area': area,
                'aspect_ratio': aspect_ratio,
                'area_valid': area_valid,
                'aspect_valid': aspect_valid,
                'yellow_roi': roi_yellow
            })
        
        return candidates, debug_image
    
    def create_visualization(self, image: np.ndarray) -> np.ndarray:
        """Create comprehensive visualization"""
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # Create masks
        blue_mask = self.segment_color(hsv, 'blue')
        yellow_mask = self.segment_color(hsv, 'yellow')
        
        # Find candidates
        candidates, debug_image = self.find_landing_pad_candidates(image)
        
        # Create multi-panel visualization
        h, w = image.shape[:2]
        
        # Resize for display
        display_size = (400, 300)
        image_resized = cv2.resize(image, display_size)
        debug_resized = cv2.resize(debug_image, display_size)
        blue_mask_resized = cv2.resize(blue_mask, display_size)
        yellow_mask_resized = cv2.resize(yellow_mask, display_size)
        
        # Convert masks to 3-channel for display
        blue_mask_color = cv2.cvtColor(blue_mask_resized, cv2.COLOR_GRAY2BGR)
        blue_mask_color[:,:,0] = 0  # Remove red channel
        blue_mask_color[:,:,1] = 0  # Remove green channel
        
        yellow_mask_color = cv2.cvtColor(yellow_mask_resized, cv2.COLOR_GRAY2BGR)
        yellow_mask_color[:,:,2] = 0  # Remove blue channel
        yellow_mask_color[:,:,0] = 0  # Remove red channel
        
        # Combine masks
        combined_mask = cv2.addWeighted(blue_mask_color, 0.7, yellow_mask_color, 0.7, 0)
        
        # Create 2x2 grid
        top_row = np.hstack([image_resized, debug_resized])
        bottom_row = np.hstack([blue_mask_color, yellow_mask_color])
        
        final_viz = np.vstack([top_row, bottom_row])
        
        # Add labels
        cv2.putText(final_viz, "Original", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(final_viz, "Detections", (display_size[0]+10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(final_viz, "Blue Mask", (10, display_size[1]+25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(final_viz, "Yellow Mask", (display_size[0]+10, display_size[1]+25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        return final_viz
    
    def create_hsv_visualization(self):
        """Create HSV range visualization"""
        fig, axes = plt.subplots(2, 2, figsize=(15, 10))
        fig.suptitle('Landing Pad HSV Color Ranges', fontsize=16, fontweight='bold')
        
        # Blue color samples
        self.plot_color_samples(axes[0, 0], 'Blue Square', 
                               self.params['blue_lower_h'], self.params['blue_upper_h'],
                               self.params['blue_lower_s'], self.params['blue_upper_s'],
                               self.params['blue_lower_v'], self.params['blue_upper_v'])
        
        # Yellow color samples
        self.plot_color_samples(axes[0, 1], 'Yellow Circle/Cross',
                               self.params['yellow_lower_h'], self.params['yellow_upper_h'],
                               self.params['yellow_lower_s'], self.params['yellow_upper_s'],
                               self.params['yellow_lower_v'], self.params['yellow_upper_v'])
        
        # HSV ranges as text
        axes[1, 0].text(0.1, 0.8, self.get_parameters_text(), fontsize=10, 
                       verticalalignment='top', fontfamily='monospace')
        axes[1, 0].set_title('Current Parameters')
        axes[1, 0].axis('off')
        
        # Detection constraints
        constraints_text = f"""Detection Constraints:
Min Pad Area: {self.params['min_pad_area']}
Max Pad Area: {self.params['max_pad_area']}
Min Aspect Ratio: {self.params['min_aspect_ratio']}
Max Aspect Ratio: {self.params['max_aspect_ratio']}

Morphology:
Kernel Size: {self.params['morph_kernel_size']}
Iterations: {self.params['morph_iterations']}"""
        
        axes[1, 1].text(0.1, 0.8, constraints_text, fontsize=10,
                       verticalalignment='top', fontfamily='monospace')
        axes[1, 1].set_title('Detection Constraints')
        axes[1, 1].axis('off')
        
        plt.tight_layout()
        return fig
    
    def plot_color_samples(self, ax, title, h_min, h_max, s_min, s_max, v_min, v_max):
        """Plot color samples for given HSV range"""
        samples_per_dim = 8
        
        # Generate samples
        if h_min <= h_max:
            h_samples = np.linspace(h_min, h_max, samples_per_dim)
        else:
            h_samples = np.concatenate([
                np.linspace(h_min, 180, samples_per_dim//2),
                np.linspace(0, h_max, samples_per_dim//2)
            ])
        
        s_samples = np.linspace(s_min, s_max, samples_per_dim)
        v_samples = np.linspace(v_min, v_max, samples_per_dim)
        
        # Create color grid
        color_grid = np.zeros((samples_per_dim, samples_per_dim, 3), dtype=np.uint8)
        
        for i, s in enumerate(s_samples):
            for j, v in enumerate(v_samples):
                # Use middle hue value
                h = h_samples[len(h_samples)//2]
                color_grid[i, j] = [h, s, v]
        
        # Convert to RGB for display
        color_grid_rgb = cv2.cvtColor(color_grid, cv2.COLOR_HSV2RGB)
        
        ax.imshow(color_grid_rgb, aspect='equal')
        ax.set_title(title)
        ax.set_xlabel('Value (Brightness)')
        ax.set_ylabel('Saturation')
        
        # Add range text
        range_text = f"H: {h_min}-{h_max}\nS: {s_min}-{s_max}\nV: {v_min}-{v_max}"
        ax.text(0.02, 0.98, range_text, transform=ax.transAxes, 
               verticalalignment='top', bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))
    
    def get_parameters_text(self) -> str:
        """Get formatted parameters text"""
        return f"""Blue HSV Range:
  H: {self.params['blue_lower_h']} - {self.params['blue_upper_h']}
  S: {self.params['blue_lower_s']} - {self.params['blue_upper_s']}
  V: {self.params['blue_lower_v']} - {self.params['blue_upper_v']}

Yellow HSV Range:
  H: {self.params['yellow_lower_h']} - {self.params['yellow_upper_h']}
  S: {self.params['yellow_lower_s']} - {self.params['yellow_upper_s']}
  V: {self.params['yellow_lower_v']} - {self.params['yellow_upper_v']}"""
    
    def save_debug_outputs(self, output_dir: str = None):
        """Save debug visualizations"""
        if output_dir is None:
            output_dir = os.path.join(self.script_dir, 'debug_output')
        
        os.makedirs(output_dir, exist_ok=True)
        
        # Save HSV visualization
        fig = self.create_hsv_visualization()
        hsv_path = os.path.join(output_dir, 'base_detector_hsv_ranges.png')
        fig.savefig(hsv_path, dpi=300, bbox_inches='tight')
        plt.close(fig)
        
        print(f"HSV visualization saved to: {hsv_path}")
        
        # Save parameters
        params_path = os.path.join(output_dir, 'base_detector_parameters.txt')
        with open(params_path, 'w') as f:
            f.write("Base Detector Parameters\n")
            f.write("=" * 30 + "\n\n")
            f.write(self.get_parameters_text())
            f.write("\n\nDetection Constraints:\n")
            f.write(f"Min Pad Area: {self.params['min_pad_area']}\n")
            f.write(f"Max Pad Area: {self.params['max_pad_area']}\n")
            f.write(f"Min Aspect Ratio: {self.params['min_aspect_ratio']}\n")
            f.write(f"Max Aspect Ratio: {self.params['max_aspect_ratio']}\n")
        
        print(f"Parameters saved to: {params_path}")


def main():
    parser = argparse.ArgumentParser(description='Debug HSV color ranges for landing pad detection')
    parser.add_argument('--params', type=str, 
                       default='/home/ceccon/frtl_2025_ws/src/cbr_2025/fase1/launch/params.yaml',
                       help='Path to parameters YAML file')
    parser.add_argument('--image', type=str, help='Path to test image')
    parser.add_argument('--output', type=str, help='Output directory for debug files')
    
    args = parser.parse_args()
    
    # Create debugger
    debugger = BaseDetectorDebug(args.params)
    
    # Save debug outputs
    debugger.save_debug_outputs(args.output)
    
    # If image provided, test on it
    if args.image and os.path.exists(args.image):
        image = cv2.imread(args.image)
        if image is not None:
            viz = debugger.create_visualization(image)
            
            # Save image visualization
            output_dir = args.output if args.output else os.path.join(os.path.dirname(args.image), 'debug_output')
            os.makedirs(output_dir, exist_ok=True)
            
            viz_path = os.path.join(output_dir, 'landing_pad_detection_debug.png')
            cv2.imwrite(viz_path, viz)
            print(f"Image debug visualization saved to: {viz_path}")
            
            # Show interactive window
            cv2.imshow('Landing Pad Detection Debug', viz)
            print("Press any key to close...")
            cv2.waitKey(0)
            cv2.destroyAllWindows()
        else:
            print(f"Could not load image: {args.image}")


if __name__ == '__main__':
    main()
