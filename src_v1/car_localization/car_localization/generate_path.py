#!/usr/bin/env python3
"""
Generate a test path file for path_executor.

Usage:
  python3 generate_path.py [output_file]

Generates a simple straight-line path for testing.
Starts from origin and moves forward along Y axis.
"""

import math
import sys
import os

# Map constants
BLOCK_SIZE = 0.35  # meters per block


def block_to_meters(block_name):
    """Convert block name to meters coordinates."""
    col_letters = 'ABCDEFGHIJKLMN'
    letter = block_name[0].upper()
    number = int(block_name[1:])
    
    col_idx = col_letters.index(letter)
    row_idx = number - 1
    
    x = (10.5 - row_idx) * BLOCK_SIZE
    y = (13.5 - col_idx) * BLOCK_SIZE
    
    return x, y


def yaw_to_quaternion(yaw):
    """Convert yaw angle to quaternion (qx, qy, qz, qw)."""
    qz = math.sin(yaw / 2)
    qw = math.cos(yaw / 2)
    return 0.0, 0.0, qz, qw


def generate_straight_path(start_x, start_y, length, yaw, step=0.05):
    """Generate a straight line path.
    
    Args:
        start_x: Starting X coordinate (meters)
        start_y: Starting Y coordinate (meters)
        length: Path length (meters)
        yaw: Direction yaw (radians)
        step: Distance between path points (meters)
    """
    points = []
    qx, qy, qz, qw = yaw_to_quaternion(yaw)
    
    num_points = int(length / step) + 1
    
    for i in range(num_points):
        t = i * step
        x = start_x + t * math.cos(yaw)
        y = start_y + t * math.sin(yaw)
        
        points.append(f"{x:.4f},{y:.4f},0.0000,{qx:.4f},{qy:.4f},{qz:.4f},{qw:.4f},")
    
    return points


def generate_waypoint_path(start_block, waypoints, step=0.05):
    """Generate a path through multiple waypoints (block names).
    
    Args:
        start_block: Starting block name (e.g., 'N10')
        waypoints: List of (block_name, heading_dir) tuples
        step: Distance between path points (meters)
    """
    points = []
    current_x, current_y = block_to_meters(start_block)
    
    for block_name, heading_dir in waypoints:
        target_x, target_y = block_to_meters(block_name)
        
        # Calculate direction to target
        dx = target_x - current_x
        dy = target_y - current_y
        distance = math.sqrt(dx * dx + dy * dy)
        
        if distance < 0.01:
            current_x, current_y = target_x, target_y
            continue
        
        path_yaw = math.atan2(dy, dx)
        
        # Generate points along the path
        qx, qy, qz, qw = yaw_to_quaternion(path_yaw)
        num_points = int(distance / step) + 1
        
        for i in range(1, num_points + 1):
            t = min(i * step, distance)
            x = current_x + t * math.cos(path_yaw)
            y = current_y + t * math.sin(path_yaw)
            
            points.append(f"{x:.4f},{y:.4f},0.0000,{qx:.4f},{qy:.4f},{qz:.4f},{qw:.4f},")
        
        # Update current position
        current_x, current_y = target_x, target_y
    
    return points


def main():
    output_file = "test_path.txt"
    
    if len(sys.argv) > 1:
        output_file = sys.argv[1]
    
    # Example: Generate path from N10 to M10
    start_block = "N10"
    waypoints = [
        ("M10", "y"),  # Go to M10, facing Y+
    ]
    
    points = generate_waypoint_path(start_block, waypoints)
    
    # Write to file
    with open(output_file, 'w') as f:
        for point in points:
            f.write(point + '\n')
    
    print(f"Generated {len(points)} path points to {output_file}")
    print(f"Start: {start_block}")
    print(f"Waypoints: {waypoints}")
    
    # Also print meters coordinates
    start_x, start_y = block_to_meters(start_block)
    print(f"\nStart position: ({start_x:.2f}m, {start_y:.2f}m)")
    for block_name, heading_dir in waypoints:
        target_x, target_y = block_to_meters(block_name)
        print(f"Waypoint {block_name} ({heading_dir}): ({target_x:.2f}m, {target_y:.2f}m)")


if __name__ == "__main__":
    main()