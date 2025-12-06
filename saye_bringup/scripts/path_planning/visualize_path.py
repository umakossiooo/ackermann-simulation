#!/usr/bin/env python3
"""
Visualization script for Dijkstra and A* path planners.
This script generates a static map view comparing the paths from both algorithms.

USAGE:
    1. Ensure the Docker container is rebuilt to include matplotlib:
       docker compose build ackermann_sim
    2. Run this script inside the container:
       python3 src/ackermann-vehicle-gzsim-ros2/saye_bringup/scripts/path_planning/visualize_path.py
"""

import sys
import os
import matplotlib.pyplot as plt
from pathlib import Path

# Add current directory to path to allow imports of local modules
current_dir = Path(__file__).parent
sys.path.append(str(current_dir))

try:
    from dijkstra_path_planner import DijkstraPlanner
    from astar_path_planner import AStarPlanner
    from path_planner_utils import find_maps_directory, ROUTE_GOALS
except ImportError as e:
    print(f"Error importing planners: {e}")
    print("Ensure you are running inside the container where ROS 2 packages are built/sourced.")
    sys.exit(1)

def main():
    print("--- Path Planning Visualization ---")
    
    # 1. Setup Map Paths
    try:
        maps_dir = find_maps_directory()
    except Exception as e:
        print(f"Error finding maps: {e}")
        # Fallback path inside the container
        maps_dir = Path('/root/colcon_ws/src/map2gazebo/maps')
    
    edges_file = str(maps_dir / 'edges.json')
    map_file = str(maps_dir / 'map.json')
    polygons_file = str(maps_dir / 'road_polygons_merged.json')
    
    if not os.path.exists(edges_file):
        print(f"Error: {edges_file} not found!")
        print("Check if map2gazebo/maps is mounted correctly in docker-compose.yaml")
        sys.exit(1)

    print(f"Loading map from: {maps_dir}")

    # 2. Initialize Planners
    print("Initializing Dijkstra and A* planners...")
    dijkstra = DijkstraPlanner(edges_file, map_file, polygons_file)
    astar = AStarPlanner(edges_file, map_file, polygons_file)

    # 3. Define Start and Goal
    # Fixed start point as requested
    start = (5.55, -94.69)
    
    # Use the active goal from config
    if ROUTE_GOALS:
        goal = ROUTE_GOALS[0]
    else:
        goal = (0, 0)
        print("Warning: No goal found in ROUTE_GOALS, using (0,0)")

    print(f"Start: {start}")
    print(f"Goal:  {goal}")

    # 4. Compute Paths
    print("Calculating Dijkstra path...")
    d_path, d_metrics = dijkstra.search(start, goal)
    
    print("Calculating A* path...")
    a_path, a_metrics = astar.search(start, goal)

    # 5. Verification Output
    print("\n--- RESULTS ---")
    if d_path:
        print(f"Dijkstra: Found path with {len(d_path)} waypoints. Cost: {d_metrics['path_cost']:.2f}m")
    else:
        print("Dijkstra: No path found!")

    if a_path:
        print(f"A*:       Found path with {len(a_path)} waypoints. Cost: {a_metrics['path_cost']:.2f}m")
    else:
        print("A*:       No path found!")

    if d_path and a_path:
        length_diff = abs(d_metrics['path_cost'] - a_metrics['path_cost'])
        if length_diff < 0.01:
            print("SUCCESS: Both algorithms found optimal paths of identical length.")
        else:
            print(f"WARNING: Paths differ in length by {length_diff:.4f}m")

    # 6. Visualize
    print("\nGenerating static map plot...")
    plt.figure(figsize=(15, 15))
    
    # Plot Map Edges (Road Network)
    # We iterate over unique edges to draw the road graph
    visited_edges = set()
    for node, neighbors in dijkstra.graph.items():
        for neighbor, dist in neighbors:
            edge_key = tuple(sorted((node, neighbor)))
            if edge_key not in visited_edges:
                plt.plot([node[0], neighbor[0]], [node[1], neighbor[1]], 'k-', alpha=0.2, linewidth=1, zorder=1)
                visited_edges.add(edge_key)

    # Plot Start and Goal
    plt.scatter([start[0]], [start[1]], c='green', s=150, label='Start', zorder=5)
    plt.scatter([goal[0]], [goal[1]], c='red', marker='*', s=300, label='Goal', zorder=5)
    plt.text(start[0], start[1]+2, 'START', color='green', fontweight='bold')
    plt.text(goal[0], goal[1]+2, 'GOAL', color='red', fontweight='bold')

    # Plot Dijkstra Path (Blue, thick)
    if d_path:
        dx, dy = zip(*d_path)
        plt.plot(dx, dy, 'b-', linewidth=6, alpha=0.5, label=f'Dijkstra ({d_metrics["path_cost"]:.2f}m)', zorder=3)

    # Plot A* Path (Red, dashed, thin)
    if a_path:
        ax, ay = zip(*a_path)
        plt.plot(ax, ay, 'r--', linewidth=2, alpha=1.0, label=f'A* ({a_metrics["path_cost"]:.2f}m)', zorder=4)

    plt.title(f"Path Planning Comparison: Dijkstra vs A*\nStart: {start} -> Goal: {goal}")
    plt.xlabel("East (x)")
    plt.ylabel("North (y)")
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.axis('equal')
    
    # Save output
    output_filename = 'path_visualization.png'
    # Use current working directory or script directory for output
    save_path = Path.cwd() / output_filename
    plt.savefig(str(save_path), dpi=300, bbox_inches='tight')
    print(f"Visualization saved to: {save_path}")

if __name__ == "__main__":
    main()

