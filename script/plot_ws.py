import matplotlib.pyplot as plt
import matplotlib.patches as patches
import math

def read_exported_file(filename):
    """
    Reads the exported .txt file and parses boundary, objects, goals, and paths.

    Args:
        filename (str): Path to the exported .txt file.

    Returns:
        tuple: (xMax, yMax, objects, goals, paths)
            - xMax (float): Maximum x boundary.
            - yMax (float): Maximum y boundary.
            - objects (list of tuples): Each tuple is (x, y, nominalOrientation).
            - goals (list of tuples): Each tuple is (x, y, nominalOrientation).
            - paths (list of lists): Each inner list contains waypoints as (x, y, yaw).
    """
    with open(filename, 'r') as f:
        lines = f.readlines()
    
    if len(lines) < 3:
        raise ValueError("File must have at least 3 lines (boundary, objects, goals).")
    
    # Line 1: Boundary
    boundary = lines[0].strip().split(',')
    if len(boundary) != 2:
        raise ValueError("Boundary line must have exactly two values: xMax,yMax.")
    xMax, yMax = float(boundary[0]), float(boundary[1])
    
    # Line 2: Objects
    objects = []
    objects_entries = lines[1].strip().split(';')
    for entry in objects_entries:
        parts = entry.split(',')
        if len(parts) != 3:
            print(f"Skipping invalid object entry: {entry}")
            continue
        x, y, nominalOrientation = map(float, parts)
        objects.append((x, y, nominalOrientation))
    
    # Line 3: Goals
    goals = []
    goals_entries = lines[2].strip().split(';')
    for entry in goals_entries:
        parts = entry.split(',')
        if len(parts) != 3:
            print(f"Skipping invalid goal entry: {entry}")
            continue
        x, y, nominalOrientation = map(float, parts)
        goals.append((x, y, nominalOrientation))
    
    # Lines 4 and onward: Paths
    paths = []
    for line in lines[3:]:
        waypoints = []
        entries = line.strip().split(';')
        for entry in entries:
            parts = entry.split(',')
            if len(parts) != 3:
                print(f"Skipping invalid waypoint entry: {entry}")
                continue
            x, y, yaw = map(float, parts)
            waypoints.append((x, y, yaw))
        if waypoints:
            paths.append(waypoints)
    
    return (xMax, yMax, objects, goals, paths)

def plot_path_planning(xMax, yMax, objects, goals, paths):
    """
    Plots the boundary, objects, goals, trajectories, and orientation arrows using matplotlib.

    Args:
        xMax (float): Maximum x boundary.
        yMax (float): Maximum y boundary.
        objects (list of tuples): Each tuple is (x, y, nominalOrientation).
        goals (list of tuples): Each tuple is (x, y, nominalOrientation).
        paths (list of lists): Each inner list contains waypoints as (x, y, yaw).
    """
    fig, ax = plt.subplots(figsize=(10,10))
    
    # Set boundary
    ax.set_xlim(0, xMax)
    ax.set_ylim(0, yMax)
    ax.set_title('Path Planning Visualization')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    
    # Ensure equal scaling
    ax.set_aspect('equal', adjustable='box')
    
    # Plot Objects as Rotated Squares
    side_length = 0.3
    for obj in objects:
        x, y, nominalOrientation = obj
        # Convert orientation from radians to degrees for matplotlib
        angle_deg = math.degrees(nominalOrientation)
        # Calculate lower-left corner for Rectangle
        lower_left_x = x - side_length / 2
        lower_left_y = y - side_length / 2
        rect = patches.Rectangle((lower_left_x, lower_left_y), side_length, side_length,
                                 linewidth=1, edgecolor='r', facecolor='none', angle=angle_deg)
        ax.add_patch(rect)
        # Annotate object
        ax.text(x, y, f"Obj({x},{y})", color='r', fontsize=8, ha='center', va='center')
    
    # Plot Goals as Rotated Squares with Different Color
    for goal in goals:
        x, y, nominalOrientation = goal
        # Convert orientation from radians to degrees for matplotlib
        angle_deg = math.degrees(nominalOrientation)
        # Calculate lower-left corner for Rectangle
        lower_left_x = x - side_length / 2
        lower_left_y = y - side_length / 2
        rect = patches.Rectangle((lower_left_x, lower_left_y), side_length, side_length,
                                 linewidth=1, edgecolor='g', facecolor='none', angle=angle_deg)
        ax.add_patch(rect)
        # Annotate goal
        ax.text(x, y, f"Goal({x},{y})", color='g', fontsize=8, ha='center', va='center')
    
    # Plot Trajectories and Orientation Arrows
    arrow_length = 0.02  # Length of the orientation arrow
    for path in paths:
        x_coords = [wp[0] for wp in path]
        y_coords = [wp[1] for wp in path]
        yaws = [wp[2] for wp in path]
        ax.plot(x_coords, y_coords, 'b-', linewidth=1, label='Trajectory' if path == paths[0] else "")
        ax.plot(x_coords, y_coords, 'bo', markersize=2)  # Waypoints
        
        # Add orientation arrows at each waypoint
        for (x, y, yaw) in path:
            # Calculate arrow components
            dx = arrow_length * math.cos(yaw)
            dy = arrow_length * math.sin(yaw)
            ax.arrow(x, y, dx, dy, head_width=0.03, head_length=0.045, fc='k', ec='k', length_includes_head=True)
    
    # Create Legend
    import matplotlib.lines as mlines
    object_patch = mlines.Line2D([], [], color='r', marker='s', linestyle='None',
                                 markersize=10, label='Object')
    goal_patch = mlines.Line2D([], [], color='g', marker='s', linestyle='None',
                               markersize=10, label='Goal')
    trajectory_line = mlines.Line2D([], [], color='b', linestyle='-', label='Trajectory')
    plt.legend(handles=[object_patch, goal_patch, trajectory_line])
    
    plt.grid(True)
    plt.show()

if __name__ == "__main__":
    # Path to the exported .txt file
    filename = "path_planning_data.txt"
    
    try:
        # Read the exported data
        xMax, yMax, objects, goals, paths = read_exported_file(filename)
        
        # Plot the data
        plot_path_planning(xMax, yMax, objects, goals, paths)
    except Exception as e:
        print(f"An error occurred: {e}")

if __name__ == "__main__":
    # Path to the exported .txt file
    filename = "test.txt"
    
    try:
        # Read the exported data
        xMax, yMax, objects, goals, paths = read_exported_file(filename)
        
        # Plot the data
        plot_path_planning(xMax, yMax, objects, goals, paths)
    except Exception as e:
        print(f"An error occurred: {e}")
