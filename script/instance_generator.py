import random
import math
import matplotlib.pyplot as plt

def distance(p1, p2):
    return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

def is_valid_position(position, positions, min_distance):
    for pos in positions:
        if distance(position, pos) < min_distance:
            return False
    return True

def generate_positions(num_positions, boundary, radius, min_distance, existing_positions=[]):
    positions = []
    attempts = 0
    max_attempts = 10000  # To prevent infinite loops

    while len(positions) < num_positions and attempts < max_attempts:
        attempts += 1
        x = random.uniform(boundary['x_min'] + min_distance, boundary['x_max'] - min_distance)
        y = random.uniform(boundary['y_min'] + min_distance, boundary['y_max'] - min_distance)
        if is_valid_position((x, y), positions + existing_positions, min_distance):
            positions.append((x, y))

    if attempts == max_attempts:
        raise Exception("Unable to generate valid positions within the given constraints")

    return positions

def generate_data(num_objects, boundary_size, radius, min_distance):
    boundary = {
        'x_min': 0,
        'x_max': boundary_size[0],
        'y_min': 0,
        'y_max': boundary_size[1]
    }
    
    # Adjust minimum distance to consider both object radius and specified minimum distance
    adjusted_min_distance = max(radius * 1.5, min_distance * 1.5) + radius
    
    # Generate object positions
    objects = generate_positions(num_objects, boundary, radius, adjusted_min_distance)
    
    # Generate goal positions, considering both existing object positions and goal positions
    goals = generate_positions(num_objects, boundary, radius, adjusted_min_distance, existing_positions=objects)
    
    return objects, goals

def plot_positions(objects, goals, boundary_size, radius):
    fig, ax = plt.subplots()
    ax.set_xlim(0, boundary_size[0])
    ax.set_ylim(0, boundary_size[1])
    ax.set_aspect('equal')

    # Plot objects
    for i, (x, y) in enumerate(objects):
        circle = plt.Circle((x, y), radius, color='blue', fill=True)
        ax.add_artist(circle)
        ax.text(x, y, str(i+1), color='white', ha='center', va='center')
    
    # Plot goals
    for i, (x, y) in enumerate(goals):
        circle = plt.Circle((x, y), radius, color='red', fill=False, linestyle='dashed')
        ax.add_artist(circle)
        ax.text(x, y, str(i+1), color='red', ha='center', va='center')

    plt.xlabel('X-axis')
    plt.ylabel('Y-axis')
    plt.title('Objects and Goals')
    plt.grid(True)
    plt.show()

# Parameters
num_objects = 6
boundary_size = (4, 5)
radius = 0.2
min_distance = 0.4  # Given separate minimum distance

objects, goals = generate_data(num_objects, boundary_size, radius, min_distance)

print("Objects:", objects)
print("Goals:", goals)

plot_positions(objects, goals, boundary_size, radius)
