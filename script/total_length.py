import math
import ast
import sys

def shift_pose(x, y, yaw, do_shift, distance=0.49094):
    if do_shift:
        return (x - distance * math.cos(yaw), y - distance * math.sin(yaw))
    else:
        return (x, y)

def main():
    if len(sys.argv) < 2:
        print("Usage: python script.py actions.txt")
        return

    filename = sys.argv[1]
    path_types = []
    positions = []
    yaws = []

    with open(filename, 'r') as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('actions:') or not line.startswith('['):
                continue
            arr = ast.literal_eval(line.rstrip(','))
            path_types.append(arr[0])
            positions.append((arr[3], arr[4]))
            yaws.append(arr[5])

    total_length = 0.0
    transfer_length = 0.0
    transit_length = 0.0

    for i in range(1, len(positions)):
        type_prev, type_curr = path_types[i-1], path_types[i]
        yaw_prev, yaw_curr = yaws[i-1], yaws[i]
        x_prev, y_prev = positions[i-1]
        x_curr, y_curr = positions[i]

        # Shift each endpoint only if its own type is transfer (1)
        x0_adj, y0_adj = shift_pose(x_prev, y_prev, yaw_prev, type_prev == 1)
        x1_adj, y1_adj = shift_pose(x_curr, y_curr, yaw_curr, type_curr == 1)

        d = math.hypot(x1_adj - x0_adj, y1_adj - y0_adj)
        total_length += d

        # Transfer: only strictly transfer segments
        if type_prev == 1 and type_curr == 1:
            transfer_length += d
        else:  # Transit includes both strict and transition segments
            transit_length += d

    print(f"Total path length (shifting each transfer point): {total_length:.3f} m")
    print(f"Total transfer length (strictly transfer, shifted): {transfer_length:.3f} m")
    print(f"Total transit length (including transitions): {transit_length:.3f} m")
    print(f"Check: total = transfer + transit? {abs(total_length - (transfer_length + transit_length)) < 1e-6}")

if __name__ == "__main__":
    main()
