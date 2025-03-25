import re
import statistics
import os

def parse_data(filepath):
    with open(filepath, 'r') as f:
        content = f.read()

    # Split the file content into blocks using the delimiter.
    blocks = content.split('--------------------------')
    planning_times = []
    pushing_lengths = []

    for block in blocks:
        block = block.strip()
        if not block:
            continue

        # Extract planning time (e.g., "Planning_Time(ms): 130")
        planning_match = re.search(r'Planning_Time\(ms\):\s*(\d+)', block)
        if planning_match:
            planning_times.append(int(planning_match.group(1)))

        # Extract pushing length (e.g., "Pushing_Length(m): 8.94253")
        pushing_match = re.search(r'Pushing_Length\(m\):\s*([\d\.]+)', block)
        if pushing_match:
            pushing_lengths.append(float(pushing_match.group(1)))
            
    return planning_times, pushing_lengths

def process_file(filepath):
    planning_times, pushing_lengths = parse_data(filepath)
    
    # Success rate: number of sets divided by 100.
    num_sets = len(planning_times)
    success_rate = num_sets / 100.0

    # Compute averages and standard deviations for planning times.
    if planning_times:
        avg_planning_time = statistics.mean(planning_times)
        std_planning_time = statistics.stdev(planning_times) if len(planning_times) > 1 else 0.0
    else:
        avg_planning_time = std_planning_time = 0.0

    # Compute averages and standard deviations for pushing lengths.
    if pushing_lengths:
        avg_pushing_length = statistics.mean(pushing_lengths)
        std_pushing_length = statistics.stdev(pushing_lengths) if len(pushing_lengths) > 1 else 0.0
    else:
        avg_pushing_length = std_pushing_length = 0.0

    # Print the results for this file.
    print(f"Results for file: {os.path.basename(filepath)}")
    print(f"  Success Rate: {success_rate}")
    print(f"  Average Planning Time: {avg_planning_time} ms")
    print(f"  Standard Deviation of Planning Time: {std_planning_time} ms")
    print(f"  Average Pushing Length: {avg_pushing_length}")
    print(f"  Standard Deviation of Pushing Length: {std_pushing_length}")
    print("-" * 40)

def main():
    # Determine the script's directory and construct the path to the log folder.
    script_dir = os.path.dirname(os.path.abspath(__file__))
    log_dir = os.path.join(script_dir, "..", "log")

    # Process each .txt file in the log folder separately.
    for filename in os.listdir(log_dir):
        if filename.endswith('.txt'):
            filepath = os.path.join(log_dir, filename)
            process_file(filepath)

if __name__ == "__main__":
    main()
