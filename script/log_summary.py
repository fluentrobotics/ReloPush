import numpy as np
import os

folder = "../log/raw"

# Get the directory of the current script
current_directory = os.path.dirname(__file__)


def parse_file(file_name):
    data_blocks = []
    current_block = {}

    with open(file_name, 'r') as f:
        for line in f:
            line = line.strip()
            #if line == "":  # Skip empty lines
            #    continue
            if line.startswith("#"):  # Indicates end of one block
                if(len(current_block)>0):
                    data_blocks.append(current_block)
                    current_block = {}
                continue

            key_value = line.split(":")
            if len(key_value) == 2:
                key, value = key_value[0].strip(), key_value[1].strip()

                # Convert numerical values to float
                try:
                    if key in ["Time Task", "Time MP", "Time Total", "Num Pre-relo", "Num Temp-relo", 
                               "Num Total relo", "Length Pushing", "Length Total", "Execution Time", "Lost"]:
                        current_block[key] = float(value)
                    elif key == "Is Success":
                        current_block[key] = value
                except ValueError:
                    current_block[key] = value

            

    return data_blocks

def calculate_stats(data_blocks):
    # Initialize lists for calculations
    time_task = []
    time_mp = []
    time_total = []
    num_pre = []
    num_temp = []
    num_total = []
    length_pushing = []
    length_total = []
    exec_time = []
    block_lost = []

    # Filter data for 'Is Success' == 'T'
    success_blocks = [block for block in data_blocks if block.get("Is Success") == 'T']

    for block in success_blocks:
        if "Time Task" in block:
            time_task.append(block["Time Task"])
        if "Time MP" in block:
            time_mp.append(block["Time MP"])
        if "Time Total" in block:
            time_total.append(block["Time Total"])
        if "Num Pre-relo" in block:
            num_pre.append(block["Num Pre-relo"])
        if "Num Temp-relo" in block:
            num_temp.append(block["Num Temp-relo"])
        if "Num Total relo" in block:
            num_total.append(block["Num Total relo"])
        if "Length Pushing" in block:
            length_pushing.append(block["Length Pushing"])
        if "Length Total" in block:
            length_total.append(block["Length Total"])
        if "Execution Time" in block:
            exec_time.append(block["Execution Time"])
        if "Lost" in block:
            block_lost.append(block["Lost"])

    total_entries = len(success_blocks)

    # Calculate averages and standard deviations
    def calc_avg_std(values):
        return np.mean(values), np.std(values)

    avg_time_task, std_time_task = calc_avg_std(time_task)
    avg_time_mp, std_time_mp = calc_avg_std(time_mp)
    avg_time_total, std_time_total = calc_avg_std(time_total)
    avg_num_pre, std_num_pre = calc_avg_std(num_pre)
    avg_num_temp, std_num_temp = calc_avg_std(num_temp)
    avg_num_total, std_num_total = calc_avg_std(num_total)
    avg_length_pushing, std_length_pushing = calc_avg_std(length_pushing)
    avg_length_total, std_length_total = calc_avg_std(length_total)
    avg_exec_time, std_exec_time = calc_avg_std(exec_time)
    avg_block_lost, std_block_lost = calc_avg_std(block_lost)

    # Output results
    print(f"Time Task: Avg = {avg_time_task}, Std = {std_time_task}")
    print(f"Time MP: Avg = {avg_time_mp}, Std = {std_time_mp}")
    print(f"Time Total: Avg = {avg_time_total}, Std = {std_time_total}")
    print(f"Num Pre-relo: Avg = {avg_num_pre}, Std = {std_num_pre}")
    print(f"Num Temp-relo: Avg = {avg_num_temp}, Std = {std_num_temp}")
    print(f"Num Total relo: Avg = {avg_num_total}, Std = {std_num_total}")
    print(f"Length Pushing: Avg = {avg_length_pushing}, Std = {std_length_pushing}")
    print(f"Length Total: Avg = {avg_length_total}, Std = {std_length_total}")
    print(f"Total Success Entries: {total_entries}")
    print(f"Execution Time: Avg = {avg_exec_time}, Std = {std_exec_time}")
    print(f"Lost: Avg = {avg_block_lost}, Std = {std_block_lost}")


# Construct the full file path
folder_path = os.path.join(current_directory, folder)
print(os.listdir(folder_path))
# Example usage
for file in os.listdir(folder_path):
    file_name = folder_path + "/"+ file
    print("\n========== " + file + " ==========")
    data_blocks = parse_file(file_name)
    calculate_stats(data_blocks)



