#include <batchInstanceParcer.hpp>

float convertEulerRange_to_2pi(float angle) {
    if (angle < 0) {
        return angle + 2 * M_PI;
    } else {
        return angle;
    }
}

float convertEulerRange_to_pi(float yaw) {
    if (yaw > M_PI) {
        yaw -= 2 * M_PI;
    }
    return yaw;
}

std::vector<std::string> split(std::string& s, std::string delimiter)
{
    size_t pos_start = 0, pos_end, delim_len = delimiter.length();
    std::string token;
    std::vector<std::string> res;

    while ((pos_end = s.find(delimiter, pos_start)) != std::string::npos)
    {
        token = s.substr(pos_start, pos_end - pos_start);
        pos_start = pos_end + delim_len;
        res.push_back(token);
    }

    res.push_back(s.substr(pos_start));
    return res;
}

/// Read a file and create a std::vector by lines
std::vector<std::string> read_file(std::string f_path)
{
    std::string cmake_data_dir = std::string(CMAKE_SOURCE_DIR) + "/input/";

    using namespace std;
    ifstream file_to_read;
    file_to_read.open(cmake_data_dir + f_path);
    if (!file_to_read) {
        throw std::runtime_error("Unable to open file: " + f_path);
    }

    string line;

    std::vector<std::string> out_result;

    if (file_to_read.is_open())
    {
        while (getline(file_to_read, line))
        {
            //cout << line << '\n';
            out_result.push_back(line);
        }
        file_to_read.close();
    }

    else
    {
        cout << "Unable to open file" << endl;
    }

    return out_result;
}

bool parse_instance_from_file( std::string file_path, size_t data_ind,
                              std::unordered_map<std::string, ObjectInfo>& objects,
                              std::unordered_map<std::string, GoalInfo>& goals,
                              std::vector<ReloPush::State>& robots,
                              std::unordered_map<std::string, ObjectGoalPair> &objGoalPairs)
{
    std::string type_delim = "!"; // separate mo and robot
    std::string header_delim = ":";
    std::string object_delim = ";";
    std::string elem_delilm = ",";

    double obs_rad = 0.075; // todo: parse from params

    //read all lines first then choose one
    auto file_lines = read_file(file_path);
    if (data_ind >= file_lines.size()) {
        std::ostringstream oss;
        oss << "Data index " << data_ind << " is out of range. File has "
            << file_lines.size() << " lines.";
        throw std::out_of_range(oss.str());
    }

    // format
    // mo:mo1.name,mo1.x,mo1.y,mo1.th,mo1.num_side;mo2.name ... !robot:r1.x,r1.y,r1.th !goal: !assign:

    auto type_sp = split(file_lines[data_ind],"!");


    std::string mo_str, robot_str, goal_str, assign_str;
    //std::unordered_map<std::string,std::string> d_table;

    for(auto& it : type_sp)
    {
        auto temp_sp = split(it,":");
        if (temp_sp.size() < 2)
            throw std::runtime_error("Malformed header: " + it);

        if(temp_sp[0] == "mo")
            mo_str = temp_sp[1];

        else if(temp_sp[0] == "robot")
            robot_str = temp_sp[1];

        else if(temp_sp[0] == "goal")
            goal_str = temp_sp[1];

        else if(temp_sp[0] == "assign")
            assign_str = temp_sp[1];
    }

    // parse movable objects
    auto mo_sp = split(mo_str,object_delim);
    //mo_list.resize(mo_sp.size());
    objects.clear();
    for(size_t i=0; i<mo_sp.size(); i++)
    {
        //name, x, y, th, n_sides
        auto mo_elem_sp = split(mo_sp[i],elem_delilm);
        if (mo_elem_sp.size() < 5) {
            throw std::runtime_error("Malformed movable object entry: " + mo_sp[i]);
        }

        ObjectInfo parsedObj;
        try {
            parsedObj = ObjectInfo(
                mo_elem_sp[0],
                std::stod(mo_elem_sp[1]),
                std::stod(mo_elem_sp[2]),
                std::stod(mo_elem_sp[3]),
                std::stoi(mo_elem_sp[4]),
                obs_rad
                );
        }
        catch (const std::exception& e) {
            throw std::runtime_error("Error parsing movable object at index " + std::to_string(i) +
                                     ": " + e.what());
        }

        objects.insert({mo_elem_sp[0],parsedObj});

        //mo_list[i] = ObjectInfo(mo_elem_sp[0],std::stod(mo_elem_sp[1]),std::stod(mo_elem_sp[2]),
        //                           std::stod(mo_elem_sp[3]),std::stoi(mo_elem_sp[4]),obs_rad);
    }

    // parse robots
    auto robot_sp = split(robot_str,object_delim);
    robots.resize(robot_sp.size());
    for(size_t i=0; i<robot_sp.size(); i++)
    {
        // x, y, th
        auto robot_elem_sp = split(robot_sp[i],elem_delilm);
        if (robot_elem_sp.size() < 3) {
            throw std::runtime_error("Malformed robot entry: " + robot_sp[i]);
        }
        try {
            robots[i] = ReloPush::State(
                std::stof(robot_elem_sp[0]),
                std::stof(robot_elem_sp[1]),
                std::stof(robot_elem_sp[2])
                );
            // Convert yaw into 0 ~ 2pi range
            robots[i].yaw = convertEulerRange_to_2pi(robots[i].yaw);
        }
        catch (const std::exception& e) {
            throw std::runtime_error("Error parsing robot at index " + std::to_string(i) +
                                     ": " + e.what());
        }

        //robots[i] = ReloPush::State(std::stof(robot_elem_sp[0]),std::stof(robot_elem_sp[1]),std::stof(robot_elem_sp[2]));
        // in 0 ~ 2pi range
        //robots[i].yaw = convertEulerRange_to_2pi(robots[i].yaw);
        // negate yaw for hybrid astar use
    }

    // parse delivery poses
    auto goal_sp = split(goal_str,object_delim);
    //delivery_list.resize(goal_sp.size());
    goals.clear();
    for(size_t i=0; i<goal_sp.size(); i++)
    {
        //name, x, y, th, n_sides
        auto goal_elem_sp = split(goal_sp[i],elem_delilm);
        if (goal_elem_sp.size() < 5) {
            throw std::runtime_error("Malformed goal entry: " + goal_sp[i]);
        }

        GoalInfo parsedGoal;
        try {
            parsedGoal = GoalInfo(
                goal_elem_sp[0],
                std::stof(goal_elem_sp[1]),
                std::stof(goal_elem_sp[2]),
                std::stof(goal_elem_sp[3]),
                std::stoi(goal_elem_sp[4]),
                obs_rad
                );
        }
        catch (const std::exception& e) {
            throw std::runtime_error("Error parsing goal at index " + std::to_string(i) +
                                     ": " + e.what());
        }

        goals.insert({goal_elem_sp[0],parsedGoal});
        //delivery_list[i] = GoalInfo(goal_elem_sp[0],std::stof(goal_elem_sp[1]),std::stof(goal_elem_sp[2]),
        //                                 std::stof(goal_elem_sp[3]),std::stoi(goal_elem_sp[4]),obs_rad);
    }

    // parse assignment
    auto assign_sp = split(assign_str,object_delim);
    objGoalPairs.clear();
    for(auto& it : assign_sp)
    {
        auto assign_elem_sp = split(it,elem_delilm);
        if (assign_elem_sp.size() < 2) {
            throw std::runtime_error("Malformed assignment entry: " + it);
        }
        objGoalPairs.insert({assign_elem_sp[0], ObjectGoalPair(assign_elem_sp[0],assign_elem_sp[1])});
    }

    return true;
}

void handle_args(int argc, char **argv, std::string& data_file, int& data_ind, bool& use_opt)
{
    data_file = std::string(argv[1]);
    data_ind = std::atoi(argv[2]);
    if(std::atoi(argv[3])==1)
        use_opt = true;
    else
        use_opt = false;
}
