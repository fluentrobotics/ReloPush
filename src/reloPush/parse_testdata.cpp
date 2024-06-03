#include <reloPush/parse_testdata.h>

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

std::string removeExtension(const std::string& filename) {
    size_t lastDotPos = filename.find_last_of(".");
    if (lastDotPos == std::string::npos) {
        // No dot found, return the original filename
        return filename;
    } else {
        // Return the substring from the beginning to the position of the last dot
        return filename.substr(0, lastDotPos);
    }
}

/// Read a file and create a std::vector by lines
std::vector<std::string> read_file(std::string f_path)
{
	using namespace std;
	ifstream file_to_read;
	file_to_read.open(f_path);

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


std::unordered_map<std::string,std::string> parse_movableObjects_robots_from_file(std::vector<movableObject>& mo_list, std::vector<State>& robots,
                                            std::vector<movableObject>& delivery_list, 
                                            size_t data_ind, std::string file_path)
{
    std::string type_delim = "!"; // separate mo and robot
    std::string header_delim = ":";
    std::string object_delim = ";";
    std::string elem_delilm = ",";

    //read all lines first then choose one
    auto file_lines = read_file(file_path);

    // format
    // mo:mo1.name,mo1.x,mo1.y,mo1.th,mo1.num_side;mo2.name ... !robot:r1.x,r1.y,r1.th !goal: !assign:

    auto type_sp = split(file_lines[data_ind],"!");

    std::string mo_str, robot_str, goal_str, assign_str;
    std::unordered_map<std::string,std::string> d_table;

    for(auto& it : type_sp)
    {
        auto temp_sp = split(it,":");
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
    mo_list.resize(mo_sp.size());
    for(size_t i=0; i<mo_sp.size(); i++)
    {
        //name, x, y, th, n_sides
        auto mo_elem_sp = split(mo_sp[i],elem_delilm);
        mo_list[i] = movableObject(std::stof(mo_elem_sp[1]),std::stof(mo_elem_sp[2]),
            std::stof(mo_elem_sp[3]),mo_elem_sp[0],std::stoi(mo_elem_sp[4]));
    }

    // parse robots
    auto robot_sp = split(robot_str,object_delim);
    robots.resize(robot_sp.size());
    for(size_t i=0; i<robot_sp.size(); i++)
    {
        // x, y, th
        auto robot_elem_sp = split(robot_sp[i],elem_delilm);
        robots[i] = State(std::stof(robot_elem_sp[0]),std::stof(robot_elem_sp[1]),std::stof(robot_elem_sp[2]));
        // in 0 ~ 2pi range
        robots[i].yaw = jeeho::convertEulerRange_to_2pi(robots[i].yaw);
        // negate yaw for hybrid astar use
        //robots[i].yaw *= -1;
    }

    // parse delivery poses
    auto goal_sp = split(goal_str,object_delim);
    delivery_list.resize(goal_sp.size());
    for(size_t i=0; i<goal_sp.size(); i++)
    {
        //name, x, y, th, n_sides
        auto goal_elem_sp = split(goal_sp[i],elem_delilm);
        delivery_list[i] = movableObject(std::stof(goal_elem_sp[1]),std::stof(goal_elem_sp[2]),
            std::stof(goal_elem_sp[3]),goal_elem_sp[0],std::stoi(goal_elem_sp[4]));
    }

    // parse assignment
    auto assign_sp = split(assign_str,object_delim);
    d_table.clear();
    for(auto& it : assign_sp)
    {
        auto assign_elem_sp = split(it,elem_delilm);
        d_table.insert({assign_elem_sp[0],assign_elem_sp[1]});
    }

    return d_table;
}

