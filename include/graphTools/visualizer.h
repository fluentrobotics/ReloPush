#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

#include <graphTools/graph_info.h>
#include <reloPush/movableObject.h>

#include <unordered_set>

class position2D
{
    public:
    position2D()
    {}
    position2D(float x_in, float y_in)
    {
        x = x_in;
        y = y_in;
    }
    float x;
    float y;
};

double node_size = 0.075;
double graph_height = 1.5;

//todo: get it as a param
double vehicle_lengh = 0.1;

//void publish_marker(std::vector<position2D> blocks_list, ros::Publisher* pub_ptr, double block_size=node_size, std::string shape="Sphere",  bool use_color=true) {
void publish_pose_nodes(std::unordered_map<std::string,VertexStatePair> nameMatcher, ros::Publisher* pub_ptr, 
                        double block_size=node_size, std::string shape="Sphere",  bool use_color=true) {


    std::vector<std::vector<float>>colors = {
                        {0.615, 0.851, 0.823},
                        {0.075, 0.435, 0.388},
                        {0.878, 0.792, 0.235},
                        {0.953, 0.258, 0.075},
                        {0.968, 0.698, 0.678},
                        {0.035, 0.016, 0.275},
                        {0.471, 0.435, 0.322},
                        {0.996, 0.725, 0.373},
                        {0.969, 0.090, 0.208},
                        {0.761, 0.035, 0.353},
                        {0.196, 0.192, 0.357},
                        {0.267, 0.294, 0.431},
                        {0.439, 0.545, 0.459},
                        {0.604, 0.722, 0.478},
                        {0.972, 0.976, 0.568},
                        {0.6039215686274509,0.6784313725490196,0.7490196078431373}, 
                        {0.42745098039215684,0.596078431372549,0.7294117647058823},
                        {0.8274509803921568,0.7254901960784313,0.6235294117647059},
                        {0.7568627450980392,0.4666666666666667,0.403921568627451},
                        {0.7568627450980392,0.4666666666666667,0.403921568627451}};


    visualization_msgs::MarkerArray marker_array;
    //for (size_t n = 0; n < blocks_list.size(); ++n) {
    size_t color_n = 0;
    for (auto const& [key, val] : nameMatcher)
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "graph";  // Set the frame in which the marker will be displayed
        marker.header.stamp = ros::Time::now();
        if (shape == "cube") {
            marker.type = visualization_msgs::Marker::CUBE;
        } else {
            marker.type = visualization_msgs::Marker::SPHERE;
        }
        marker.action = visualization_msgs::Marker::ADD;

        //todo: use a common value
        //float direct_offset = 0.25f;
        // node push direction
        float node_th = val.state->yaw;

        marker.pose.position.x = val.state->x;  // Set the position of the marker
        marker.pose.position.x += vehicle_lengh * cos(node_th);
        marker.pose.position.y = val.state->y;
        marker.pose.position.y += vehicle_lengh * sin(node_th);
        marker.pose.position.z = graph_height;
        marker.pose.orientation.x = 0;  // Set the orientation of the marker
        marker.pose.orientation.y = 0;
        marker.pose.orientation.z = 0;
        marker.pose.orientation.w = 1;
        marker.scale.x = block_size;  // Set the scale of the marker (size)
        marker.scale.y = block_size;
        marker.scale.z = block_size;
        marker.color.a = 0.5;  // Set the alpha (transparency) of the marker
        if (use_color) {
            marker.color.r = colors[3][0];  // Set the color of the marker (red)
            marker.color.g = colors[3][1];
            marker.color.b = colors[3][2];
        } else {
            marker.color.r = 0.93;  // grey
            marker.color.g = 0.93;
            marker.color.b = 0.93;
        }
        marker.id = color_n;
        marker_array.markers.push_back(marker);

        color_n++;
    }

    // Publish the marker
    pub_ptr->publish(marker_array);
}

/*
void draw_lines(std::vector<geometry_msgs::Point>& plist) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "graph";  // Set the frame in which the points are defined
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.025;  // Line width

    // Set the points
    marker.points = plist;

    marker.color.a = 1.0;  // Alpha
    marker.color.r = 0.13725490196;  // Red
    marker.color.g = 0.94117647;  // Green
    marker.color.b = 0.78039216;  // Blue

    edge_marker_pub_ptr->publish(marker);
}
*/

double vector_length(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2) {
    return std::sqrt(pow(p2.x - p1.x,2) + pow(p2.y - p1.y,2));
}

// Function to normalize a vector
geometry_msgs::Point normalize(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2) {
    double length = vector_length(p1,p2);
    geometry_msgs::Point normalized;
    if (length != 0) {
        normalized.x = (p2.x-p1.x) / length;
        normalized.y = (p2.y-p1.y) / length;
    }
    return normalized;
}

geometry_msgs::Point scale(const geometry_msgs::Point& p, double factor) {
    geometry_msgs::Point scaled;
    scaled.x = p.x * factor;
    scaled.y = p.y * factor;
    return scaled;
}

void draw_arrows(std::vector<geometry_msgs::Point>& slist, std::vector<geometry_msgs::Point>& elist, ros::Publisher* pub_ptr) {
    // Create a MarkerArray message
    visualization_msgs::MarkerArray marker_array;

    // Example arrow parameters
    int arrow_count = slist.size();
    double arrow_shaft = 0.01;
    double arrow_scale = 0.04;
    double arrow_height = 0.067;

    // Loop to create multiple arrows
    for (int i = 0; i < arrow_count; ++i) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "graph";
        marker.type = visualization_msgs::Marker::ARROW;
        marker.action = visualization_msgs::Marker::ADD;
        marker.id = i;
        marker.scale.x = arrow_shaft;
        marker.scale.y = arrow_scale;
        marker.scale.z = arrow_height;
        marker.color.a = 1.0;  // Alpha
        //marker.color.r = 0.13725490196;  // Red
        //marker.color.g = 0.94117647;  // Green
        //marker.color.b = 0.78039216;  // Blue
        //change color by angle
        auto ang = atan2((elist[i].y-slist[i].y),(elist[i].x-slist[i].x));
        auto ang_ratio = ang/(2*M_PI);
        //marker.color.r = 0.161;  // Red
        marker.color.r = ang_ratio;  // Red
        marker.color.g = 0.455;  // Green
        marker.color.b = 0.451;  // Blue
        marker.pose.orientation.w = 1.0;

        //make the arrow shorter
        auto normalized = normalize(slist[i],elist[i]);
        auto offset = scale(normalized,node_size/2);
        geometry_msgs::Point ps,pe;
        ps.x = slist[i].x + offset.x;
        ps.y = slist[i].y + offset.y;
        ps.z = graph_height;
        pe.x = elist[i].x - offset.x;
        pe.y = elist[i].y - offset.y;
        pe.z = graph_height;

        // Set start and end positions for the arrow
        marker.points.push_back(ps);
        marker.points.push_back(pe);

        // Add the marker to the MarkerArray
        marker_array.markers.push_back(marker);
    }

    pub_ptr->publish(marker_array);
}

void visualize_graph(Graph& g, NameMatcher& nameMatcher, ros::Publisher* node_pub_ptr, ros::Publisher* edge_pub_ptr, double block_size=node_size)
{
    // Vector to store vertex information
    std::vector<std::pair<Vertex, std::string>> vertex_info;
    // Vector to store edge information
    std::vector<std::tuple<Vertex, Vertex, int>> edge_info;

    // Store vertex information
    graph_traits<Graph>::vertex_iterator vi, vend;
    for (boost::tie(vi, vend) = vertices(g); vi != vend; ++vi) {
        vertex_info.push_back(std::make_pair(*vi, get(vertex_name, g, *vi)));
    }

    // Store edge information
    graph_traits<Graph>::edge_iterator ei, eend;
    for (boost::tie(ei, eend) = edges(g); ei != eend; ++ei) {
        edge_info.push_back(std::make_tuple(source(*ei, g), target(*ei, g), get(edge_weight, g, *ei)));
    }
    publish_pose_nodes(nameMatcher.vsMap, node_pub_ptr);

    auto edge_list = graphTools::getEdges(g);
    //iterate all edges and connected vertices
    std::vector<geometry_msgs::Point> startPointsList(0), endPointsList(0);
    for (size_t n=0; n<edge_list->size(); n++)
    {
        auto vertPair = graphTools::getVertexPair(edge_list, n, g);
        auto vSource = graphTools::getVertexName(vertPair.getSource(), g);

        //source vertex orientation
        float source_th = nameMatcher.vsMap[vSource].state->yaw;

        auto source_x = nameMatcher.vsMap[vSource].state->x;
        source_x -= vehicle_lengh * cos(source_th);

        auto source_y = nameMatcher.vsMap[vSource].state->y;
        source_y -= vehicle_lengh * sin(source_th);

        auto vSink = graphTools::getVertexName(vertPair.getSink(), g);
        //sink vertex orientation
        float sink_th = nameMatcher.vsMap[vSink].state->yaw;

        auto sink_x = nameMatcher.vsMap[vSink].state->x;
        sink_x -= vehicle_lengh * cos(sink_th);
        auto sink_y = nameMatcher.vsMap[vSink].state->y;
        sink_y -= vehicle_lengh * sin(sink_th);
        //auto eWeight = vertPair.getEdgeWeight();
        //std::cout << "Edge Weight: " << eWeight << "\n" << std::endl;

        geometry_msgs::Point p1,p2;
        p1.x = source_x;
        p1.y = source_y;
        p1.z = 1.0;
        p2.x = sink_x;
        p2.y = sink_y;
        p2.z = 1.0;

        startPointsList.push_back(p1);
        endPointsList.push_back(p2);
    }
    draw_arrows(startPointsList,endPointsList,edge_pub_ptr);
}