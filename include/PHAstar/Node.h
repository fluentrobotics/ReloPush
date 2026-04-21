#ifndef NODE_H
#define NODE_H

struct Node {
    double x, y, yaw, t, cost, steer;
    Node* parent;
    int direction;
    Node(double x_ = 0, double y_ = 0, double yaw_ = 0, double t_ = 0, double cost_ = 0, double steer_ = 0, Node* p = nullptr, int dir = 1)
        : x(x_), y(y_), yaw(yaw_), t(t_), cost(cost_), steer(steer_), parent(p), direction(dir) {}
};

#endif // NODE_H
