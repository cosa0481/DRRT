#ifndef OBSTACLE_H
#define OBSTACLE_H


class Obstacle
{
    int kind_;
public:
    Obstacle(int kind) : kind_(kind) {}

    int GetKind() { return kind_; }
    void SetKind(int k) { kind_ = k; }
};

#endif // OBSTACLE_H
