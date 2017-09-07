#ifndef LISTNODE_H
#define LISTNODE_H

#include <DRRT/libraries.h>
#include <DRRT/obstacle.h>

class ObstacleListNode
{
    bool is_empty_;
    bool in_list_;

public:
    std::shared_ptr<ObstacleListNode> child_;
    std::shared_ptr<ObstacleListNode> parent_;
    std::shared_ptr<Obstacle> data_;

    ObstacleListNode(std::shared_ptr<Obstacle> &o) : data_(o) { SetEmpty(false); }
    ObstacleListNode() : is_empty_(true) {}

    void GetData(std::shared_ptr<Obstacle> &obstacle) { obstacle = data_; }
    void SetEmpty(bool empty) { is_empty_ = empty; }
    bool IsEmpty() { return is_empty_; }
    void SetInList(bool inlist) { in_list_ = inlist; }
    bool InList() { return in_list_; }
};

#endif // LISTNODE_H
