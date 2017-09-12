#ifndef LISTNODE_H
#define LISTNODE_H

#include <DRRT/libraries.h>
#include <DRRT/obstacle.h>
#include <DRRT/list.h>

class ObstacleListNode
{
    bool is_empty_;
    bool in_list_;

    std::shared_ptr<ObstacleListNode> child_;
    std::shared_ptr<ObstacleListNode> parent_;
    Obstacle_ptr data_;

public:
    ObstacleListNode(Obstacle_ptr &o) : data_(o) { SetEmpty(false); }
    ObstacleListNode() : is_empty_(true) {}

    void GetData(Obstacle_ptr &obstacle) { obstacle = data_; }
    std::shared_ptr<ObstacleListNode> GetChild() { return child_; }
    std::shared_ptr<ObstacleListNode> GetParent() { return parent_; }
    void SetChild(std::shared_ptr<ObstacleListNode> new_child) { child_ = new_child; }
    void SetParent(std::shared_ptr<ObstacleListNode> new_parent) { parent_ = new_parent; }
    void SetEmpty(bool empty) { is_empty_ = empty; }
    bool IsEmpty() { return is_empty_; }
    void SetInList(bool inlist) { in_list_ = inlist; }
    bool InList() { return in_list_; }
};

typedef std::shared_ptr<ObstacleListNode> ObstacleListNode_ptr;
typedef List<ObstacleListNode> ObstacleList;
typedef std::shared_ptr<ObstacleList> ObstacleList_ptr;

#endif // LISTNODE_H
