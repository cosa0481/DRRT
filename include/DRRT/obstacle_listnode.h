#ifndef LISTNODE_H
#define LISTNODE_H

#include <DRRT/listnode.h>
#include <DRRT/obstacle.h>

class ObstacleListNode : public ListNode
{
public:
    std::shared_ptr<Obstacle> data_;

    ObstacleListNode(std::shared_ptr<Obstacle> &o) : data_(o) { SetEmpty(false); }
    ObstacleListNode() : ListNode() {}

    void GetData(std::shared_ptr<Obstacle> obstacle) { obstacle = data_; }
};

#endif // LISTNODE_H
