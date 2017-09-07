#include <DRRT/list.h>

int List::GetLength()
{
    return length_;
}

void List::Push(std::shared_ptr<ListNode> &new_node)
{
    new_node->child_ = front_;
    front_ = new_node;
    length_ += 1;
}

void List::Top(std::shared_ptr<ListNode> &top_node)
{
    if(front_ != front_->child_) {
        top_node = front_;
        top_node->SetEmpty(false);
    } else {
        // List is empty, leave top_node as is
        top_node->SetEmpty(true);
    }
}

void List::Pop(std::shared_ptr<ListNode> &top_node)
{
    if(front_ != front_->child_) {
        top_node = front_;
        top_node->SetEmpty(false);
        front_ = front_->child_;
        length_ -= 1;
    } else {
        // List is empty, leave top_node as is
        top_node->SetEmpty(true);
    }
}

void List::Empty()
{
    std::shared_ptr<ListNode> temp = std::make_shared<ListNode>();
    List::Pop(temp);
    while(temp != temp->child_) List::Pop(temp);
}


/*#include <DRRT/obstacle_listnode.h>

int main()
{
    std::shared_ptr<Obstacle> obs = std::make_shared<Obstacle>(1);
    std::shared_ptr<ListNode> obs_node = std::make_shared<ObstacleListNode>(obs);
    List list = List(obs_node);

    std::shared_ptr<ListNode> obs_node_0 = std::make_shared<ObstacleListNode>(obs);

    list.Push(obs_node_0);

    std::cout << "length: " << list.GetLength() << std::endl;

    std::shared_ptr<ListNode> obs_node_1 = std::make_shared<ObstacleListNode>(obs);

    list.Push(obs_node_1);

    std::cout << "length: " << list.GetLength() << std::endl;

    std::shared_ptr<ListNode> obs_node_2 = std::make_shared<ListNode>();

    std::cout << "obs_node_2: " << obs_node_2->IsEmpty() << std::endl;

    list.Pop(obs_node_2);

    std::cout << "obs_node_2: " << obs_node_2->IsEmpty() << std::endl;


    return 0;
}*/
