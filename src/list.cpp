#include <DRRT/list.h>

template <class T>
int List<T>::GetLength()
{
    return length_;
}

template <class T>
void List<T>::Push(std::shared_ptr<T> &new_node)
{
    if(!new_node->InList()) {
        new_node->parent_ = front_->parent_;
        new_node->child_ = front_;

        if(length_ == 0) back_ = new_node;
        else front_->parent_ = new_node;

        front_ = new_node;
        length_ += 1;

        new_node->SetInList(true);
    }
}

template <class T>
void List<T>::Top(std::shared_ptr<T> &top_node)
{
    if(front_ != front_->child_) {
        top_node = front_;
        top_node->SetEmpty(false);
    } else {
        // List is empty, leave top_node as is
        top_node->SetEmpty(true);
    }
}

template <class T>
void List<T>::Pop(std::shared_ptr<T> &top_node)
{
    if(front_ != front_->child_) {
        std::shared_ptr<T> old_top = front_;
        if(length_ > 1) {
            front_->child_->parent_ = front_->parent_;
            front_ = front_->child_;
        } else if(length_ == 1) {
            back_ = bound_;
            front_ = bound_;
        }
        length_ -= 1;
        old_top->child_ = old_top;
        old_top->parent_ = old_top;
        top_node = old_top;
        top_node->SetInList(false);
        top_node->SetEmpty(false);
    } else {
        // List is empty, leave top_node as is
        top_node->SetEmpty(true);
    }
}

template <class T>
void List<T>::Remove(std::shared_ptr<T> &node)
{
    if(length_ != 0 && !node->IsEmpty() && node->InList()) {
        if(front_ == node) front_ = node->child_;
        if(back_ == node) back_ = node->parent_;

        std::shared_ptr<T> next_node = node->child_;
        std::shared_ptr<T> prev_node = node->parent_;

        if(length_ > 1 && prev_node != prev_node->child_) prev_node->child_ = next_node;
        if(length_ > 1 && next_node != next_node->child_) next_node->parent_ = prev_node;

        length_ -= 1;

        if(length_ == 0) {
            back_ = bound_;
            front_ = bound_;
        }

        node->child_ = node;
        node->parent_ = node;
        node->SetInList(false);
    } else { if(DEBUG) std::cout << "Node not in List." << std::endl; }
}

template <class T>
void List<T>::Empty()
{
    std::shared_ptr<T> temp = std::make_shared<T>();
    List::Pop(temp);
    while(temp != temp->child_) List::Pop(temp);
}

//template <class T>
//void List<T>::Print()
//{
//    if(length_ == 0) std::cout << "Empty List." << std::endl;
//    else {
//        std::shared_ptr<T> ptr = front_;
//        std::shared_ptr<Obstacle> obs;
//        int i = 0;
//        while(ptr != ptr->child_ && i < 10) {
//            ptr->GetData(obs);
//            std::cout << "ListNode[" << i << "]: "
//                      << obs->GetKind() << std::endl;
//            i += 1;
//            ptr = ptr->child_;
//        }
//    }
//}


/*#include <DRRT/obstacle_listnode.h>

using namespace std;
typedef std::shared_ptr<ObstacleListNode> ObstacleListNode_ptr;

int main()
{
    shared_ptr<Obstacle> obs0 = make_shared<Obstacle>(0);
    shared_ptr<Obstacle> obs1 = make_shared<Obstacle>(1);
    shared_ptr<Obstacle> obs2 = make_shared<Obstacle>(2);
    shared_ptr<Obstacle> obs3 = make_shared<Obstacle>(3);

    ObstacleListNode_ptr obs_node_0 = make_shared<ObstacleListNode>(obs0);
    ObstacleListNode_ptr obs_node_1 = make_shared<ObstacleListNode>(obs1);
    ObstacleListNode_ptr obs_node_2 = make_shared<ObstacleListNode>(obs2);
    ObstacleListNode_ptr obs_node_3 = make_shared<ObstacleListNode>(obs3);

    List<ObstacleListNode> list = List<ObstacleListNode>();

    cout << "Print:" << endl;
    list.Print();

    cout << "Push #1, #2, #3" << endl;
    list.Push(obs_node_1);
    list.Push(obs_node_2);
    list.Push(obs_node_3);

    cout << "Print:" << endl;
    list.Print();

    cout << "length: " << list.GetLength() << endl;

    ObstacleListNode_ptr popped_node = make_shared<ObstacleListNode>();
    cout << "popped_node empty: " << popped_node->IsEmpty() << endl;
    cout << "Pop" << endl;
    list.Pop(popped_node);
    cout << "popped_node empty: " << popped_node->IsEmpty() << endl;

    cout << "Print:" << endl;
    list.Print();
    cout << "length: " << list.GetLength() << endl;

    cout << "Remove #1" << endl;
    list.Remove(obs_node_1);
    cout << "Print:" << endl;
    list.Print();
    cout << "Push #3" << endl;
    list.Push(obs_node_3);
    cout << "Print:" << endl;
    list.Print();
    cout << "Remove #1" << endl;
    list.Remove(obs_node_1);

    cout << "Print:" << endl;
    list.Print();
    cout << "length: " << list.GetLength() << endl;

    cout << "Remove #2" << endl;
    list.Remove(obs_node_2);

    cout << "Print:" << endl;
    list.Print();
    cout << "length: " << list.GetLength() << endl;

    cout << "Remove #3" << endl;
    list.Remove(obs_node_3);

    cout << "Print:" << endl;
    list.Print();
    cout << "length: " << list.GetLength() << endl;

    return 0;
}*/
