#include <DRRT/kdtree.h>


void KdTree::Print(Kdnode_ptr node, int indent, char type)
{
    if(indent) std::cout << std::string(indent - 1, ' ') << type;
    std::cout << node->GetPosition()(0) << ","
              << node->GetPosition()(1) << ": "
              << node->GetCost();

    if(node->ParentExist()) {
        Edge_ptr parent_edge;
        node->GetRrtParentEdge(parent_edge);
        std::cout << " : " << parent_edge->GetEnd()->GetPosition()(0)
                  << "," << parent_edge->GetEnd()->GetPosition()(1);
        std::cout << " (" << parent_edge->GetDist() << ")";
    }

    if(!node->LChildExist() && !node->RChildExist()) {
        std::cout << " | leaf" << std::endl;
    } else {
        std::cout << std::endl;
        if(node->LChildExist()) PrintTree(node->lchild_, indent + 4, '<');
        if(node->RChildExist()) PrintTree(node->rchild_, indent + 4, '>');
    }
}

void KdTree::GetNodeAt(Kdnode_ptr &node, Eigen::VectorXd pos)
{
    Kdnode_ptr parent = root;
    node = parent;
    while(true) {
        if(pos(parent->GetKdSplit()) < parent->GetPosition()(parent->GetKdSplit())) {
            // Go left
            if(parent->GetPosition() == pos) {
                node = parent;
                break;
            }
            if(!parent->LChildExist()) std::cout << "no left child" << std::endl;
            parent = parent->lchild_;
            continue;
        } else {
            // Go right
            if(parent->GetPosition() == pos) {
                node = parent;
                break;
            }
            if(!parent->RChildExist()) std::cout << "no right child" << std::endl;
            parent = parent->rchild_;
            continue;
        }
    }
}

bool KdTree::KdInsert(Kdnode_ptr &node)
{
    if(node->InTree()) return false;
    node->SetInTree(true);

    if(size_ == 0) {
        root_ = node;
        root_->SetKdSplit(0);
        size_ = 1;
        return true
    }

    Kdnode_ptr parent = root_;
    while(true) {
        if(node->GetPosition()(parent->GetKdSplit())
                < parent->GetPosition()(parent->GetKdSplit())) {
            // Go left
            if(!parent->LChildExist()) {
                parent->lchild_ = node;
                parent->SetLChildExist(true);
                break;
            }
            parent = parent->lchild_;
            continue;
        } else {
            // Go right
            if(!parent->RChildExist()) {
                parent->rchild_ = node;
                parent->SetRChildExist(true);
                break;
            }
            parent = parent->rchild_;
            continue;
        }

        node->parent_ = parent;
        node->SetParentExist(true);
        if(parent->GetKdSplit() == NUM_DIM - 1) node->SetKdSplit(0);
        else node->SetKdSplit(parent->GetKdSplit() + 1);

        size_ += 1;
        return true;
    }
}
