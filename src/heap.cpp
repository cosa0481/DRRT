#include <DRRT/heap.h>

void Heap::Clean(std::vector<HeapNode_ptr> &heap)
{
    for(int i = 0; i <= index_of_last_; i++) {
        heap_[i]->SetInHeap(false);
        heap_[i]->SetIndex(UNSET_IDX);
    }

    index_of_last_ = 0;
    parent_of_last_ = -1;

    Heap::GetHeap(heap);
}


///////////////// Min Heap /////////////////

void Heap::AddMin(HeapNode_ptr &node)
{
    if(!node->InHeap()) {
        index_of_last_ += 1;
        parent_of_last_ = index_of_last_/2;  /****************/
        heap_.push_back(node);
        heap_[index_of_last_]->SetIndex(index_of_last_);
        heap_[index_of_last_]->SetInHeap(true);
        Heap::BubbleUpMin(index_of_last_);
    } else {
        if(DEBUG) std::cout << "AddMin: Node already in heap." << std::endl;
    }
}

void Heap::RemoveMin(HeapNode_ptr &node)
{
    if(node->InHeap()) {
        int idx = node->GetIndex();
        heap_[idx] = heap_[index_of_last_];
        heap_[idx]->SetIndex(idx);
        index_of_last_ -= 1;
        parent_of_last_ = index_of_last_/2;  /****************/
        Heap::BubbleUpMin(idx);
        Heap::BubbleDownMin(heap_[idx]->GetIndex());
        node->SetInHeap(false);
        node->SetIndex(UNSET_IDX);
    } else {
        if(DEBUG) std::cout << "RemoveMin: Node not in heap." << std::endl;
    }
}

void Heap::UpdateMin(HeapNode_ptr &node)
{
    if(node->InHeap()) {
        Heap::BubbleUpMin(node->GetIndex());
        Heap::BubbleDownMin(node->GetIndex());
    } else {
        if(DEBUG) std::cout << "UpdateMin: Node not in heap." << std::endl;
    }
}

void Heap::TopMin(HeapNode_ptr &node)
{
    if(index_of_last_ >= 1) node = heap_[1];
    else { if(DEBUG) std::cout << "TopMin: index_of_last_ < 1." << std::endl; }
}

void Heap::PopMin(HeapNode_ptr &node)
{
    HeapNode_ptr old_top;
    if(index_of_last_ > 1) {
        old_top = heap_[1];
        heap_[1] = heap_[index_of_last_];
        heap_[1]->SetIndex(1);
    } else if(index_of_last_ == 1) {
        old_top = heap_[1];
    } else { if(DEBUG) std::cout << "PopMin: index_of_last_ < 1." << std::endl; }
    heap_.erase(heap_.begin() + index_of_last_);
    index_of_last_ -= 1;
    parent_of_last_ = index_of_last_/2;  /****************/
    Heap::BubbleDownMin(1);
    old_top->SetInHeap(false);
    old_top->SetIndex(UNSET_IDX);
    node = old_top;
}

void Heap::BubbleUpMin(int idx)
{
    if(idx != 1) {
        int parent = idx/2;  /****************/
        while(idx > 1 && (heap_[parent]->GetValue() > heap_[idx]->GetValue())) {
            HeapNode_ptr temp = heap_[parent];
            heap_[parent] = heap_[idx];
            heap_[idx] = temp;

            heap_[parent]->SetIndex(parent);
            heap_[idx]->SetIndex(idx);

            idx = parent;
            parent = idx/2;  /****************/
        }
    }
}

void Heap::BubbleDownMin(int idx)
{
    int child = -1;
    if(2*idx == index_of_last_) {
        child = 2*idx;
    } else if(2*idx + 1 > index_of_last_) {
        child = child;
    } else if(heap_[2*idx]->GetValue() < heap_[2*idx + 1]->GetValue()) {
        child = 2*idx;
    } else {
        child = 2*idx + 1;
    }

    if(child != -1) {
        while(idx <= parent_of_last_ && (heap_[child]->GetValue() < heap_[idx]->GetValue())) {
            HeapNode_ptr temp = heap_[child];
            heap_[child] = heap_[idx];
            heap_[idx] = temp;

            heap_[child]->SetIndex(child);
            heap_[idx]->SetIndex(idx);

            idx = child;
            if(2*idx == index_of_last_) {
                child = 2*idx;
            } else if(2*idx + 1 > index_of_last_) {
                break;
            } else if(heap_[2*idx]->GetValue() < heap_[2*idx + 1]->GetValue()) {
                child = 2*idx;
            } else {
                child = 2*idx + 1;
            }
        }
    }
}

bool Heap::CheckMin()
{
    if(index_of_last_ < 1) {
        if(DEBUG) std::cout << "The heap is empty." << std::endl;
        return true;
    } else if(heap_[1]->GetIndex() != 1) {
        if(DEBUG) std::cout << "There is a problem with the heap.\n\theap_[1]->GetIndex() = " << heap_[1]->GetIndex() << std::endl;
        return false;
    }

    int i = 2;
    while( i <= index_of_last_) {
        if(heap_[i]->GetValue() < heap_[i/2]->GetValue()) {  /****************/
            if(DEBUG) std::cout << "There is a problem with the heap order." << std::endl;
            return false;
        } else if(heap_[i]->GetIndex() != i) {
            if(DEBUG) std::cout << "There is a problem with the heap.\n\theap_[" << i << "]->GetIndex() = " << heap_[i]->GetIndex() << std::endl;
            return false;
        }
        i += 1;
    }

    if(DEBUG) std::cout << "The heap is OK." << std::endl;
    return true;
}


///////////////// Max Heap /////////////////

void Heap::AddMax(HeapNode_ptr &node)
{
    if(!node->InHeap()) {
        index_of_last_ += 1;
        parent_of_last_ = index_of_last_/2;  /****************/
        heap_.push_back(node);
        heap_[index_of_last_]->SetIndex(index_of_last_);
        heap_[index_of_last_]->SetInHeap(true);
        Heap::BubbleUpMax(index_of_last_);
    } else {
        if(DEBUG) std::cout << "AddMax: Node already in heap." << std::endl;
    }
}

void Heap::RemoveMax(HeapNode_ptr &node)
{
    if(node->InHeap()) {
        int idx = node->GetIndex();
        heap_[idx] = heap_[index_of_last_];
        heap_[idx]->SetIndex(idx);
        index_of_last_ -= 1;
        parent_of_last_ = index_of_last_/2;  /****************/
        Heap::BubbleUpMax(idx);
        Heap::BubbleDownMax(heap_[idx]->GetIndex());
        node->SetInHeap(false);
        node->SetIndex(UNSET_IDX);
    } else {
        if(DEBUG) std::cout << "RemoveMax: Node not in heap." << std::endl;
    }
}

void Heap::UpdateMax(HeapNode_ptr &node)
{
    if(node->InHeap()) {
        Heap::BubbleUpMax(node->GetIndex());
        Heap::BubbleDownMax(node->GetIndex());
    } else {
        if(DEBUG) std::cout << "UpdateMax: Node not in heap." << std::endl;
    }
}

void Heap::TopMax(HeapNode_ptr &node)
{
    if(index_of_last_ >= 1) node = heap_[1];
    else { if(DEBUG) std::cout << "TopMax: index_of_last_ < 1." << std::endl; }
}

void Heap::PopMax(HeapNode_ptr &node)
{
    HeapNode_ptr old_top;
    if(index_of_last_ > 1) {
        old_top = heap_[1];
        heap_[1] = heap_[index_of_last_];
        heap_[1]->SetIndex(1);
    } else if(index_of_last_ == 1) {
        old_top = heap_[1];
    } else { if(DEBUG) std::cout << "PopMax: index_of_last_ < 1." << std::endl; }
    heap_.erase(heap_.begin() + index_of_last_);
    index_of_last_ -= 1;
    parent_of_last_ = index_of_last_/2;  /****************/
    Heap::BubbleDownMax(1);
    old_top->SetInHeap(false);
    old_top->SetIndex(UNSET_IDX);
    node = old_top;
}

void Heap::BubbleUpMax(int idx)
{
    if(idx != 1) {
        int parent = idx/2;  /****************/
        while(idx > 1 && (heap_[parent]->GetValue() < heap_[idx]->GetValue())) {
            HeapNode_ptr temp = heap_[parent];
            heap_[parent] = heap_[idx];
            heap_[idx] = temp;

            heap_[parent]->SetIndex(parent);
            heap_[idx]->SetIndex(idx);

            idx = parent;
            parent = idx/2;  /****************/
        }
    }
}

void Heap::BubbleDownMax(int idx)
{
    int child = -1;
    if(2*idx == index_of_last_) {
        child = 2*idx;
    } else if(2*idx + 1 > index_of_last_) {
        child = child;
    } else if(heap_[2*idx]->GetValue() > heap_[2*idx + 1]->GetValue()) {
        child = 2*idx;
    } else {
        child = 2*idx + 1;
    }

    if(child != -1) {
        while(idx <= parent_of_last_ && (heap_[child]->GetValue() > heap_[idx]->GetValue())) {
            HeapNode_ptr temp = heap_[child];
            heap_[child] = heap_[idx];
            heap_[idx] = temp;

            heap_[child]->SetIndex(child);
            heap_[idx]->SetIndex(idx);

            idx = child;
            if(2*idx == index_of_last_) {
                child = 2*idx;
            } else if(2*idx + 1 > index_of_last_) {
                break;
            } else if(heap_[2*idx]->GetValue() > heap_[2*idx + 1]->GetValue()) {
                child = 2*idx;
            } else {
                child = 2*idx + 1;
            }
        }
    }
}

bool Heap::CheckMax()
{
    if(index_of_last_ < 1) {
        if(DEBUG) std::cout << "The heap is empty." << std::endl;
        return true;
    } else if(heap_[1]->GetIndex() != 1) {
        if(DEBUG) std::cout << "There is a problem with the heap.\n\theap_[1]->GetIndex() = " << heap_[1]->GetIndex() << std::endl;
        return false;
    }

    int i = 2;
    while( i <= index_of_last_) {
        if(heap_[i]->GetValue() > heap_[i/2]->GetValue()) {  /****************/
            if(DEBUG) std::cout << "There is a problem with the heap order." << std::endl;
            return false;
        } else if(heap_[i]->GetIndex() != i) {
            if(DEBUG) std::cout << "There is a problem with the heap.\n\theap_[" << i << "]->GetIndex() = " << heap_[i]->GetIndex() << std::endl;
            return false;
        }
        i += 1;
    }

    if(DEBUG) std::cout << "The heap is OK." << std::endl;
    return true;
}


/*#include <DRRT/kdnode_heapnode.h>

int main()
{
    Heap heap = Heap(0);  // 0 for Min heap
    std::shared_ptr<Kdnode> kd_node1 = std::make_shared<Kdnode>(10);
    std::shared_ptr<Kdnode> kd_node2 = std::make_shared<Kdnode>(-10);
    std::shared_ptr<Kdnode> kd_node3 = std::make_shared<Kdnode>(30);
    std::shared_ptr<Kdnode> kd_node4 = std::make_shared<Kdnode>(40);
    std::shared_ptr<Kdnode> kd_node5 = std::make_shared<Kdnode>(50);

    std::cout << "kd_node1->GetCost(): " << kd_node1->GetCost() << std::endl;

    std::shared_ptr<HeapNode> node1 = std::make_shared<KdHeapNode>(kd_node1);
    std::shared_ptr<HeapNode> node2 = std::make_shared<KdHeapNode>(kd_node2);

    std::cout << "node1->GetValue(): " << node1->GetValue() << std::endl;

    std::cout << "heap is empty: " << heap.IsEmpty() << std::endl;
    heap.Check();

    heap.Add(node1);
    heap.Add(node2);

    std::cout << "heap is empty: " << heap.IsEmpty() << std::endl;
    std::cout << "size: " << heap.GetHeapSize() << std::endl;

    heap.Check();

    std::shared_ptr<HeapNode> popped_node;
    heap.Top(popped_node);
    std::cout << "popped_node in heap: " << popped_node->InHeap() << std::endl;
    heap.Pop(popped_node);
    std::cout << "min node: " << popped_node->GetValue() << std::endl;
    std::cout << "popped_node in heap: " << popped_node->InHeap() << std::endl;
    std::cout << "heap is empty: " << heap.IsEmpty() << std::endl;
    std::cout << "size: " << heap.GetHeapSize() << std::endl;

    heap.Check();

    std::shared_ptr<HeapNode> node3 = std::make_shared<KdHeapNode>(kd_node3);
    heap.Add(node3);

    std::shared_ptr<HeapNode> removed_node = node1;
    std::shared_ptr<HeapNode> node4 = std::make_shared<KdHeapNode>(kd_node4);
    std::shared_ptr<HeapNode> node5 = std::make_shared<KdHeapNode>(kd_node5);

    heap.Add(node1);
    heap.Add(node4);
    heap.Add(node5);

    std::cout << "heap is empty: " << heap.IsEmpty() << std::endl;
    std::cout << "size: " << heap.GetHeapSize() << std::endl;

    heap.Check();

    std::cout << "removed_node in heap: " << removed_node->InHeap() << std::endl;
    heap.Remove(removed_node);
    std::cout << "removed_node in heap: " << removed_node->InHeap() << std::endl;
    std::cout << "heap is empty: " << heap.IsEmpty() << std::endl;
    std::cout << "size: " << heap.GetHeapSize() << std::endl;

    heap.Check();

    //--------------------------------------------------------------------

//    Heap heap = Heap(1);  // 1 for Max heap
//    std::shared_ptr<Kdnode> kd_node1 = std::make_shared<Kdnode>(-10);
//    std::shared_ptr<Kdnode> kd_node2 = std::make_shared<Kdnode>(10);
//    std::shared_ptr<Kdnode> kd_node3 = std::make_shared<Kdnode>(30);
//    std::shared_ptr<Kdnode> kd_node4 = std::make_shared<Kdnode>(40);
//    std::shared_ptr<Kdnode> kd_node5 = std::make_shared<Kdnode>(50);

//    std::cout << "kd_node1->GetCost(): " << kd_node1->GetCost() << std::endl;

//    std::shared_ptr<HeapNode> node1 = std::make_shared<KdHeapNode>(kd_node1);
//    std::shared_ptr<HeapNode> node2 = std::make_shared<KdHeapNode>(kd_node2);

//    std::cout << "node1->GetValue(): " << node1->GetValue() << std::endl;

//    std::cout << "heap is empty: " << heap.IsEmpty() << std::endl;
//    heap.Check();

//    heap.Add(node1);
//    heap.Add(node2);

//    std::cout << "heap is empty: " << heap.IsEmpty() << std::endl;
//    std::cout << "size: " << heap.GetHeapSize() << std::endl;

//    heap.Check();

//    std::shared_ptr<HeapNode> popped_node;
//    heap.Top(popped_node);
//    std::cout << "popped_node in heap: " << popped_node->InHeap() << std::endl;
//    heap.Pop(popped_node);
//    std::cout << "max node: " << popped_node->GetValue() << std::endl;
//    std::cout << "popped_node in heap: " << popped_node->InHeap() << std::endl;
//    std::cout << "heap is empty: " << heap.IsEmpty() << std::endl;
//    std::cout << "size: " << heap.GetHeapSize() << std::endl;

//    heap.Check();

//    std::shared_ptr<HeapNode> node3 = std::make_shared<KdHeapNode>(kd_node3);
//    heap.Add(node3);

//    std::shared_ptr<HeapNode> removed_node = node1;
//    std::shared_ptr<HeapNode> node4 = std::make_shared<KdHeapNode>(kd_node4);
//    std::shared_ptr<HeapNode> node5 = std::make_shared<KdHeapNode>(kd_node5);

//    heap.Add(node1);
//    heap.Add(node4);
//    heap.Add(node5);

//    std::cout << "heap is empty: " << heap.IsEmpty() << std::endl;
//    std::cout << "size: " << heap.GetHeapSize() << std::endl;

//    heap.Check();

//    std::cout << "removed_node in heap: " << removed_node->InHeap() << std::endl;
//    heap.Remove(removed_node);
//    std::cout << "removed_node in heap: " << removed_node->InHeap() << std::endl;
//    std::cout << "heap is empty: " << heap.IsEmpty() << std::endl;
//    std::cout << "size: " << heap.GetHeapSize() << std::endl;

//    heap.Check();

    return 0;
}*/
