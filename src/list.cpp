/* list.cpp
 * Corin Sandford
 * Fall 2016
 */

#include <DRRT/list.h>

int main()
{
    List L = List();

    KDTreeNode* a = new KDTreeNode(1);
    KDTreeNode* b = new KDTreeNode(2);
    KDTreeNode* c = new KDTreeNode(3);

    L.listPush(a,1);
    L.listPush(b,2);
    L.listPush(c,3);

    L.listPrint();
    std::cout << "list printed" << std::endl;

    L.listEmpty();
    std::cout << "list emptied" << std::endl;

    L.listPrint();
    std::cout << "list printed" << std::endl;

    L.listPush(a,1);
    L.listPush(b,2);

    L.listPrint();
    std::cout << "list printed" << std::endl;

    std::cout << "-- copy:" << std::endl;
    List L2 = L.listCopy( L.front );
    L2.listPrint();

    std::cout << "-- original:" << std::endl;
    L.listPrint();

    std::cout << "L.length =?= L2.length" << " : " << L.length << " =?= " << L2.length << std::endl;

    L.listPush(c,3);
    L2.listPush(c,3);
    L.listPrint();
    std::cout << "list printed" << std::endl;
    L2.listPrint();
    std::cout << "list2 printed" << std::endl;
    std::cout << "L.length =?= L2.length" << " : " << L.length << " =?= " << L2.length << std::endl;
}
