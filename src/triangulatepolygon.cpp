#include <DRRT/triangulatepolygon.h>
#include <DRRT/sampling.h>

using namespace std;

// construct.c
vector<shared_ptr<node>> qs(QSIZE);         // query structure
vector<shared_ptr<trapezoid>> tr(TRSIZE);   // trapezoid structure
vector<shared_ptr<segment>> seg(SEGSIZE);   // segment table
int q_idx;                                  // query structure index
int tr_idx;                                 // trapezoid structure index

// misc.c
vector<int> permute(SEGSIZE);               // permutation vector
int choose_idx;                             // permutation vector index

// monotone.c
vector<shared_ptr<monochain>> mchain(TRSIZE);
vector<shared_ptr<vertexchain>> vert(SEGSIZE);
vector<int> mon(SEGSIZE);
vector<bool> visited(TRSIZE);
int chain_idx, op_idx, mon_idx;

// From tri.c
// This functions returns true or false depending upon whether
// the vertex is inside the polygon or not. The polygon must
// already have been triangulated before this routine is called.
// This routine will always detect all the points belonging to
// the set (polygon-area - polygon-boundary). The return value
// for points on the boundary is not consistent!
bool IsPointInsidePolygon(Eigen::Vector2d vertex)
{
    shared_ptr<point> v;
    int trnum, rseg;
    shared_ptr<trapezoid> t;

    v->x = vertex[0];
    v->y = vertex[1];

    trnum = LocateEndpoint(v,v,1);
    t = tr[trnum];

    if(t->state == ST_INVALID) return false;

    if((t->lseg <= 0) || (t->rseg <= 0)) return false;

    rseg = t->rseg;
    return GreaterThanEqualTo_(*seg[rseg]->v1, *seg[rseg]->v0);
}

// construct.c
// Return a new node to be added to the query tree
int NewNode()
{
    if(q_idx < QSIZE) {
        qs.resize(q_idx+1);
        qs[q_idx] = make_shared<node>();
        qs[q_idx]->yval = make_shared<point>();
        return q_idx++;
    }
    else {
        cout << "NewNode:\tQuery table overflow" << endl;
        return -1;
    }
}

// Return a free trapezoid
int NewTrap()
{
    if(tr_idx < TRSIZE) {
        tr.resize(tr_idx+1);
        tr[tr_idx] = make_shared<trapezoid>();
        tr[tr_idx]->hi = make_shared<point>();
        tr[tr_idx]->lo = make_shared<point>();
        tr[tr_idx]->lseg = -1;
        tr[tr_idx]->rseg = -1;
        tr[tr_idx]->state = ST_VALID;
        return tr_idx++;
    } else {
        cout << "NewTrap:\tTrapezoid table overflow" << endl;
        return -1;
    }
}

// Return the maximum of the two points into the yal structure
int Max_(shared_ptr<point> yval, shared_ptr<point> v0, shared_ptr<point> v1)
{
    if(v0->y > v1->y + EPSILON) yval = v0;
    else if(FP_EQUAL(v0->y,v1->y)) {
        if(v0->x > v1->x + EPSILON) yval = v0;
        else yval = v1;
    } else yval = v1;

    return 0;
}

// Return the minimum of the two points into the yval structure
int Min_(shared_ptr<point> yval, shared_ptr<point> v0, shared_ptr<point> v1)
{
    if(v0->y < v1->y - EPSILON) yval = v0;
    else if(FP_EQUAL(v0->y, v1->y)) {
        if(v0->x < v1->x) yval = v0;
        else yval = v1;
    } else yval = v1;

    return 0;
}


bool GreaterThan_(point v0, point v1)
{
    if(v0.y > v1.y + EPSILON) return true;
    else if(v0.y < v1.y - EPSILON) return false;
    else return(v0.x > v1.x);
}

bool EqualTo_(point v0, point v1) {
    return FP_EQUAL(v0.y,v1.y) && FP_EQUAL(v0.x,v1.x);
}

bool GreaterThanEqualTo_(point v0, point v1)
{
    if(v0.y > v1.y + EPSILON) return true;
    else if(v0.y < v1.y - EPSILON) return false;
    else return (v0.x) >= (v1.x);
}

bool LessThan_(point v0, point v1)
{
    if(v0.y < v1.y - EPSILON) return true;
    else if(v0.y > v1.y + EPSILON) return false;
    else return v0.x < v1.x;
}

// Initialize the query structure and the trapezoid table
// when the first segment is added to start the trapezoidation
// The query tree starts with 4 trapezoids, 1 S-node, and 2 Y-nodes
int InitQueryStructure(int seg_num)
{
    int i1, i2, i3, i4, i5, i6, i7, root;
    int t1, t2, t3, t4;
    shared_ptr<segment> s = seg[seg_num];

    q_idx = 1;
    tr_idx = 1;
    i1 = NewNode();
    qs[i1]->node_type = T_Y;
    Max_(qs[i1]->yval, s->v0, s->v1); // root
    root = i1;

    i2 = NewNode();
    qs[i1]->right = i2;
    qs[i2]->node_type = T_SINK;
    qs[i2]->parent = i1;

    i3 = NewNode();
    qs[i1]->left = i3;
    qs[i3]->node_type = T_Y;
    Min_(qs[i3]->yval, s->v0, s->v1); // root
    qs[i3]->parent = i1;

    i4 = NewNode();
    qs[i3]->left = i4;
    qs[i4]->node_type = T_SINK;
    qs[i4]->parent = i3;

    i5 = NewNode();
    qs[i3]->right = i5;
    qs[i5]->node_type = T_X;
    qs[i5]->seg_number = seg_num;
    qs[i5]->parent = i3;

    i6 = NewNode();
    qs[i5]->left = i6;
    qs[i6]->node_type = T_SINK;
    qs[i6]->parent = i5;

    i7 = NewNode();
    qs[i5]->right = i7;
    qs[i7]->node_type = T_SINK;
    qs[i7]->parent = i5;

    t1 = NewTrap();     // middle left
    t2 = NewTrap();     // middle right
    t3 = NewTrap();     // bottom
    t4 = NewTrap();     // top

    tr[t1]->hi = tr[t2]->hi = tr[t4]->lo = qs[i1]->yval;
    tr[t1]->lo = tr[t2]->lo = tr[t3]->hi = qs[i3]->yval;

    tr[t4]->hi->y = (double)INFINITY;
    tr[t4]->hi->x = (double)INFINITY;

    tr[t3]->lo->y = (double) -1 * INFINITY;
    tr[t3]->lo->x = (double) -1 * INFINITY;

    tr[t1]->rseg = tr[t2]->lseg = seg_num;
    tr[t1]->u0 = tr[t2]->u0 = t4;
    tr[t1]->d0 = tr[t2]->d0 = t3;

    tr[t4]->d0 = tr[t3]->u0 = t1;
    tr[t4]->d1 = tr[t3]->u1 = t2;

    tr[t1]->sink = i6;
    tr[t2]->sink = i7;
    tr[t3]->sink = i4;
    tr[t4]->sink = i2;

    tr[t1]->state = tr[t2]->state = ST_VALID;
    tr[t3]->state = tr[t4]->state = ST_VALID;

    qs[i2]->trap_num = t4;
    qs[i4]->trap_num = t3;
    qs[i6]->trap_num = t1;
    qs[i7]->trap_num = t2;

    s->is_inserted = true;

    return root;
}

// Return true if the vertex v is to the left of the line segment
// number seg_num. Takes care of degenerate cases when both
// vertices have the same y--coordinate, etc.
bool IsLeftOf(int seg_num, shared_ptr<point> v)
{
    shared_ptr<segment> s = seg[seg_num];
    double area;

    if(GreaterThan_(*s->v1, *s->v0)) {    // seg going upwards
        if(FP_EQUAL(s->v1->y, v->y)) {
            if(v->x < s->v1->x) {
                area = 1.0;
            } else {
                area = -1.0;
            }
        } else if(FP_EQUAL(s->v0->y, v->y)) {
            if(v->x < s->v0->x) {
                area = 1.0;
            } else {
                area = -1.0;
            }
        } else {
            area = CROSS(*s->v0, *s->v1, *v);
        }
    } else {    // v0 > v1
        if(FP_EQUAL(s->v1->y, v->y)) {
            if(v->x < s->v1->x) {
                area = 1.0;
            } else {
                area = -1.0;
            }
        } else if(FP_EQUAL(s->v0->y, v->y)) {
            if(v->x < s->v0->x) {
                area = 1.0;
            } else {
                area = -1.0;
            }
        } else {
            area = CROSS(*s->v1, *s->v0, *v);
        }
    }

    if(area > 0.0) return true;
    else return false;
}

// Returns true if the corresponding endpoint of the given segment
// is already inserted into the segment tree. Use the simple test
// of whether the segment which shares this endpoint is already inserted
bool Inserted(int seg_num, int which_pt)
{
    if(which_pt == FIRSTPT) return seg[seg[seg_num]->prev]->is_inserted;
    else return seg[seg[seg_num]->next]->is_inserted;
}


// This query routine which determines which trapezoid
// the point  v lies in. The return value is the trapezoid number
int LocateEndpoint(shared_ptr<point> v, shared_ptr<point> vo, int r)
{
    shared_ptr<node> rptr = qs[r];

    switch(rptr->node_type)
    {
        case T_SINK:
            return rptr->trap_num;
        case T_Y:
            if(GreaterThan_(*v,*rptr->yval)) // above
                return LocateEndpoint(v,vo,rptr->right);
            else if(EqualTo_(*v,*rptr->yval)) { // point is already inserted
                if(GreaterThan_(*vo,*rptr->yval)) // above
                    return LocateEndpoint(v,vo,rptr->right);
                else // below
                    return LocateEndpoint(v,vo,rptr->left);
            } else return LocateEndpoint(v,vo,rptr->left); // below
        case T_X:
            if(EqualTo_(*v,*seg[rptr->seg_number]->v0)
                    || EqualTo_(*v, *seg[rptr->seg_number]->v1)) {
                if(FP_EQUAL(v->y,vo->y)) { // horizontal segment
                    if(vo->x < v->x)  // left
                        return LocateEndpoint(v,vo,rptr->left);
                    else // right
                        return LocateEndpoint(v,vo,rptr->right);
                } else if(IsLeftOf(rptr->seg_number,vo)) // left
                    return LocateEndpoint(v,vo,rptr->left);
                else // right
                    return LocateEndpoint(v,vo,rptr->right);
            } else if(IsLeftOf(rptr->seg_number, v)) // left
                return LocateEndpoint(v,vo,rptr->left);
            else // right
                return LocateEndpoint(v,vo,rptr->right);
        default:
            cout << "Haggu!!!!!" << endl;
            break;
    }
    return -1;
}

// Thread in the segment into the existing trapezoidation. The
// limiting trapezoids are given by tfirst and tlast (which
// are the trapezoids containing the two endpoints of the segment.
// Merges all possible trapezoids which flank this segment and have
// recently been divided because of its insertion
int MergeTrapezoids(int seg_num, int tfirst, int tlast, int side)
{
    cout << "MergeTrapezoids" << endl;
    int t, tnext;
    bool cond;
    int ptnext;

    // First merge polys on the LHS
    t = tfirst;
    while((t > 0) && GreaterThanEqualTo_(*tr[t]->lo, *tr[tlast]->lo)) {
        cout << "t: " << t << endl;
        cout << "tlast: " << tlast << endl;
        cout << "tr[" << t << "]->lo: " << tr[t]->lo->x << "," << tr[t]->lo->y << endl;
        cout << "tr[" << tlast << "]->lo: " << tr[tlast]->lo->x <<","<< tr[tlast]->lo->y << endl;
        if(side == S_LEFT)
            cond = ((((tnext = tr[t]->d0) > 0)
                     && (tr[tnext]->rseg == seg_num))
                    || (((tnext = tr[t]->d1) > 0)
                        && (tr[tnext]->rseg == seg_num)));
        else
            cond = ((((tnext = tr[t]->d0) > 0)
                     && (tr[tnext]->lseg == seg_num))
                    || (((tnext = tr[t]->d1) > 0)
                        && (tr[tnext]->lseg == seg_num)));
        cout << "tnext: " << tnext << endl;
        if(cond) {
            cout << "cond: true" << endl;
            if((tr[t]->lseg == tr[tnext]->lseg)
                    && (tr[t]->rseg == tr[tnext]->rseg)) {
                cout << "t->lseg = tnext->lseg && t->rseg == tnext->rseg" << endl;
                // Good neighbors so merge them
                // Use the upper node as the new node i.e. t
                ptnext = qs[tr[tnext]->sink]->parent;

                if(qs[ptnext]->left == tr[tnext]->sink)
                    qs[ptnext]->left = tr[t]->sink;
                else
                    qs[ptnext]->right = tr[t]->sink; // redirect parent

                // Change the upper neighbors of the lower trapezoids

                if((tr[t]->d0 = tr[tnext]->d0) > 0) {
                    if(tr[tr[t]->d0]->u0 == tnext)
                        tr[tr[t]->d0]->u0 = t;
                    else if(tr[tr[t]->d0]->u1 == tnext)
                        tr[tr[t]->d0]->u1 = t;
                }
                if((tr[t]->d1 = tr[tnext]->d1) > 0) {
                    if(tr[tr[t]->d1]->u0 == tnext)
                        tr[tr[t]->d1]->u0 = t;
                    else if(tr[tr[t]->d1]->u1 == tnext)
                        tr[tr[t]->d1]->u1 = t;
                }
                tr[t]->lo = tr[tnext]->lo;
                cout << "tr[" << tnext << "]->lo: " << tr[tnext]->lo->x <<","<<tr[tnext]->lo->y << endl;
                tr[tnext]->state = ST_INVALID; // invalidate the lower trapezium
            } else {
                t = tnext; // not good neighbors
            }
        } else {
            cout << "cond: false" << endl;
            t = tnext; // do not satisfy the outer if
        }
        cout << endl;
    }
    return 0;
}

// Add in the new segment into the trapezoidation and update qs and tr
// structures. First locate the two endpoints of the segment in the qs
// structure. Then start from the topmost trapezoid and go down to
// the lower trapezoid dividing all the trapezoids in betwen
int AddSegment(int seg_num)
{
    cout << "AddSegment" << endl;
    shared_ptr<segment> s;
    shared_ptr<segment> so = seg[seg_num];
    int tu, tl, sk, tfirst, tlast;
    int tfirstr, tlastr, tfirstl, tlastl;
    int i1, i2, t, tn;
    shared_ptr<point> tpt;
    int tritop = 0, tribot = 0;
    bool is_swapped = false;
    int tmptriseg;

    s = seg[seg_num];
    cout << "seg[" << seg_num << "]: " << s << endl;
    if(GreaterThan_(*s->v1, *s->v0)) {
        // Get higher vertex in v0
        int tmp;
        tpt = s->v0;
        s->v0 = s->v1;
        s->v1 = tpt;
        tmp = s->root0;
        s->root0 = s->root1;
        s->root1 = tmp;
        is_swapped = true;
    }

    if((is_swapped) ? !Inserted(seg_num,LASTPT) : !Inserted(seg_num,FIRSTPT)) {
        // Insert v0 in the tree
        int tmp_d;

        tu = LocateEndpoint(s->v0,s->v1,s->root0);
        tl = NewTrap(); // tl is the new lower trapezoid
        tr[tl]->state = ST_VALID;
        tr[tl] = tr[tu];

        tr[tu]->lo->y = tr[tl]->hi->y = s->v0->y;
        tr[tu]->lo->x = tr[tl]->hi->x = s->v0->x;

        tr[tu]->d0 = tl;
        tr[tu]->d1 = 0;
        tr[tl]->u0 = tu;
        tr[tl]->u1 = 0;

        if(((tmp_d = tr[tl]->d0) > 0) && (tr[tmp_d]->u0 == tu))
            tr[tmp_d]->u0 = tl;
        if(((tmp_d = tr[tl]->d0) > 0) && (tr[tmp_d]->u1 == tu))
            tr[tmp_d]->u1 = tl;
        if(((tmp_d = tr[tl]->d1) > 0) && (tr[tmp_d]->u0 == tu))
            tr[tmp_d]->u0 = tl;
        if(((tmp_d = tr[tl]->d1) > 0) && (tr[tmp_d]->u1 == tu))
            tr[tmp_d]->u1 = tl;

        // Update the query structure and obtain the sinks for the two traps
        i1 = NewNode(); // upper trapezoid sink
        i2 = NewNode(); // lower trapezoid sink
        sk = tr[tu]->sink;

        qs[sk]->node_type = T_Y;
        qs[sk]->yval = s->v0;
        qs[sk]->seg_number = seg_num; // not really required (maybe later)
        qs[sk]->left = i2;
        qs[sk]->right = i1;

        qs[i1]->node_type = T_SINK;
        qs[i1]->trap_num = tu;
        qs[i1]->parent = sk;

        qs[i2]->node_type = T_SINK;
        qs[i2]->trap_num = tl;
        qs[i2]->parent = sk;

        tr[tu]->sink = i1;
        tr[tl]->sink = i2;
        tfirst = tl;
    } else {
        // v0 already present
        // Get topmost intersecting trapezoid
        tfirst = LocateEndpoint(s->v0,s->v1,s->root0);
        tritop = 1;
    }

    if((is_swapped) ? !Inserted(seg_num,FIRSTPT) : !Inserted(seg_num,LASTPT)) {
        // Insert v1 in the tree
        int tmp_d;

        tu = LocateEndpoint(s->v1,s->v0,s->root1);
        tl = NewTrap(); // tl is the new lower trapezoid
        tr[tl]->state = ST_VALID;
        tr[tl] = tr[tu];

        tr[tu]->lo->y = tr[tl]->hi->y = s->v1->y;
        tr[tu]->lo->x = tr[tl]->hi->x = s->v1->x;

        tr[tu]->d0 = tl;
        tr[tu]->d1 = 0;
        tr[tl]->u0 = tu;
        tr[tl]->u1 = 0;

        if(((tmp_d = tr[tl]->d0) > 0) && (tr[tmp_d]->u0 == tu))
            tr[tmp_d]->u0 = tl;
        if(((tmp_d = tr[tl]->d0) > 0) && (tr[tmp_d]->u1 == tu))
            tr[tmp_d]->u1 = tl;
        if(((tmp_d = tr[tl]->d1) > 0) && (tr[tmp_d]->u0 == tu))
            tr[tmp_d]->u0 = tl;
        if(((tmp_d = tr[tl]->d1) > 0) && (tr[tmp_d]->u1 == tu))
            tr[tmp_d]->u1 = tl;

        // Update the query structure and obtain the sinks for the two traps
        i1 = NewNode(); // upper trapezoid sink
        i2 = NewNode(); // lower trapezoid sink
        sk = tr[tu]->sink;

        qs[sk]->node_type = T_Y;
        qs[sk]->yval = s->v1;
        qs[sk]->seg_number = seg_num; // not really required (maybe later)
        qs[sk]->left = i2;
        qs[sk]->right = i1;

        qs[i1]->node_type = T_SINK;
        qs[i1]->trap_num = tu;
        qs[i1]->parent = sk;

        qs[i2]->node_type = T_SINK;
        qs[i2]->trap_num = tl;
        qs[i2]->parent = sk;

        tr[tu]->sink = i1;
        tr[tl]->sink = i2;
        tlast = tl;
    } else {
        // v1 already present
        // Get bottommost intersecting trapezoid
        tlast = LocateEndpoint(s->v1,s->v0,s->root1);
        tribot = 1;
    }

    // Thread the segment into the query tree creating a new X-node
    // First, split all the trapezoids which are intersected by s
    // into two

    t = tfirst; // topmost trapezoid
    while((t > 0) && GreaterThanEqualTo_(*tr[t]->lo, *tr[tlast]->lo)) {


        // Traverse top to bottom
        int t_sav, tn_sav;
        sk = tr[t]->sink;
        i1 = NewNode(); // left trapezoid sink
        i2 = NewNode(); // right trapezoid sink

        qs[sk]->node_type = T_X;
        qs[sk]->seg_number = seg_num;
        qs[sk]->left = i1;
        qs[sk]->right = i2;

        qs[i1]->node_type = T_SINK; // left trapezoid (use existing one)
        qs[i1]->trap_num = t;
        qs[i1]->parent = sk;

        qs[i2]->node_type = T_SINK; // right trapezoid (allocate new)
        qs[i2]->trap_num = tn = NewTrap();
        tr[tn]->state = ST_VALID;
        qs[i2]->parent = sk;

        if(t == tfirst) tfirstr = tn;
        if(EqualTo_(*tr[t]->lo, *tr[tlast]->lo)) tlastr = tn;

        tr[tn] = tr[t];
        tr[t]->sink = i2;
        tr[tn]->sink = i2;
        t_sav = t;
        tn_sav = tn;

        if(tr[t]->d0 <= 0 && tr[t]->d1 <= 0) {
            cout << "AddSegment: error" << endl;
            break;
        } else if((tr[t]->d0 > 0) && (tr[t]->d1 <= 0)) {
            // Only one trapezoid below. Partition t into two and make
            // the two resulting trapezoids t and tn as the upper neighbors
            // of the sole lower trapezoid
            if((tr[t]->u0 > 0) && (tr[t]->u1 > 0)) {
                // Continuation of a chain from above
                if(tr[t]->usave > 0 ) {
                    // Three upper neighbors
                    if(tr[t]->uside == S_LEFT) {
                        tr[tn]->u0 = tr[t]->u1;
                        tr[t]->u1 = -1;
                        tr[tn]->u1 = tr[t]->usave;

                        tr[tr[t]->u0]->d0 = t;
                        tr[tr[tn]->u0]->d0 = tn;
                        tr[tr[tn]->u1]->d0 = tn;
                    } else {
                        // Intersects in the right
                        tr[tn]->u1 = -1;
                        tr[tn]->u0 = tr[t]->u1;
                        tr[t]->u1 = tr[t]->u0;
                        tr[t]->u0 = tr[t]->usave;

                        tr[tr[t]->u0]->d0 = t;
                        tr[tr[t]->u1]->d0 = t;
                        tr[tr[tn]->u0]->d0 = tn;
                    }

                    tr[t]->usave = tr[tn]->usave = 0;
                } else {
                    // No usave simple case
                    tr[tn]->u0 = tr[t]->u1;
                    tr[t]->u1 = tr[tn]->u1 = -1;
                    tr[tr[tn]->u0]->d0 = tn;
                }
            } else {
                // Fresh segment or upward cusp
                int tmp_u = tr[t]->u0;
                int td0, td1;
                if(((td0 = tr[tmp_u]->d0) > 0)
                        && ((td1 = tr[tmp_u]->d1) > 0)) {
                    // Upward cusp
                    if((tr[td0]->rseg > 0)
                            && !IsLeftOf(tr[td0]->rseg, s->v1)) {
                        tr[t]->u0 = tr[t]->u1 = tr[tn]->u1 = -1;
                        tr[tr[tn]->u0]->d1 = tn;
                    } else {
                        // Cusp going leftwards
                        tr[tn]->u0 = tr[tn]->u1 = tr[t]->u1 = -1;
                        tr[tr[t]->u0]->d0 = t;
                    }
                } else {
                    // Fresh segment
                    tr[tr[t]->u0]->d0 = t;
                    tr[tr[t]->u0]->d1 = tn;
                }
            }

            if(FP_EQUAL(tr[t]->lo->y, tr[tlast]->lo->y)
                    && FP_EQUAL(tr[t]->lo->x, tr[tlast]->lo->x)
                    && tribot) {
                // Bottom forms a triangle
                if(is_swapped) tmptriseg = seg[seg_num]->prev;
                else tmptriseg = seg[seg_num]->next;

                if((tmptriseg > 0) && IsLeftOf(tmptriseg,s->v0)) {
                    // L-R downward cusp
                    tr[tr[t]->d0]->u0 = t;
                    tr[tn]->d0 = tr[tn]->d1 = -1;
                } else {
                    // R-L downward cusp
                    tr[tr[tn]->d0]->u1 = tn;
                    tr[t]->d0 = tr[t]->d1 = -1;
                }
            } else {
                if((tr[tr[t]->d0]->u0 > 0) && (tr[tr[t]->d0]->u1 > 0)) {
                    if(tr[tr[t]->d0]->u0 == t) {
                        // Passes through LHS
                        tr[tr[t]->d0]->usave = tr[tr[t]->d0]->u1;
                        tr[tr[t]->d0]->uside = S_LEFT;
                    } else {
                        tr[tr[t]->d0]->usave = tr[tr[t]->d0]->u0;
                        tr[tr[t]->d0]->uside = S_RIGHT;
                    }
                }
                tr[tr[t]->d0]->u0 = t;
                tr[tr[t]->d0]->u1 = tn;
            }
            t = tr[t]->d0;
        } else if((tr[t]->d0 <= 0) && (tr[t]->d1 > 0)) {
            // Only one trapezoid below
            if((tr[t]->u0 > 0) && (tr[t]->u1 > 0)) {
                if(tr[t]->usave > 0) {
                    // Three upper neighbors
                    if(tr[t]->uside == S_LEFT) {
                        tr[tn]->u0 = tr[t]->u1;
                        tr[t]->u1 = -1;
                        tr[tn]->u1 = tr[t]->usave;

                        tr[tr[t]->u0]->d0 = t;
                        tr[tr[tn]->u0]->d0 = tn;
                        tr[tr[tn]->u1]->d0 = tn;
                    } else {
                        // Intersects in the right
                        tr[tn]->u1 = -1;
                        tr[tn]->u0 = tr[t]->u1;
                        tr[t]->u1 = tr[t]->u0;
                        tr[t]->u0 = tr[t]->usave;

                        tr[tr[t]->u0]->d0 = t;
                        tr[tr[t]->u1]->d0 = t;
                        tr[tr[tn]->u0]->d0 = tn;
                    }
                    tr[t]->usave = tr[tn]->usave = 0;
                } else {
                    // No usave simple case
                    tr[tn]->u0 = tr[t]->u1;
                    tr[t]->u1 = tr[tn]->u1 = -1;
                    tr[tr[tn]->u0]->d0 = tn;
                }
            } else {
                // Fresh segment or upward cusp
                int tmp_u = tr[t]->u0;
                int td0, td1;
                if(((td0 = tr[tmp_u]->d0) > 0)
                        && ((td1 = tr[tmp_u]->d1) > 0)) {
                    // Upward cusp
                    if((tr[td0]->rseg > 0)
                            && !IsLeftOf(tr[td0]->rseg, s->v1)) {
                        tr[t]->u0 = tr[t]->u1 = tr[tn]->u1 = -1;
                        tr[tr[tn]->u0]->d0 = t;
                    }
                } else {
                    // Fresh segment
                    tr[tr[t]->u0]->d0 = t;
                    tr[tr[t]->u0]->d1 = tn;
                }
            }

            if(FP_EQUAL(tr[t]->lo->y, tr[tlast]->lo->y)
                    && FP_EQUAL(tr[t]->lo->x, tr[tlast]->lo->x)
                    && tribot) {
                // Bottom forms a triangle
                int tmptriseg; /// I changed this from tmpseg since
                if(is_swapped)
                    tmptriseg = seg[seg_num]->prev; /// this
                else
                    tmptriseg = seg[seg_num]->next; /// and this are tmptriseg

                /// changed these tmptrisegs from tmpseg as well
                if((tmptriseg > 0) && IsLeftOf(tmptriseg, s->v0)) {
                    // L-R downward cusp
                    tr[tr[t]->d1]->u0 = t;
                    tr[tn]->d0 = tr[tn]->d1 = -1;
                } else {
                    // R-L downward cusp
                    tr[tr[tn]->d1]->u1 = tn;
                    tr[t]->d0 = tr[t]->d1 = -1;
                }
            } else {
                if((tr[tr[t]->d1]->u0 > 0) && (tr[tr[t]->d1]->u1 > 0)) {
                    if(tr[tr[t]->d1]->u0 == t) {
                        // Passes through LHS
                        tr[tr[t]->d1]->usave = tr[tr[t]->d1]->u1;
                        tr[tr[t]->d1]->uside = S_LEFT;
                    } else {
                        tr[tr[t]->d1]->usave = tr[tr[t]->d1]->u0;
                        tr[tr[t]->d1]->uside = S_RIGHT;
                    }
                }

                tr[tr[t]->d1]->u0 = t;
                tr[tr[t]->d1]->u1 = tn;
            }

            t = tr[t]->d1;
            cout << "t = tr[t]->d1 = " << t << endl;
        } else {
            // Two trapezoids below. Find out which one is intersected by
            // this segment and proceed down that one
            int tmpseg = tr[tr[t]->d0]->rseg;
            double y0, yt;
            shared_ptr<point> tmppt = make_shared<point>();
            int tnext;
            bool i_d0, i_d1;

            i_d0 = i_d1 = false;
            if(FP_EQUAL(tr[t]->lo->y, s->v0->y)) {
                if(tr[t]->lo->x > s->v0->x) i_d0 = true;
                else i_d1 = true;
            } else {
                tmppt->y = y0 = tr[t]->lo->y;
                yt = (y0 - s->v0->y)/(s->v1->y - s->v0->y);
                tmppt->x = s->v0->x + yt*(s->v1->x - s->v0->x);

                if(LessThan_(*tmppt, *tr[t]->lo)) i_d0 = true;
                else i_d1 = true;
            }

            // Check continuity from the top so that the lower neighbor
            // values are properly filled for the upper trapezoid
            if((tr[t]->u0 > 0) && (tr[t]->u1 > 0)) {
                // Continuation of a chain from above
                if(tr[t]->usave > 0) {
                    // Three upper neighbors
                    if(tr[t]->uside == S_LEFT) {
                        tr[tn]->u0 = tr[t]->u1;
                        tr[t]->u1 = -1;
                        tr[tn]->u1 = tr[t]->usave;

                        tr[tr[t]->u0]->d0 = t;
                        tr[tr[tn]->u0]->d0 = tn;
                        tr[tr[tn]->u1]->d0 = tn;
                    } else {
                        // Intersects in the right
                        tr[tn]->u1 = -1;
                        tr[tn]->u0 = tr[t]->u1;
                        tr[t]->u1 = tr[t]->u0;
                        tr[t]->u0 = tr[t]->usave;

                        tr[tr[t]->u0]->d0 = t;
                        tr[tr[t]->u1]->d1 = t;
                        tr[tr[tn]->u0]->d0 = tn;
                    }
                    tr[t]->usave = tr[tn]->usave = 0;
                } else {
                    // No usave simple case
                    tr[tn]->u0 = tr[t]->u1;
                    tr[tn]->u1 = -1;
                    tr[t]->u1 = -1;
                    tr[tr[tn]->u0]->d0 = tn;
                }
            } else {
                // Fresh segment or upward cusp
                int tmp_u = tr[t]->u0;
                int td0, td1;
                if(((td0 = tr[tmp_u]->d0) > 0)
                        && ((td1 = tr[tmp_u]->d1) > 0)) {
                    // Upward cusp
                    if((tr[td0]->rseg > 0)
                            && !IsLeftOf(tr[td0]->rseg,s->v1)) {
                        tr[t]->u0 = tr[t]->u1 = tr[tn]->u1 = -1;
                        tr[tr[tn]->u0]->d1 = tn;
                    } else {
                        tr[tn]->u0 = tr[tn]->u1 = tr[t]->u1 = -1;
                        tr[tr[t]->u0]->d0 = t;
                    }
                } else {
                    // Fresh segment
                    tr[tr[t]->u0]->d0 = t;
                    tr[tr[t]->u0]->d1 = tn;
                }
            }

            if(FP_EQUAL(tr[t]->lo->y, tr[tlast]->lo->y)
                    && FP_EQUAL(tr[t]->lo->x, tr[tlast]->lo->x)
                    && tribot) {
                // This case arises only at the lowest trapezoid
                // i.e. tlast, if the lower endpoint of the segment is
                // already inserted in the structure

                tr[tr[t]->d0]->u0 = t;
                tr[tr[t]->d0]->u1 = -1;
                tr[tr[t]->d1]->u0 = tn;
                tr[tr[t]->d1]->u1 = -1;

                tr[tn]->d0 = tr[t]->d1;
                tr[t]->d1 = tr[tn]->d1 = -1;

                tnext = tr[t]->d1;
            } else if(i_d0) {
                // Intersecting d0
                tr[tr[t]->d0]->u0 = t;
                tr[tr[t]->d0]->u1 = tn;
                tr[tr[t]->d1]->u0 = tn;
                tr[tr[t]->d1]->u1 = -1;

                // new code to determine the bottom neighbors of the
                // newly partitioned trapeszoid
                tr[t]->d1 = -1;
                tnext = tr[t]->d0;
            } else {
                // Intersecting d1
                tr[tr[t]->d0]->u0 = t;
                tr[tr[t]->d0]->u1 = -1;
                tr[tr[t]->d1]->u0 = t;
                tr[tr[t]->d1]->u1 = tn;

                // new code to determine the bottom neighbors of the
                // newly partitioned trapezoid
                tr[tn]->d0 = tr[t]->d1;
                tr[tn]->d1 = -1;
                tnext = tr[t]->d1;
            }
            t = tnext;
        }
        tr[t_sav]->rseg = tr[tn_sav]->lseg = seg_num;
    } // end while

    // Now combine those trapezoids which share common segments.
    // Can use the pointers to the parent to connect these.
    // This works only because all these new trapezoids have been formed
    // due to splitting by the segment and hence have only one parent

    tfirstl = tfirst;
    tlastl = tlast;
    cout << tr.size() << endl;
    for(int i = 1; i < tr.size(); i++) {
        cout << "lseg: " << tr[i]->lseg << endl;
        cout << "rseg: " << tr[i]->rseg << endl;
        cout << "tr[" <<i<< "]: " << endl;
        if(tr[i]->lseg > 0)
        cout << seg[tr[i]->lseg]->v0->x << "," << seg[tr[i]->lseg]->v0->y << " : "
             << seg[tr[i]->lseg]->v1->x << "," << seg[tr[i]->lseg]->v1->y
             << "\n--\n";
        if(tr[i]->rseg > 0)
        cout << seg[tr[i]->rseg]->v0->x << "," << seg[tr[i]->rseg]->v0->y << " : "
             << seg[tr[i]->rseg]->v1->x << "," << seg[tr[i]->rseg]->v1->y
             << endl;
    }
    exit(-1);
    MergeTrapezoids(seg_num,tfirstl,tlastl,S_LEFT);
    MergeTrapezoids(seg_num,tfirstr,tlastr,S_RIGHT);
    cout << "Merged trapezoids" << endl;

    seg[seg_num]->is_inserted = true;
    return 0;
}

// Update the roots stored for each of the endpoints of the segment
// This is done to speed up the location query for the endpoint
// when the segment is inserted into the trapezoidation subsequently
int FindNewRoots(int seg_num)
{
    shared_ptr<segment> s = seg[seg_num];

    if(s->is_inserted) return 0;

    s->root0 = LocateEndpoint(s->v0, s->v1, s->root0);
    s->root0 = tr[s->root0]->sink;

    s->root1 = LocateEndpoint(s->v1, s->v0, s->root1);
    s->root1 = tr[s->root1]->sink;
    return 0;
}


// Main routine to perform trapezoidation
int ConstructTrapezoids(int nseg)
{
    int root;

    // Add the first segment and get the qury structure and trapezoid
    // list initialized

    root = InitQueryStructure(ChooseSegment());
    cout << "Initialized query structure" << endl;

    for(int i = 1; i <= nseg; i++) {
        seg[i]->root0 = seg[i]->root1 = root;
    }

    cout << "set roots" << endl;

    for(int h = 1; h <= MathLogstarN(nseg); h++) {
        for(int i = MathN(nseg,h-1) + 1; i <= MathN(nseg,h); i++)
            AddSegment(ChooseSegment());

        cout << "added segments" << endl;

        // Find a new rot for each of the segment endpoints
        for(int i = 1; i <= nseg; i++)
            FindNewRoots(i);

        cout << "found new roots" << endl;
    }

    for(int i = MathN(nseg, MathLogstarN(nseg)) + 1; i <= nseg; i++)
        AddSegment(ChooseSegment());

    cout << "added more segments" << endl;

    return 0;
}


// misc.c
// Generate a random permutation of the segments 1...n
int GenerateRandomOrdering(int n)
{
    int m;
    vector<int> st(SEGSIZE);

    choose_idx = 1;

    for(int i = 0; i <= n; i++)
        st[i] = i;

    for(int i = 1; i <= n; i++) {
        /// TODO /////////////////////////////////////////////////////////////////////////
        /// Random number between 1 and n?
        m = RandDouble(1,n+1-i);
        permute[i] = st[m];
        if(m != 1) st[m] = st[1];
    }
    return 0;
}

// Return the next segment in the generated random ordering of
// all the segments in S
int ChooseSegment()
{ return permute[choose_idx++]; }

// Read in the list of vertices
int ReadSegments(Eigen::MatrixX2d polygon)
{
    int npoints, first, last;

    npoints = polygon.rows();

    // For every contour, read in all points
    // The outer-most contour is read in first (CCW)
    // Next the inner contours are input (CW)
    /// Here I'm assuming 1 simple polygon so there is 1 contour

    int i = 1;
    first = i;
    last = first + npoints - 1;
    for(int j = 0; j < npoints; j++, i++) {
        if(i == last) {
            seg[i]->next = first;
            seg[i]->prev = i-1;
            seg[i-1]->v1 = seg[i]->v0;
        } else if(i == first) {
            seg[i]->next = i+1;
            seg[i]->prev = last;
            seg[last]->v1 = seg[i]->v0;
        } else {
            seg[i]->prev = i-1;
            seg[i]->next = i+1;
            seg[i-1]->v1 = seg[i]->v0;
        }
        seg[i]->is_inserted = false;
    }
    return i-1;
}

// Get Log*n for given n
int MathLogstarN(int n)
{
    double v = (double) n;
    int i;
    for(i = 0; v >= 1; i++) {
        v = std::log2(v);
    }
    return (i-1);
}

int MathN(int n, int h)
{
    double v = (double) n;
    for(int i = 0; i < h; i++) {
        v = std::log2(v);
    }
    return (int) std::ceil((double) 1.0*n/v);
}

// monotone.c
// Function returns true if the trapezoid lies inside the polygon
bool InsidePolygon(shared_ptr<trapezoid> t)
{
    int rseg = t->rseg;

    if(t->state == ST_INVALID) return false;
    if((t->lseg <= 0) || (t->rseg <= 0)) return false;
    if(((t->u0 <= 0) && (t->u1 <= 0))
            || ((t->d0 <= 0) && (t->d1 <= 0))) // triangle
        return GreaterThan_(*seg[rseg]->v1, *seg[rseg]->v0);
    return false;
}

// Return a new mon structure from the table
int NewMon() { return ++mon_idx; }

// Return a new chain element from the table
int NewChainElement() { return ++ chain_idx; }

double GetAngle(shared_ptr<point> vp0,
                shared_ptr<point> vpnext,
                shared_ptr<point> vp1)
{
    shared_ptr<point> v0, v1;

    v0->x = vpnext->x - vp0->x;
    v0->y = vpnext->y - vp0->y;

    v1->x = vp1->x - vp0->x;
    v1->y = vp1->y - vp0->y;

    // Sine is positive
    if(CROSS_SINE(*v0,*v1) >= 0) return DOT(*v0,*v1)/LENGTH(*v0)/LENGTH(*v1);
    else return (1.0 * DOT(*v0,*v1)/LENGTH(*v0)/LENGTH(*v1) - 2);
}

// (v0,v1) is the new diagonal to be added to the polygon.
// Find which chain to use and return the position of v0 and v1 in p and q
int GetVertexPositions(int v0, int v1, shared_ptr<int> ip, shared_ptr<int> iq)
{
    shared_ptr<vertexchain> vp0, vp1;
    double angle, temp;
    int tp, tq;

    vp0 = vert[v0];
    vp1 = vert[v1];

    // P is identified as follows. Scan from (v0,v1) rightwards
    // until you hit the first segment starting from v0.
    // That chain is the chain of interest
    angle = -4.0;
    for(int i = 0; i < 4; i++) {
        if(vp0->vnext[i] <= 0) continue;
        if((temp = GetAngle(vp0->pt,vert[vp0->vnext[i]]->pt, vp1->pt)) > angle) {
            angle = temp;
            tp = i;
        }
    }
    *ip = tp;

    // Do similar actions for q
    angle = -4.0;
    for(int i = 0; i < 4; i++) {
        if(vp1->vnext[i] <= 0) continue;
        if((temp = GetAngle(vp1->pt, vert[vp1->vnext[i]]->pt, vp0->pt)) > angle) {
            angle = temp;
            tq = i;
        }
    }
    *iq = tq;

    return 0;
}

// v0 and v1 are specified in CCW order with respect to the current
// monotone polygon mcur. Split the current polygon into two polygons
// using the diagonal (v0,v1)
int MakeNewMonotonePoly(int mcur, int v0, int v1)
{
    int p, q;
    shared_ptr<int> ip, iq;
    int mnew = NewMon();
    int i, j, nf0, nf1;
    shared_ptr<vertexchain> vp0, vp1;

    vp0 = vert[v0];
    vp1 = vert[v1];

    GetVertexPositions(v0,v1,ip,iq);

    p = vp0->vpos[*ip];
    q = vp1->vpos[*iq];

    // At this stage we have the positions of v0 and v1 in the desired chain
    // Now modify the linked lists

    i = NewChainElement(); // for the new list
    j = NewChainElement();

    mchain[i]->vnum = v0;
    mchain[j]->vnum = v1;

    mchain[i]->next = mchain[p]->next;
    mchain[mchain[p]->next]->prev = i;
    mchain[i]->prev = j;
    mchain[j]->next = i;
    mchain[j]->prev = mchain[q]->prev;
    mchain[mchain[q]->prev]->next = j;

    mchain[p]->next = q;
    mchain[q]->prev = p;

    nf0 = vp0->next_free;
    nf1 = vp1->next_free;

    vp0->vnext[*ip] = v1;

    vp0->vpos[nf0] = i;
    vp0->vnext[nf0] = mchain[mchain[i]->next]->vnum;
    vp1->vpos[nf1] = j;
    vp1->vnext[nf1] = v0;

    vp0->next_free++;
    vp1->next_free++;

    mon[mcur] = p;
    mon[mnew] = i;
    return mnew;
}

// Recursively visit all the trapezoids
int TraversePolygon(int mcur, int trnum, int from, int dir)
{
    shared_ptr<trapezoid> t = tr[trnum];
    int howsplit, mnew;
    int v0, v1, v0next, v1next;
    int retval, tmp;
    bool do_switch = false;

    if((trnum <= 0) || visited[trnum]) return 0;

    visited[trnum] = true;

    // We have much more information available here
    // rseg: goes upwards
    // lseg: goes downwards

    // Initially assume that dir = TR_FROM_DN (from the left)
    // Switch v0 and v1 if necessary afterwards

    // Special cases for triangles with cusps at the opposite ends
    // Take care of these first
    if((t->u0 <= 0) && (t->u1 <= 0)) {
        if((t->d0 > 0) && (t->d1 > 0)) {
            // Downward opening triangle
            v0 = tr[t->d1]->lseg;
            v1 = t->lseg;
            if(from == t->d1) {
                do_switch = true;
                mnew = MakeNewMonotonePoly(mcur,v1,v0);
                TraversePolygon(mcur,t->d1,trnum,TR_FROM_UP);
                TraversePolygon(mnew,t->d0,trnum,TR_FROM_UP);
            } else {
                mnew = MakeNewMonotonePoly(mcur,v0,v1);
                TraversePolygon(mcur,t->d0,trnum,TR_FROM_UP);
                TraversePolygon(mnew,t->d1,trnum,TR_FROM_UP);
            }
        } else {
            retval = SP_NOSPLIT; // Just traverse all neighbors
            TraversePolygon(mcur,t->u0,trnum,TR_FROM_DN);
            TraversePolygon(mcur,t->u1,trnum,TR_FROM_DN);
            TraversePolygon(mcur,t->d0,trnum,TR_FROM_UP);
            TraversePolygon(mcur,t->d1,trnum,TR_FROM_UP);
        }
    } else if((t->d0 <= 0) && (t->d1 <= 0)) {
        if((t->u0 > 0) && (t->u1 > 0)) {
            // Upward opening triangle
            v0 = t->rseg;
            v1 = tr[t->u0]->rseg;
            if(from == t->u1) {
                do_switch = true;
                mnew = MakeNewMonotonePoly(mcur,v1,v0);
                TraversePolygon(mcur,t->u1,trnum,TR_FROM_DN);
                TraversePolygon(mnew,t->u0,trnum,TR_FROM_DN);
            } else {
                mnew = MakeNewMonotonePoly(mcur,v0,v1);
                TraversePolygon(mcur,t->u0,trnum,TR_FROM_DN);
                TraversePolygon(mnew,t->u1,trnum,TR_FROM_DN);
            }
        } else {
            retval = SP_NOSPLIT; // Just traverse all neighbors
            TraversePolygon(mcur,t->u0,trnum,TR_FROM_DN);
            TraversePolygon(mcur,t->u1,trnum,TR_FROM_DN);
            TraversePolygon(mcur,t->d0,trnum,TR_FROM_UP);
            TraversePolygon(mcur,t->d1,trnum,TR_FROM_UP);
        }
    } else if((t->u0 > 0) && (t->u1 > 0)) {
        if((t->d0 > 0) && (t->d1 > 0)) {
            // Downward + upward cusps
            v0 = tr[t->d1]->lseg;
            v1 = tr[t->u0]->rseg;
            retval = SP_2UP_2DN;
            if(((dir == TR_FROM_DN) && (t->d1 == from))
                    || ((dir == TR_FROM_UP) && (t->u1 == from))) {
                do_switch = true;
                mnew = MakeNewMonotonePoly(mcur,v1,v0);
                TraversePolygon(mcur,t->u1,trnum,TR_FROM_DN);
                TraversePolygon(mcur,t->d1,trnum,TR_FROM_UP);
                TraversePolygon(mnew,t->u0,trnum,TR_FROM_DN);
                TraversePolygon(mnew,t->d0,trnum,TR_FROM_UP);
            } else {
                mnew = MakeNewMonotonePoly(mcur,v0,v1);
                TraversePolygon(mcur,t->u0,trnum,TR_FROM_DN);
                TraversePolygon(mcur,t->d0,trnum,TR_FROM_UP);
                TraversePolygon(mnew,t->u1,trnum,TR_FROM_DN);
                TraversePolygon(mnew,t->d1,trnum,TR_FROM_UP);
            }
        } else {
            // Only downward cusp
            if(EqualTo_(*t->lo,*seg[t->lseg]->v1)) {
                v0 = tr[t->u0]->rseg;
                v1 = seg[t->lseg]->next;
                retval = SP_2UP_LEFT;
                if((dir == TR_FROM_UP) && (t->u0 == from)) {
                    do_switch = true;
                    mnew = MakeNewMonotonePoly(mcur,v1,v0);
                    TraversePolygon(mcur,t->u0,trnum,TR_FROM_DN);
                    TraversePolygon(mnew,t->d0,trnum,TR_FROM_UP);
                    TraversePolygon(mnew,t->u1,trnum,TR_FROM_DN);
                    TraversePolygon(mnew,t->d1,trnum,TR_FROM_UP);
                } else {
                    mnew = MakeNewMonotonePoly(mcur,v0,v1);
                    TraversePolygon(mcur,t->u1,trnum,TR_FROM_DN);
                    TraversePolygon(mcur,t->d0,trnum,TR_FROM_UP);
                    TraversePolygon(mcur,t->d1,trnum,TR_FROM_UP);
                    TraversePolygon(mnew,t->u0,trnum,TR_FROM_DN);
                }
            } else {
                v0 = t->rseg;
                v1 = tr[t->u0]->rseg;
                retval = SP_2UP_RIGHT;
                if((dir == TR_FROM_UP) && (t->u1 == from)) {
                    do_switch = true;
                    mnew = MakeNewMonotonePoly(mcur,v1,v0);
                    TraversePolygon(mcur,t->u1,trnum,TR_FROM_DN);
                    TraversePolygon(mnew,t->d1,trnum,TR_FROM_UP);
                    TraversePolygon(mnew,t->d0,trnum,TR_FROM_UP);
                    TraversePolygon(mnew,t->u0,trnum,TR_FROM_DN);
                } else {
                    mnew = MakeNewMonotonePoly(mcur,v0,v1);
                    TraversePolygon(mcur,t->u0,trnum,TR_FROM_DN);
                    TraversePolygon(mcur,t->d0,trnum,TR_FROM_UP);
                    TraversePolygon(mcur,t->d1,trnum,TR_FROM_UP);
                    TraversePolygon(mnew,t->u1,trnum,TR_FROM_DN);
                }
            }
        }
    } else if((t->u0 > 0) || (t->u1 > 0)) {
        // No downward cusp
        if((t->d0 > 0) && (t->d1 > 0)) {
            // Only upward cusp
            if(EqualTo_(*t->hi, *seg[t->lseg]->v0)) {
                v0 = tr[t->d1]->lseg;
                v1 = t->lseg;
                retval = SP_2DN_LEFT;
                if(!((dir == TR_FROM_DN) && (t->d0 == from))) {
                    do_switch = true;
                    mnew = MakeNewMonotonePoly(mcur,v1,v0);
                    TraversePolygon(mcur,t->u1,trnum,TR_FROM_DN);
                    TraversePolygon(mcur,t->d1,trnum,TR_FROM_UP);
                    TraversePolygon(mcur,t->u0,trnum,TR_FROM_DN);
                    TraversePolygon(mnew,t->d0,trnum,TR_FROM_UP);
                } else {
                    mnew = MakeNewMonotonePoly(mcur,v0,v1);
                    TraversePolygon(mcur,t->d0,trnum,TR_FROM_UP);
                    TraversePolygon(mnew,t->u0,trnum,TR_FROM_DN);
                    TraversePolygon(mnew,t->u1,trnum,TR_FROM_DN);
                    TraversePolygon(mnew,t->d1,trnum,TR_FROM_UP);
                }
            } else {
                v0 = tr[t->d1]->lseg;
                v1 = seg[t->rseg]->next;
                retval = SP_2DN_RIGHT;
                if((dir == TR_FROM_DN) && (t->d1 == from)) {
                    do_switch = true;
                    mnew = MakeNewMonotonePoly(mcur,v1,v0);
                    TraversePolygon(mcur,t->d1,trnum,TR_FROM_UP);
                    TraversePolygon(mnew,t->u1,trnum,TR_FROM_DN);
                    TraversePolygon(mnew,t->u0,trnum,TR_FROM_DN);
                    TraversePolygon(mnew,t->d0,trnum,TR_FROM_UP);
                } else {
                    mnew = MakeNewMonotonePoly(mcur,v0,v1);
                    TraversePolygon(mcur,t->u0,trnum,TR_FROM_DN);
                    TraversePolygon(mcur,t->d0,trnum,TR_FROM_UP);
                    TraversePolygon(mcur,t->u1,trnum,TR_FROM_DN);
                    TraversePolygon(mnew,t->d1,trnum,TR_FROM_UP);
                }
            }
        } else {
            // No cusp
            if(EqualTo_(*t->hi,*seg[t->lseg]->v0)
                    && EqualTo_(*t->lo, *seg[t->rseg]->v0)) {
                v0 = t->rseg;
                v1 = t->lseg;
                retval = SP_SIMPLE_LRDN;
                if(dir == TR_FROM_UP) {
                    do_switch = true;
                    mnew = MakeNewMonotonePoly(mcur,v1,v0);
                    TraversePolygon(mcur,t->u0,trnum,TR_FROM_DN);
                    TraversePolygon(mcur,t->u1,trnum,TR_FROM_DN);
                    TraversePolygon(mnew,t->d1,trnum,TR_FROM_UP);
                    TraversePolygon(mnew,t->d0,trnum,TR_FROM_UP);
                } else {
                    mnew = MakeNewMonotonePoly(mcur,v0,v1);
                    TraversePolygon(mcur,t->d1,trnum,TR_FROM_UP);
                    TraversePolygon(mcur,t->d0,trnum,TR_FROM_UP);
                    TraversePolygon(mnew,t->u0,trnum,TR_FROM_DN);
                    TraversePolygon(mnew,t->u1,trnum,TR_FROM_DN);
                }
            } else if(EqualTo_(*t->hi, *seg[t->rseg]->v1)
                      && EqualTo_(*t->lo, *seg[t->lseg]->v1)) {
                v0 = seg[t->rseg]->next;
                v1 = seg[t->lseg]->next;
                retval = SP_SIMPLE_LRUP;
                if(dir == TR_FROM_UP) {
                    do_switch = true;
                    mnew = MakeNewMonotonePoly(mcur,v1,v0);
                    TraversePolygon(mcur,t->u0,trnum,TR_FROM_DN);
                    TraversePolygon(mcur,t->u1,trnum,TR_FROM_DN);
                    TraversePolygon(mnew,t->d1,trnum,TR_FROM_UP);
                    TraversePolygon(mnew,t->d0,trnum,TR_FROM_UP);
                } else {
                    mnew = MakeNewMonotonePoly(mcur,v0,v1);
                    TraversePolygon(mcur,t->d1,trnum,TR_FROM_UP);
                    TraversePolygon(mcur,t->d0,trnum,TR_FROM_UP);
                    TraversePolygon(mnew,t->u0,trnum,TR_FROM_DN);
                    TraversePolygon(mnew,t->u1,trnum,TR_FROM_DN);
                }
            } else {
                // no split passible
                retval = SP_NOSPLIT;
                TraversePolygon(mcur,t->u0,trnum,TR_FROM_DN);
                TraversePolygon(mcur,t->d0,trnum,TR_FROM_UP);
                TraversePolygon(mcur,t->u1,trnum,TR_FROM_DN);
                TraversePolygon(mcur,t->d1,trnum,TR_FROM_UP);
            }
        }
    }
    return retval;
}

// Main routine to get monotone polygons from the trapezoidation
// of the polygon
int MonotonateTrapezoids(int n)
{
    // First locate a trapezoid which lies inside the polygon
    // and which is triangular
    int i;
    for(i = 0; i < TRSIZE; i++)
        if(InsidePolygon(tr[i])) break;
    int tr_start = i;

    // Initialize the mon data structure and start spanning all the
    // trapezoids within the polygon
    for(int i = 1; i <= n; i++) {
        mchain[i]->prev = seg[i]->prev;
        mchain[i]->next = seg[i]->next;
        mchain[i]->vnum = i;
        vert[i]->pt = seg[i]->v0;
        vert[i]->vnext[0] = seg[i]->next; // next vertex
        vert[i]->next_free = 1;
    }

    chain_idx = n;
    mon_idx = 0;
    mon[0] = 1; // position of any vertex in the first chain

    // Traverse the polygon
    if(tr[tr_start]->u0 > 0)
        TraversePolygon(0,tr_start,tr[tr_start]->u0,TR_FROM_UP);
    else if(tr[tr_start]->d0 > 0)
        TraversePolygon(0,tr_start,tr[tr_start]->d0,TR_FROM_DN);

    // Return the number of polygons created
    return NewMon();
}

// A greedy corner-cutting algorithm to triangulate a y-monotone
// polygon in O(n) time
// Joseph O-Rourk, Computational Geometry in C.
int TriangulateSinglePolygon(int nvert, int posmax, int side, Eigen::MatrixX3d op) {
    vector<int> rc(SEGSIZE); // reflex chain
    int ri = 0;
    int endv, tmp, vpos;
    int v;

    if(side == TRI_RHS) {
        // RHS segment is a single segment
        rc[0] = mchain[posmax]->vnum;
        tmp = mchain[posmax]->next;
        rc[1] = mchain[tmp]->vnum;
        ri = 1;

        vpos = mchain[tmp]->next;
        v = mchain[vpos]->vnum;

        if((endv = mchain[mchain[posmax]->prev]->vnum) == 0) endv = nvert;
    } else {
        // LHS is a single segment
        tmp = mchain[posmax]->next;
        rc[0] = mchain[tmp]->vnum;
        tmp = mchain[tmp]->next;
        rc[1] = mchain[tmp]->vnum;
        ri = 1;

        vpos = mchain[tmp]->next;
        v = mchain[vpos]->vnum;

        endv = mchain[posmax]->vnum;
    }

    while((v != endv) || (ri > 1)) {
        if(ri > 0) {
            // Reflex chain is non-empty
            if(CROSS(*vert[v]->pt, *vert[rc[ri-1]]->pt, *vert[rc[ri]]->pt) > 0) {
                // Convex corner: OFF WITH ITS HEAD
                op.resize(op_idx,Eigen::NoChange_t());
                op(op_idx,0) = rc[ri-1];
                op(op_idx,1) = rc[ri];
                op(op_idx,2) = v;
                op_idx++;
                ri--;
            } else {
                // Non-convex
                // Add v to the chain
                ri++;
                rc[ri] = v;
                vpos = mchain[vpos]->next;
                v = mchain[vpos]->vnum;
            }
        } else {
            // Reflex chain empty: add v to the reflex chain and advance it
            rc[++ri] = v;
            vpos = mchain[vpos]->next;
            v = mchain[vpos]->vnum;
        }
    } // end while

    // Reached bottom vertex
    // Add in the triange formed
    op(op_idx,0) = rc[ri-1];
    op(op_idx,1) = rc[ri];
    op(op_idx,2) = v;
    op_idx++;
    ri--;

    return 0;
}

// For each monotype polygon, find the ymax and ymin to determino
// two y-monotone chains and pass on this monotone polygon for greedy
// triangulation. Take care not to triangulate duplicate monotone polygons
int TriangulateMonotonePolygons(int nvert, int nmonpoly, Eigen::MatrixX3d op)
{
    shared_ptr<point> ymax, ymin;
    int p, vfirst, posmax, posmin, v;
    int vcount;
    bool processed;

    op_idx = 0;
    for(int i = 0; i < nmonpoly; i++) {
        vcount = 1;
        processed = false;
        vfirst = mchain[mon[i]]->vnum;
        ymax = ymin = vert[vfirst]->pt;
        posmax = posmin = mon[i];
        mchain[mon[i]]->marked = true;
        p = mchain[mon[i]]->next;
        while((v = mchain[p]->vnum) != vfirst) {
            if(mchain[p]->marked) {
                processed = true;
                break;
            } else {
                mchain[p]->marked = true;
            }

            if(GreaterThan_(*vert[v]->pt, *ymax)) {
                ymax = vert[v]->pt;
                posmax = p;
            }
            if(LessThan_(*vert[v]->pt, *ymin)) {
                ymin = vert[v]->pt;
                posmin = p;
            }
            p = mchain[p]->next;
            vcount++;
        }

        if(processed) continue; // go to next polygon

        if(vcount == 3) {
            // Already a triangle
            op.resize(op_idx,Eigen::NoChange_t());
            op(op_idx,0) = mchain[p]->vnum;
            op(op_idx,1) = mchain[mchain[p]->next]->vnum;
            op(op_idx,2) = mchain[mchain[p]->prev]->vnum;
            op_idx++;
        } else {
            // Triangulate the polygon
            v = mchain[mchain[posmax]->next]->vnum;
            if(EqualTo_(*vert[v]->pt, *ymin)) {
                // LHS is a single line
                TriangulateSinglePolygon(nvert,posmax,TRI_LHS,op);
            } else {
                TriangulateSinglePolygon(nvert,posmax,TRI_RHS,op);
            }
        }
    }
    return op_idx;
}

// tri.c
int Initialize(int n)
{
    for(int i = 1; i <= n; i++) {
        seg[i]->is_inserted = false;
    }
    GenerateRandomOrdering(n);
    return 0;
}

int InitializeSeg(int nsegs)
{
    seg.resize(nsegs+1);
    for(int i = 0; i <= nsegs; i++) {
        seg[i] = make_shared<segment>();
        seg[i]->v0 = make_shared<point>();
        seg[i]->v1 = make_shared<point>();
    }
    return 0;
}

Eigen::MatrixX3d TriangulatePolygon(Eigen::MatrixX2d polygon)
{
    int nmonpoly, first, last, n;
    Eigen::MatrixX3d triangles;
    int npoints = polygon.rows();
    int i = 1;
    first = i;
    last = first + npoints - 1;
    InitializeSeg(npoints);
    for(int j = 0; j < npoints; j++, i++) {
        seg[i]->v0->x = polygon(j,0);
        seg[i]->v0->y = polygon(j,1);

        if(i == last) {
            seg[i]->next = first;
            seg[i]->prev = i-1;
            seg[i-1]->v1 = seg[i]->v0;
        } else if(i == first) {
            seg[i]->next = i+1;
            seg[i]->prev = last;
            seg[last]->v1 = seg[i]->v0;
        } else {
            seg[i]->prev = i-1;
            seg[i]->next = i+1;
            seg[i-1]->v1 = seg[i]->v0;
        }
        seg[i]->is_inserted = false;
    }
    n = i-1;

    Initialize(n);
    cout << "Initialized" << endl;
    ConstructTrapezoids(n);
    cout << "Constructed Trapezoids" << endl;
    nmonpoly = MonotonateTrapezoids(n);
    TriangulateMonotonePolygons(n, nmonpoly, triangles);

    return triangles;
}

// main function for testing the triangulate polygon framework
int main(int argc, char* argv[])
{
    // Polygon vertices in CCW order
    Eigen::MatrixX2d polygon_to_triangulate;
    polygon_to_triangulate.resize(4,Eigen::NoChange_t());

    polygon_to_triangulate(0,0) = 0;
    polygon_to_triangulate(0,1) = 0;

    polygon_to_triangulate(1,0) = 1;
    polygon_to_triangulate(1,1) = 0;

    polygon_to_triangulate(2,0) = 1;
    polygon_to_triangulate(2,1) = 1;

    polygon_to_triangulate(3,0) = 0;
    polygon_to_triangulate(3,1) = 1;

    cout << "Created test polygon" << endl;

    Eigen::MatrixX3d triangles = TriangulatePolygon(polygon_to_triangulate);
    for(int i = 0; i < triangles.rows(); i++) {
        cout << "Triangle #" << i+1 << ": " << triangles(i,0)
             << ", " << triangles(i,1) << ", " << triangles(i,2) << endl;
    }

    return 0;
}
