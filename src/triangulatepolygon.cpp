#include <DRRT/triangulatepolygon.h>

using namespace std;

// construct.c
vector<shared_ptr<node>> qs(QSIZE);         // query structure
vector<shared_ptr<trapezoid>> tr(TRSIZE);   // trapezoid structure
vector<shared_ptr<segment>> seg(SEGSIZE);   // segment table

int q_idx;
int tr_idx;

// Return a new node to be added to the query tree
int NewNode()
{
    if(q_idx < QSIZE) return q_idx++;
    else {
        cout << "NewNode:\tQuery table overflow" << endl;
        return -1;
    }
}

// Return a free trapezoid
int NewTrap()
{
    if(tr_idx < TRSIZE) {
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
    int t, tnext;
    bool cond;
    int ptnext;

    // First merge polys on the LHS
    t = tfirst;
    while((t > 0) && GreaterThanEqualTo_(*tr[t]->lo, *tr[tlast]->lo)) {
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

        if(cond) {
            if((tr[t]->lseg == tr[tnext]->lseg)
                    && (tr[t]->rseg == tr[tnext]->rseg)) {
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
                tr[tnext]->state = ST_INVALID; // invalidate the lower trapezium
            } else t = tnext; // not good neighbors
        } else t = tnext; // do not satisfy the outer if
    }
    return 0;
}

// Add in the new segment into the trapezoidation and update qs and tr
// structures. First locate the two endpoints of the segment in the qs
// structure. Then start from the topmost trapezoid and go down to
// the lower trapezoid dividing all the trapezoids in betwen
int AddSegment(int seg_num)
{
    shared_ptr<segment> s;
    shared_ptr<segment> so = seg[seg_num];
    int tu, tl, sk, tfirst, tlast, tnext;
    int tfirstr, tlastr, tfirstl, tlastl;
    int i1, i2, t, t1, t2, tn;
    shared_ptr<point> tpt;
    int tritop = 0, tribot = 0;
    bool is_swapped = false;
    int tmptriseg;

    s = seg[seg_num];
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
        tfirst = tl;
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
        } else {
            // Two trapezoids below. Find out which one is intersected by
            // this segment and proceed down that one
            int tmpseg = tr[tr[t]->d0]->rseg;
            double y0, yt;
            shared_ptr<point> tmppt;
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
    MergeTrapezoids(seg_num,tfirstl,tlastl,S_LEFT);
    MergeTrapezoids(seg_num,tfirstr,tlastr,S_RIGHT);

    seg[seg_num]->is_inserted = true;
    return 0;
}

// Update the roots stored for each of the endpoints of the segment
// This is done to speed up the location query for the endpoint
// when the segment is inserted into the trapezoidation subsequently
int FindNewRoots(int seg_num)
{
    return 0;
}


// Main routine to perform trapezoidation
int ConstructTrapezoids(int nseg)
{
    return 0;
}
