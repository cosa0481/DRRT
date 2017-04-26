#ifndef __tripolyinterface_h
#define __tripolyinterface_h

#ifdef __cplusplus
    extern "C" int triangulate_polygon(int, double (*)[2], int (*)[3]);
    extern "C" int is_point_inside_polygon(double *);
#endif

#endif /* __tripolyinterface_h */
