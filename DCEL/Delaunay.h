#pragma once
#include "DCEL.h"
#include "geometry.h"
#include <tuple>

DCEL::EdgeList::iterator clip_ear(DCEL& d, DCEL::EdgeList::iterator h);
DCEL::EdgeList::iterator clip_delaunay_ear(DCEL& d, DCEL::EdgeList::iterator h);
void polygon_delaunay_triangulation(DCEL& d, DCEL::FaceList::iterator in_face);

bool localDelaunay(DCEL& dcel, DCEL::EdgeList::iterator h);
DCEL::EdgeList::iterator flip(DCEL& d, DCEL::EdgeList::iterator h);

// {h,true} if inside
std::tuple<DCEL::EdgeList::iterator, bool> localize(DCEL& d, Point const& p);
void insert_point(DCEL& d, Point const& p);
void insert_point_inside(DCEL& d, DCEL::EdgeList::iterator h, Point const& p);
void insert_point_on_edge(DCEL& d, DCEL::EdgeList::iterator h, Point const& p);

bool check_triangulation(DCEL& d);

std::tuple<DCEL,treeNode*> initialize_Delaunay_Kirkpatrick(double coord);

std::vector<Point> rectHull(Rect r, int x_pts, int y_pts);
std::vector<Point> rectInsides(Rect r, int x_pts, int y_pts);
std::vector<Point> rectInsidesRand(Rect r, int n_pts);
std::vector<Point> circleHull(Circle c, int nsteps);