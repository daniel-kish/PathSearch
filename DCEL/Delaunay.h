#pragma once
#include "DCEL.h"
#include "geometry.h"

DCEL::EdgeList::iterator clip_ear(DCEL& d, DCEL::EdgeList::iterator h);

bool localDelaunay(DCEL& dcel, DCEL::EdgeList::iterator h);
DCEL::EdgeList::iterator flip(DCEL& d, DCEL::EdgeList::iterator h);
void toDelaunay(DCEL& d);
std::vector<Point> rectHull(Rect r, int x_pts, int y_pts);
std::vector<Point> rectInsides(Rect r, int x_pts, int y_pts);
std::vector<Point> rectInsidesRand(Rect r, int n_pts);