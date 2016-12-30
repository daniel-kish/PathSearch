#pragma once
#include "DCEL.h"
#include "geometry.h"

DCEL::EdgeList::iterator clip_ear(DCEL& d, DCEL::EdgeList::iterator h);
DCEL::EdgeList::iterator clip_delaunay_ear(DCEL& d, DCEL::EdgeList::iterator h);
void polygon_delaunay_triangulation(DCEL& d, DCEL::FaceList::iterator in_face);

bool localDelaunay(DCEL& dcel, DCEL::EdgeList::iterator h);
DCEL::EdgeList::iterator flip(DCEL& d, DCEL::EdgeList::iterator h);

// TODO return halfedge and 0,1 - inside on edge
DCEL::EdgeList::iterator localizeTriangle(DCEL& d, Point const& p)
{
	for (auto f = d.faces.begin(); f != d.faces.end(); ++f)
	{
		auto h = f->halfedge;
		Point const& a = h->prev->target->p;
		Point const& b = h->target->p;
		Point const& c = h->next->target->p;
		if (insideTriangle(a, b, c, p) == Position::in)
			return h;
	}
}


std::vector<Point> rectHull(Rect r, int x_pts, int y_pts);
std::vector<Point> rectInsides(Rect r, int x_pts, int y_pts);
std::vector<Point> rectInsidesRand(Rect r, int n_pts);