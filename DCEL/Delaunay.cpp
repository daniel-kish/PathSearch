#define _USE_MATH_DEFINES
#include "Delaunay.h"
#include <cmath>
#include <cassert>
#include <random>
#include <deque>
#include <set>

DCEL::EdgeList::iterator clip_ear(DCEL& d, DCEL::EdgeList::iterator h)
{
	auto a = h->prev->target;
	auto b = h->target;
	auto c = h->next->target;

	if (h->next->next->target == a) // it's a triangle
		return h;
	if (side(a->p, c->p, b->p) != Side::right) // not an ear
		return h->next;

	auto i = h->next->next;
	do {
		if (insideTriangle(a->p, b->p, c->p, i->target->p) != Position::out) // not an ear
			return h->next;
		i = i->next;
	} while (i != h->prev);

	d.split_face(h->next, a);
	return h->next->next->twin;
}

DCEL::EdgeList::iterator clip_delaunay_ear(DCEL& d, DCEL::EdgeList::iterator h)
{
	// preconditions: at least two faces should already be built
	assert(d.faces.size() > 1);
	assert(h->face != d.out_face);
}

bool localDelaunay(DCEL& dcel, DCEL::EdgeList::iterator h)
{
	// preconditions:
	assert(h->face != dcel.out_face && h->twin->face != dcel.out_face);

	auto g = h->twin;
	auto a = h->next->target;
	auto b = g->next->target;
	auto c = h->target;
	auto d = g->target;

	Point ac = c->p - a->p;
	Point ad = d->p - a->p;
	Point bc = c->p - b->p;
	Point bd = d->p - b->p;

	double alpha = std::acos((ac*ad) / norm(ac) / norm(ad));
	double beta = std::acos((bc*bd) / norm(bc) / norm(bd));
	return alpha + beta < M_PI;
}

DCEL::EdgeList::iterator flip(DCEL& d, DCEL::EdgeList::iterator h)
{
	// preconditions:
	assert(h->face != d.out_face && h->twin->face != d.out_face);

	auto g = h->next;
	auto v = h->twin->next->target;
	d.join_face(h);
	d.split_face(g, v);
	return g->next;
}




std::vector<Point> rectHull(Rect r, int x_pts, int y_pts)
{
	double wid = std::abs(r.dir.x);
	double height = std::abs(r.dir.y);
	double xStep = wid / x_pts;
	double yStep = height / y_pts;

	std::vector<Point> pts; pts.reserve(2 * x_pts + 2 * x_pts);
	for (double x = 0.0; x < wid; x += xStep)
		pts.push_back(Point{x,0.0});
	for (double y = 0.0; y < height; y += yStep)
		pts.push_back(Point{wid,y});
	for (double x = wid; x > 0.0; x -= xStep)
		pts.push_back(Point{x,height});
	for (double y = height; y > 0.0; y -= yStep)
		pts.push_back(Point{0.0,y});

	pts.shrink_to_fit();

	if (r.origin != Point{0.0,0.0})
		for (Point& p : pts)
			p = p + r.origin;

	return pts;
}

std::vector<Point> rectInsides(Rect r, int x_pts, int y_pts)
{
	std::vector<Point> pts;
	double xb = r.origin.x, xe = r.origin.x + r.dir.x;
	double yb = r.origin.y, ye = r.origin.y + r.dir.y;
	double x_step = (xe - xb) / x_pts;
	double y_step = (ye - yb) / y_pts;

	for (double x = xb + x_step; x < xe; x += x_step)
		for (double y = yb + y_step; y < ye; y += y_step)
			pts.push_back({x,y});
	return pts;
}

std::vector<Point> rectInsidesRand(Rect r, int n_pts)
{
	std::vector<Point> pts; pts.reserve(n_pts);
	double xb = r.origin.x, xe = r.origin.x + r.dir.x;
	double yb = r.origin.y, ye = r.origin.y + r.dir.y;
	std::mt19937 mt{};
	std::uniform_real_distribution<double> xd(xb, xe);
	std::uniform_real_distribution<double> yd(yb, ye);

	while (n_pts--)
		pts.push_back({xd(mt),yd(mt)});
	return pts;
}