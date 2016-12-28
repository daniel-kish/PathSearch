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

bool localDelaunay(DCEL& dcel, DCEL::EdgeList::iterator h)
{
	// preconditions:
	//assert(h->face != dcel.out_face && h->twin->face != dcel.out_face);

	if (h->face == dcel.out_face && h->twin->face == dcel.out_face)
		return true;

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

bool nonstrict_global(DCEL& d)
{
	for (auto h = d.halfedges.begin(); h != d.halfedges.end();)
	{
		if (h->face == d.out_face || h->twin->face == d.out_face) {
			++h;
			continue;
		}
		if (!localDelaunay(d, h)) {
			auto next = h->next;
			h = flip(d, h);
			if (localDelaunay(d, h))
				return false;
			h = next;
		}
		else
			++h;
	}
}

void toDelaunay(DCEL& d)
{
	/*bool more = false;
	do
	{
		more = false;
		for (auto h = d.halfedges.begin(); h != d.halfedges.end();)
		{
			if (h->face == d.out_face || h->twin->face == d.out_face) {
				++h;
				continue;
			}
			if (!localDelaunay(d, h)) {
				auto next = h->next;
				h = flip(d, h);
				if (localDelaunay(d, h)) 
					more = true;
				h = next;
			}
			else
				++h;
		}
	} while (more);*/
	auto comp = [](DCEL::EdgeList::iterator h, DCEL::EdgeList::iterator g) {
		if (h->twin == g)
			return false;
		return h->i < g->i;
	};
	std::set<DCEL::EdgeList::iterator, decltype(comp)> s(comp);


	while (true)
	{
		s.clear();
		for (auto h = d.halfedges.begin(); h != d.halfedges.end(); ++h) {
			if (!localDelaunay(d, h))
				s.insert(h);
		}
		bool more = false;
		for (DCEL::EdgeList::iterator h : s)
		{
			h = flip(d, h);
			if (localDelaunay(d, h)) {
				more = true;
				break;
			}
		}
		if (!more) break;
		//std::cout << "here\n";
	}
	//std::cout << "done\n";
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