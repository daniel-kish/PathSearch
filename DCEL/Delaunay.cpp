#define _USE_MATH_DEFINES
#include "Delaunay.h"
#include <cmath>
#include <cassert>
#include <random>
#include <queue>

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
	// preconditions: at least two faces should already be present
	assert(d.faces.size() > 1);
	assert(h->face != d.out_face);
	if (h->next->next->next == h) return h;

	auto a = h->prev->target;
	auto b = h->target;
	auto c = h->next->target;

	if (side(a->p, c->p, b->p) != Side::right)
		return h->next;

	Circle circle = circumCircle(a->p, b->p, c->p);
	auto i = h->next->next;
	do {
		if (inCircle(circle, i->target->p))
			return h->next;
		i = i->next;
	} while (i != h->prev);

	d.split_face(h->next, a);
	return h->prev->twin;
}

void polygon_delaunay_triangulation(DCEL& d, DCEL::FaceList::iterator in_face)
{
	assert(d.faces.size() > 1);
	assert(in_face != d.out_face);

	auto h = in_face->halfedge;
	DCEL::EdgeList::iterator rh{h};
	do {
		h = rh;
		rh = clip_delaunay_ear(d,h);
	} while (rh != h);
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
	return g->next->twin;
}

// {h,true} if inside
std::tuple<DCEL::EdgeList::iterator,bool> 
localize(DCEL& d, Point const& p)
{
	for (auto f = d.faces.begin(); f != d.faces.end(); ++f)
	{
		if (f == d.out_face) continue;
		auto h = f->halfedge;
		Point const& a = h->prev->target->p;
		Point const& b = h->target->p;
		Point const& c = h->next->target->p;
		
		Point closest;  TriPos pos;
		std::tie(closest,pos) = ClosestPointOnTriangle(p,a,b,c);

		if (!zero(dist(closest, p))) 
			continue;
		// otherwise
		switch (pos)
		{
		case TriPos::V0:
		case TriPos::V1:
		case TriPos::V2: 
			return {d.halfedges.end(),false}; // ignore existing point
		case TriPos::edge01: return {h,false};
		case TriPos::edge12: return{h->next,false};
		case TriPos::edge20: return{h->prev,false};
		case TriPos::inside: return {h,true};
		}
	}
	// if 'p' is an outer point - ignore it
	return {d.halfedges.end(),false};
}

void insert_point_inside(DCEL& d, DCEL::EdgeList::iterator h, Point const& p)
{
	d.add_vertex(p, h);
	auto g = h->next;
	d.split_face(g,h->prev->target);
	g = g->next->twin;
	d.split_face(g, g->prev->prev->target);
}

void insert_point_on_edge(DCEL& d, DCEL::EdgeList::iterator h, Point const& p)
{
	d.split_edge(h, p);
	d.split_face(h, h->prev->prev->target);
	if (h->twin->face != d.out_face)
		d.split_face(h->twin->prev, h->twin->next->target);
}

void insert_point(DCEL& d, Point const& p)
{
	DCEL::EdgeList::iterator h; bool inside;
	std::tie(h,inside) = localize(d, p);
	if (h == d.halfedges.end()) return;

	std::queue<DCEL::EdgeList::iterator> q;
	if (inside) {
		insert_point_inside(d, h, p);
		q.push(h);
		h = h->next->twin->next;
		q.push(h);
		h = h->next->twin->next;
		q.push(h);
	}
	else {
		insert_point_on_edge(d, h, p);
		h = h->prev;
		q.push(h);

		if (h->next->twin->face != d.out_face) {
			h = h->next->twin->next;
			q.push(h);
			h = h->next->twin->next;
			q.push(h);
			h = h->next->twin->next;
			q.push(h);
		}
		else {
			h = h->prev->twin->prev;
			q.push(h);
		}
	}
	while (!q.empty())
	{
		auto h = q.front(); q.pop();
		if (h->twin->face == d.out_face) continue;
		if (!localDelaunay(d, h))
		{
			h = flip(d, h);
			q.push(h->prev);
			q.push(h->twin->next);
		}
	}
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

std::vector<Point> circleHull(Circle c, int nsteps)
{
	double step = 2.0*M_PI / nsteps;
	std::vector<Point> pts; pts.reserve(nsteps);

	for (double phi = 0.0; phi < 2.0*M_PI; phi += step)
		pts.push_back( {c.rad*cos(phi), c.rad*sin(phi)} );
	
	if (zero(dist(pts.back(), pts.front())))
		pts.pop_back();
	pts.shrink_to_fit();
	
	return pts;
}