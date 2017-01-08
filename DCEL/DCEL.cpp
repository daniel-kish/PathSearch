#include "DCEL.h"
#include "Point.h"
#include <cassert>
#include "geometry.h"
#include <stack>

DCEL::DCEL(Point p, Point q)
{
	auto f0 = newFace();
	out_face = f0;
	auto h0 = newHalfedge();
	auto h1 = newHalfedge();
	auto v0 = newVertex(p);
	auto v1 = newVertex(q);

	v0->halfedge = h1;
	v1->halfedge = h0;
	f0->halfedge = h0;

	h0->face = f0;
	h0->next = h1;
	h0->prev = h1;
	h0->target = v0;
	h0->twin = h1;

	h1->face = f0;
	h1->next = h0;
	h1->prev = h0;
	h1->target = v1;
	h1->twin = h0;
}

DCEL::DCEL(DCEL&& from) {
	vertices = std::move(from.vertices);
	halfedges = std::move(from.halfedges);
	faces = std::move(from.faces);
	out_face = from.out_face;
}

DCEL& DCEL::operator=(DCEL && from)
{
	vertices = std::move(from.vertices);
	halfedges = std::move(from.halfedges);
	faces = std::move(from.faces);
	out_face = from.out_face;
	return *this;
}


DCEL::VertexList::iterator DCEL::newVertex(Point const& p)
{
	vertices.push_front({p});
	return vertices.begin();
}

DCEL::FaceList::iterator DCEL::newFace()
{
	int i = 0;
	if (!faces.empty()) i = faces.begin()->i + 1;
	faces.push_front(Face{nullptr,i,{}});
	return faces.begin();
}

DCEL::EdgeList::iterator DCEL::newHalfedge()
{
	int i = 0;
	if (!halfedges.empty()) i = halfedges.begin()->i + 1;
	halfedges.push_front({i});
	return halfedges.begin();
}

void DCEL::print()
{
	std::cout << "faces: \n";
	for (Face const& f : faces)
		std::cout << 'f' << f.i << " h" << f.halfedge->i << '\n';

	std::cout << "vertices: \n";
	for (Vertex const& v : vertices)
		std::cout << v.p << " h" << v.halfedge->i << '\n';

	std::cout << "halfedges: \n";
	for (Halfedge const& h : halfedges) {
		std::cout << 'h' << h.i << ":\n";
		std::cout << "face " << 'f' << h.face->i << '\n';
		std::cout << "next " << 'h' << h.next->i << '\n';
		std::cout << "prev " << 'h' << h.prev->i << '\n';
		std::cout << "target " << h.target->p << '\n';
		std::cout << "twin " << 'h' << h.twin->i << '\n';
	}
	std::cout << '\n';
}

void DCEL::add_vertex(Point p, DCEL::EdgeList::iterator h)
{
	auto v = newVertex(p);
	auto h1 = newHalfedge();
	auto h2 = newHalfedge();

	v->halfedge = h2;
	h1->twin = h2;
	h2->twin = h1;

	h1->target = v;
	h2->target = h->target;

	h1->face = h->face;
	h2->face = h->face;

	h1->next = h2;
	h2->next = h->next;
	h1->prev = h;

	h2->prev = h1;
	h->next = h1;
	h2->next->prev = h2;
}

void DCEL::split_face(EdgeList::iterator h, VertexList::iterator v)
{
	auto f = h->face;
	auto u = h->target;
	auto f1 = newFace();
	auto f2 = newFace();
	auto h1 = newHalfedge();
	auto h2 = newHalfedge();

	f1->halfedge = h1;
	f2->halfedge = h2;

	h1->twin = h2;
	h2->twin = h1;

	h1->target = v;
	h2->target = u;
	h2->next = h->next;

	h2->next->prev = h2;
	h1->prev = h;
	h->next = h1;

	auto i = h2;
	while (true) {
		i->face = f2;
		if (i->target == v) break;
		i = i->next;
	}
	h1->next = i->next;
	h1->next->prev = h1;
	i->next = h2;
	h2->prev = i;
	i = h1;
	do {
		i->face = f1;
		i = i->next;
	} while (i != h1);
	faces.erase(f);
}

void DCEL::split_edge(EdgeList::iterator cb, Point p)
{
	auto b = newVertex(p);

	auto ba = newHalfedge();
	auto bc = newHalfedge();

	// save
	auto ab = cb->twin;
	auto ag = cb->next;
	auto ce = ab->next;
	auto c = ab->target;
	auto a = cb->target;

	// go
	ab->twin = ba;
	ab->next = bc;
	ab->target = b;

	bc->face = ab->face;
	bc->twin = cb;
	bc->next = ce;
	bc->prev = ab;
	bc->target = c;

	cb->twin = bc;
	cb->next = ba;
	cb->target = b;
	
	ba->face = cb->face;
	ba->twin = ab;
	ba->next = ag;
	ba->prev = cb;
	ba->target = a;

	b->halfedge = ba;
	ag->prev = ba;
	ce->prev = bc;
}

void DCEL::join_face(EdgeList::iterator h)
{
	auto g = h->twin;
	auto f1 = h->face;
	auto f2 = g->face;
	
	assert(f1 != f2);

	f1->halfedge = h->next;
	f2->halfedge = g->next;

	h->next->prev = g->prev;
	g->prev->next = h->next;

	g->next->prev = h->prev;
	h->prev->next = g->next;

	h->target->halfedge = h->next;
	g->target->halfedge = g->next;

	auto i = g->next;
	while (true) {
		i->face = f1;
		i = i->next;
		if (i == h->next) break;
	}
	halfedges.erase(g);
	halfedges.erase(h);
	faces.erase(f2);
}

bool is_out_face(DCEL::FaceList::iterator f)
{
	auto h = f->halfedge;
	auto i = h;

	std::vector<Point> poly;

	while (true) {
		poly.push_back(i->target->p);
		i = i->next;
		if (i == h) break;
	}
	auto or = orientation(poly);
	return or == PolyOrientation::CW;
}

DCEL mk_CCW_poly(std::vector<Point> const& poly)
{
	{
		// precondition: poly is oriented 'CCW', poly.size() > 2
		assert(poly.size() > 2);
		assert(orientation(poly) == PolyOrientation::CCW);
	}

	DCEL d(poly[0], poly[1]);
	auto v0 = std::next(d.vertices.begin());
	assert(v0->p == poly[0]);

	auto h = v0->halfedge;
	for (auto point = poly.begin() + 2; point != poly.end(); ++point) {
		d.add_vertex(*point, h);
		h = h->next;
	}
	d.split_face(h, v0);

	{
		// postcondition: exactly two faces, one is CW, 
		// poly.size() vertices, 2*poly.size() halfedges
		assert(d.faces.size() == 2);
		assert(is_out_face(d.faces.begin()));
		assert(d.vertices.size() == poly.size());
		assert(d.halfedges.size() == 2 * poly.size());
	}
	return d;
}


