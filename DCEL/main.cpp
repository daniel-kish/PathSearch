#include <iostream>
#include "Point.h"
#include <list>
#include <vector>
#include "geometry.h"
#include <cassert>
#include "DCEL.h"


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

int main()
{
	std::vector<Point> poly{{0,0},{4,1},{4,3},{2,5},{-2,3}};
	DCEL d = mk_CCW_poly(poly);

}