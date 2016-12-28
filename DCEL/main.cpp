#include <iostream>
#include "Point.h"
#include <list>
#include <vector>
#include "geometry.h"
#include <cassert>
#include "DCEL.h"
#include "Delaunay.h"
#include <set>


int main()
{
	std::vector<Point> poly = rectHull(Rect({500,300}, {-250,-150}), 20, 10);
	DCEL d = mk_CCW_poly(poly);
	auto poly_face = std::next(d.faces.begin());
	d.out_face = d.faces.begin();
	
	auto h = poly_face->halfedge;
	while (true)
	{
		auto rh = clip_ear(d, h);
		if (rh == h) break; // done
		h = rh; // wrong ear or ok
	}

	//toDelaunay(d);


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
			if (localDelaunay(d, h))
			{
				more = true;
				break;
			}
		}
		if (!more) break;
		std::cout << "here\n";
	}
	std::cout << "done\n";
	//d.print();
}