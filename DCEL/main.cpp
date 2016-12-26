#include <iostream>
#include "Point.h"
#include <list>
#include <vector>

struct DCEL {
	struct Vertex;
	struct Face;
	struct Halfedge;
	using VertexList = std::list<Vertex>;
	using FaceList = std::list<Face>;
	using EdgeList = std::list<Halfedge>;

	struct Vertex {
		Point p;
		EdgeList::iterator halfedge;
	};
	struct Face {
		int i;
		EdgeList::iterator halfedge;
	};
	struct Halfedge {
		int i;
		EdgeList::iterator twin;
		EdgeList::iterator next;
		EdgeList::iterator prev;
		VertexList::iterator target;
		FaceList::iterator face;
	};

	VertexList vertices;
	FaceList faces;
	EdgeList halfedges;

	VertexList::iterator newVertex(Point const& p)
	{
		vertices.push_front({p});
		return vertices.begin();
	}
	FaceList::iterator newFace()
	{
		int i = 0;
		if (!faces.empty()) i = faces.begin()->i+1;
		faces.push_front({i});
		return faces.begin();
	}
	EdgeList::iterator newHalfedge()
	{
		int i = 0;
		if (!halfedges.empty()) i = halfedges.begin()->i + 1;
		halfedges.push_front({i});
		return halfedges.begin();
	}

	DCEL(Point p, Point q)
	{
		if (q < p) std::swap(p, q);

		auto f0 = newFace();
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
	void print()
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
	
	void add_vertex(Point p, EdgeList::iterator h)
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

	void split_face(EdgeList::iterator h, VertexList::iterator v)
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
};

int main()
{
	DCEL d({0,0}, {1,1});
	
	d.add_vertex({2,0}, d.halfedges.begin());
	//d.print();
	
	auto last = std::prev(d.halfedges.end());

	d.split_face(last, d.vertices.begin());

	d.print();
}