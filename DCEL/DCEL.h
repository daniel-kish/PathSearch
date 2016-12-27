#pragma once
#include <list>
#include "Point.h"

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

	FaceList::iterator out_face;

	VertexList::iterator newVertex(Point const& p);
	FaceList::iterator newFace();
	EdgeList::iterator newHalfedge();

	DCEL(Point p, Point q);
	void print();

	void add_vertex(Point p, EdgeList::iterator h);

	void split_face(EdgeList::iterator h, VertexList::iterator v);

	void split_edge(EdgeList::iterator h, Point p);
};