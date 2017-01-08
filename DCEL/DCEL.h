#pragma once
#include <list>
#include "Point.h"
#include <vector>
#include "boost\variant.hpp"
#include <memory>
#include <tuple>

struct treeNode;

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
		treeNode* hist; // this face's representation in history graph
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

	DCEL(DCEL&&);
	DCEL(DCEL const&) = delete;
	
	DCEL& operator=(DCEL&&);
	DCEL& operator=(DCEL const& d) = delete;

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

	void join_face(EdgeList::iterator h);
};

bool is_out_face(DCEL::FaceList::iterator f);
DCEL mk_CCW_poly(std::vector<Point> const& poly);
