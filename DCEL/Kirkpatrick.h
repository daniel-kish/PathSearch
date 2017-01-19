#pragma once
#include <tuple>
#include "Point.h"
#include <boost\variant.hpp>
#include "DCEL.h"
#include "geometry.h"

extern long int depth;
extern long int num_vertices;

using triangle_vertices = std::tuple<DCEL::VertexList::iterator,
	DCEL::VertexList::iterator, DCEL::VertexList::iterator>;

Point const& point(triangle_vertices const& tp, int i);

struct treeNode {
	boost::variant<DCEL::FaceList::iterator, triangle_vertices> face_data;
	std::vector<std::shared_ptr<treeNode>> successors;
	int depth;
	treeNode(DCEL::FaceList::iterator f, int i);
	treeNode(treeNode&&) = default;
};

class print_visitor : public boost::static_visitor<>
{
	std::ostream& os;
public:
	print_visitor(std::ostream& os_);
	void operator()(DCEL::FaceList::iterator f) const;
	void operator()(triangle_vertices const& vs) const;
};

std::ostream& operator<< (std::ostream& os, const treeNode* tn);

class point_inside_visitor : public boost::static_visitor<>
{
public:
	Point const& p;
	Position& pos;
	point_inside_visitor(Point const& p_, Position& pos_);

	void operator()(DCEL::FaceList::iterator f) const;
	void operator()(triangle_vertices const& vs) const;
};

struct search_result {
	treeNode* node;
	Position pos;
};

std::vector<search_result> Kirkpatrick_localize(treeNode* root, Point const& point);
DCEL::EdgeList::iterator flip_history(DCEL& d, DCEL::EdgeList::iterator h1);
void insert_point_inside_history(DCEL& d, treeNode* node, Point const& point);
void insert_point(DCEL& d, treeNode* root, Point const& point);