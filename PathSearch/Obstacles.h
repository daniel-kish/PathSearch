#pragma once
#include "Graph.h"
#include "Tree.h"

using NodeSet = std::set<Graph::Node::Ref>;
using PolyEdges = std::vector<Edge>;
class ObstacleGrower
{
	Rect r;
	int nx, ny;
	float inner_to_hull_pts_ratio;
	std::vector<Point> pts;
	Graph g;
	std::mt19937 mt;
public:
	ObstacleGrower(Rect r, int,int, float inner_to_hull_pts_ratio);
	int max_tree_lvl;
	float probability_balance;

	std::vector<NodeSet> sets;
	std::vector<NodeSet> boundaries;
	std::vector<PolyEdges> edge_lists;

	std::vector<Point> const& points() const;
	Graph const& graph() const;

	void growObstacle();
	std::vector<Point> getPoly(int i);
	void clear();
};

NodeSet boundary(NodeSet& obs);
std::vector<Edge> boundaryEdges(NodeSet& obs);
bool check_poly(std::vector<Edge>& edges);

template <class T>
using iter = typename std::vector<T>::iterator;
iter<Edge> connect_first(iter<Edge> p, iter<Edge> e);
std::pair<iter<Edge>, iter<Edge>> find_max_path(std::vector<Edge>& v);
std::vector<int> edgesToPoly(std::vector<Edge>& v);