#pragma once
#include <vector>
#include "Point.h"
#include "geometry.h"
#include "Graph.h"
#include <set>

struct PathNode {
	Graph::Node::Ref node;
	std::set<PathNode>::iterator par;
	double dist_to_trg;
};

bool operator<(PathNode const& p, PathNode const& q);

struct DelaunayPathFinder
{
	std::vector<Poly> obstacles;
	Rect boundingRect;
	std::vector<Point> pts;
	Graph g;
	std::vector<Point> src_trg;
	std::vector<Graph::Node::Ref> tri_src_trg;
	std::set<Graph::Node::Ref> obstacleNodes;

	int last_hull_pos;

	DelaunayPathFinder(Rect r, std::vector<Poly> const& polys);
	bool set_point(Point const& p);
	void find_path();
	void find_obstacle_nodes();
	std::vector<Graph::Node::Ref> path;
	std::vector<Point> path_points;
	void path_to_points();

	bool in_any_obstacle(Point const& p);
};