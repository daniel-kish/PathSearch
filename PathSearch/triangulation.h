#pragma once
#include "geometry.h"
#include "Graph.h"
#include <vector>

struct NotNeighbors
{
	std::string msg;
	NotNeighbors(std::string s);
	std::string const& what() const;
};

Graph triangulatePolygon(std::vector<Point> const& pts);

void addNewPoint(std::vector<Point>& pts, Graph& g, Point const& p);

bool localDelaunay(std::vector<Point> const& pts, Triangle const& t1, Triangle const& t2, Edge ce);

std::vector<Graph::Node::Ref>
getNextBest(std::vector<Point> const& pts, Graph::Node::Ref tr, Point const& p);

void toDelaunay(std::vector<Point> const& pts, Graph& g);

std::vector<Point> rectHull(Rect r, int x_pts, int y_pts);
std::vector<Point> rectInsides(Rect r, int x_pts, int y_pts);
std::vector<Point> rectInsidesRand(Rect r, int n_pts);