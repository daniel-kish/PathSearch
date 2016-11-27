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