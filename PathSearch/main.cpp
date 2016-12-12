#include <iostream>
#include "Point.h"
#include "Graph.h"
#include "geometry.h"
#include "triangulation.h"
#include <deque>
#include <set>
#include "Search.h"
#include <random>
#include <functional>
#include <chrono>

void print(std::deque<Graph::Node::Ref> const& dq)
{
	for (auto const& e : dq) std::cout << '(' << e->triangle << ',' << &(*e) << ')' << ' ';
	std::cout << '\n';
}




int main()
{
	using namespace std;
	using namespace std::literals;
	using Node = Graph::Node;

	vector<Point> pts = rectHull(Rect({10.0,20.0}),10,10);
	Graph g = triangulatePolygon(pts);

	toDelaunay(pts, g);
}