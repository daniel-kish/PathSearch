#include <iostream>
#include "Point.h"
#include "geometry.h"
#include <chrono>
#include <list>
#include <vector>
#include <cassert>
#include <algorithm>
#include <numeric>
#include "Graph.h"
#include "geometry.h"
#include "triangulation.h"

int main()
{
	using namespace std;
	using namespace std::literals;
	using namespace std::chrono;

	vector<Point> pts{{12,2},{12,6},{10,6},{8,8},{2,2},{8,4}};
	Graph g = triangulatePolygon(pts);
	cout << g << '\n';

	for (auto p = g.nodes.begin(); p != g.nodes.end(); ++p)
	{
		auto& refs = p->refs;
		for (Node* neib : refs)
			if (!localDelaunay(pts, neib->triangle, p->triangle)) {
				
			}
	}
	
}