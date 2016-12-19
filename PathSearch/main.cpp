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
#include "PathFinding.h"

int main()
{
	using namespace std;
	using namespace std::literals;
	using Node = Graph::Node;

	vector<Point> pts{{5,1.5},{5.5,4},{3.5,6.5},{1,5.5},{1,1},
	{3,0.5},{4,4},{3,5},{2,4},{3.5,2},{3,3},{3,1.5},{0,0},{0,0},{2,2}};

	vector<Edge> v{{0,1},{1,2},{2,3},{3,4},{4,5},{5,0}};
	std::random_device rd;
	std::shuffle(begin(v), end(v), std::mt19937{rd()});

	auto rng = find_max_path(v);

	std::vector<Edge> range(rng.first, rng.second);

	auto poly = edgesToPoly(range);

	for (int i : poly) cout << i << ' '; cout << '\n';

	std::vector<Point> polyPts = polygonPts(pts, poly);
	for (Point const& p : polyPts) cout << p << ' '; cout << '\n';
	cout << "orient " << orientation(polyPts) << '\n';
}