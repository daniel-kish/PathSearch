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

void print(std::deque<Graph::Node::Ref> const& dq)
{
	for (auto const& e : dq) std::cout << '(' << e->triangle << ',' << &(*e) << ')' << ' ';
	std::cout << '\n';
}

auto rectHull(double wid, double height, int wpts, int hpts)
{
	double xStep = wid / wpts;
	double yStep = height / hpts;
	std::vector<Point> pts;
	for (double x = 0.0; x < wid; x += xStep)
		pts.push_back(Point{x,0.0});
	for (double y = 0.0; y < height; y += yStep)
		pts.push_back(Point{wid,y});
	for (double x = wid; x > 0.0; x -= xStep)
		pts.push_back(Point{x,height});
	for (double y = height; y > 0.0; y -= yStep)
		pts.push_back(Point{0.0,y});
	return pts;
}


int main()
{
	using namespace std;
	using namespace std::literals;
	using Node = Graph::Node;

	vector<Point> pts = rectHull(50.0, 30.0, 4, 2);
	for (auto p : pts) cout << p << '\n';
	Graph g = triangulatePolygon(pts);
	cout << g << '\n';

	deque<Graph::Node::Ref> q;
	set<Triangle> closed;
	q.push_back(g.nodes.begin());
	while (!q.empty())
	{
		Node::Ref head = q.front();
		q.pop_front();
		auto res = closed.insert(head->triangle);
		if (!res.second)
			continue;

		std::vector<Node::Ref> neibs = head->refs;
		for (Node::Ref neib : neibs)
		{
			Triangle t1 = head->triangle, t2 = neib->triangle;
			Edge e = common_edge(t1, t2);
			if (!localDelaunay(pts, t1, t2, e)) {
				g.flipNodes(head, neib, e);
				q.push_back(head); q.push_back(neib);
				break;
			}
			if (closed.find(neib->triangle) == closed.end())
				q.push_back(neib);
		}
	}
	cout << g << '\n';

	GreedySearch s(pts, g);
	Line l{{31.25,0},{31.25,30}};
	//mt19937 mt;
	//std::uniform_real_distribution<double> xdist(0.0, 50.0);
	//std::uniform_real_distribution<double> ydist(0.0, 30.0);
	//int samples = 10;
	//
	double t = 1.0;
	while (t > 0) {
		s.setPoint(l(t));
		cout << s.searchedPt << ' ';
		while (s.state == GreedySearch::State::inProgress) {
			s.step();
		}
		if (s.state == GreedySearch::State::found)
			cout << "found: " << s.curStep.node->triangle << ' ' << s.pos << '\n';
		else
			cout << "not found\n";
		t -= 0.1;
	}
}