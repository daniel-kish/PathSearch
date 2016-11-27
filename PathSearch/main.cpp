#include <iostream>
#include "Point.h"
#include "Graph.h"
#include "geometry.h"
#include "triangulation.h"
#include <deque>
#include <unordered_set>

int main()
{
	using namespace std;
	using namespace std::literals;
	using Node = Graph::Node;

	vector<Point> pts{{12,2},{12,6},{10,6},{8,8},{2,2},{8,4}};
	Graph g = triangulatePolygon(pts);
	cout << g << '\n';

	deque<Node::Ref> q{g.nodes.begin()};

	while (!q.empty())
	{
		Node::Ref head = q.front();
		q.pop_front();
		
		for (Node::Ref neib : head->refs) 
		{
			Edge e = common_edge(neib->triangle, head->triangle);
			if (!localDelaunay(pts, head->triangle, neib->triangle,e)) {
				Node::Ref m1, m2;
				std::tie(m1, m2) = g.flipNodes(head, neib, e);
			}
		}
	}
}