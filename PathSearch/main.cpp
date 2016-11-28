#include <iostream>
#include "Point.h"
#include "Graph.h"
#include "geometry.h"
#include "triangulation.h"
#include <deque>
#include <set>

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

	vector<Point> pts{{12,2},{12,6},{10,6},{8,8},{2,2},{8,4}};
	Graph g = triangulatePolygon(pts);
	addNewPoint(pts, g, {5,4});
	
	cout << g << '\n';


	//deque<Node::Ref> q{g.nodes.begin()};
	//set<Triangle> closed;
	//while (!q.empty())
	//{
	//	print(q);  
	//	system("pause >nul");
	//	Node::Ref head = q.front();
	//	q.pop_front();
	//	auto res = closed.insert(head->triangle);
	//	if (!res.second)
	//		continue;
	//	
	//	std::vector<Node::Ref> neibs = head->refs;
	//	for (Node::Ref neib : neibs)
	//	{
	//		Edge e = common_edge(neib->triangle, head->triangle);
	//		if (!localDelaunay(pts, head->triangle, neib->triangle,e)) {
	//			g.flipNodes(head, neib, e);
	//			q.push_back(head); q.push_back(neib);
	//			break;
	//		}
	//		if (closed.find(neib->triangle) == closed.end())
	//			q.push_back(neib);
	//	}
	//}
	//cout << g << '\n';
}