#include "PathFinding.h"
#include "Graph.h"
#include "Tree.h"

std::set<Triangle> growObstacle(int max_lvl, Graph::Node::Ref root)
{
	BFSTree bfs(root);
	while (bfs.lvl < max_lvl)
		bfs.step();

	// close all opened
	for (auto node : bfs.opened)
		bfs.closed.insert(node->triangle);
	bfs.opened.clear();

	// delete all inner
	std::deque<Graph::Node::Ref> q{root};
	auto in_closed = [&bfs](Graph::Node::Ref neib) {
		return bfs.closed.find(neib->triangle) != bfs.closed.end();
	};

	while (!q.empty())
	{
		auto h = q.front(); q.pop_front();
		if (!in_closed(h)) continue;
		bool all_in = std::all_of(begin(h->refs), end(h->refs), in_closed);
		if (all_in) {
			bfs.closed.erase(h->triangle);
			for (auto neib : h->refs) q.push_back(neib);
		}
	}

	return bfs.closed;
}