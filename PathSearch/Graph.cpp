#include "Graph.h"
#include <numeric>
#include <sstream>
#include "circ_iter.h"

void Graph::insert(Triangle t)
{
	nodes.push_back({t,{}});
	const auto last = std::prev(nodes.end());
	for (auto p = nodes.begin(); p != last; ++p)
	{
		Edge ce = common_edge(last->triangle, p->triangle);
		if (valid_edge(ce))
		{
			last->refs.push_back(p);
			p->refs.push_back(last);
		}
	}
}

void Graph::erase(Node::Ref erased)
{
	for (auto neib : erased->refs)
		eraseVal(neib->refs, erased);
	nodes.erase(erased);
}

void Graph::flipNodes(Node::Ref it1, Node::Ref it2, Edge e)
{
	/*Edge e = common_edge(it1->triangle, it2->triangle);
	if (!valid_edge(e)) {
		std::ostringstream msg;
		msg << "flipNodes(): " << it1->triangle << ' ' << it2->triangle;
		throw FlipError(msg.str());
	}*/
	std::vector<Node::Ref> neibs = it1->refs;
	std::copy(begin(it2->refs), end(it2->refs), std::back_inserter(neibs));

	// erase t1 and t2 from the 'neibs'
	eraseVal(neibs, it2);
	eraseVal(neibs, it1);

	for (auto neib : it1->refs)
		eraseVal(neib->refs, it1);
	for (auto neib : it2->refs)
		eraseVal(neib->refs, it2);
	
	it1->refs.clear();
	it2->refs.clear();

	std::tie(it1->triangle, it2->triangle) = flip(it1->triangle, it2->triangle, e);
	
	for (Node::Ref n : neibs) {
		Edge ce = common_edge(it1->triangle, n->triangle);
		if (valid_edge(ce)) {
			it1->refs.push_back(n);
			n->refs.push_back(it1);
		}
	}
	for (Node::Ref n : neibs) {
		Edge ce = common_edge(it2->triangle, n->triangle);
		if (valid_edge(ce)) {
			it2->refs.push_back(n);
			n->refs.push_back(it2);
		}
	}
	it1->refs.push_back(it2);
	it2->refs.push_back(it1);
}

std::ostream& operator<< (std::ostream& os, Graph const& g)
{
	os << "{ ";
	for (auto const& n : g.nodes)
		os << n.triangle << ' ';
	os << "}\n";
	os << "[\n";
	for (auto& node : g.nodes)
	{
		for (auto neib : node.refs)
			os << node.triangle << " -> " << neib->triangle << '\n';
	}
	os << "]\n";
	return os;
}

bool operator<(Graph::Node::Ref p, Graph::Node::Ref q)
{
	return p->triangle < q->triangle;
}
