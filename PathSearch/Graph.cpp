#include "Graph.h"
#include <numeric>
#include <sstream>
#include "circ_iter.h"

void Graph::insert(Triangle t)
{
	nodes.push_back({t,{}});
	Node& last = nodes.back();
	auto e = std::prev(nodes.end());
	for (auto p = nodes.begin(); p != e; ++p)
	{
		Edge ce = common_edge(last.triangle, p->triangle);
		if (valid_edge(ce))
		{
			last.refs.push_back(&(*p));
			p->refs.push_back(&last);
		}
	}
}

void Graph::erase(NodesList::iterator it)
{
	Node* erased = &(*it);
	for (auto neib : it->refs)
		eraseVal(neib->refs, erased);
	nodes.erase(it);
}

void Graph::flipNodes(NodesList::iterator it1, NodesList::iterator it2)
{
	Edge e = common_edge(it1->triangle, it2->triangle);
	if (!valid_edge(e)) {
		std::ostringstream msg;
		msg << "flipNodes(): " << it1->triangle << ' ' << it2->triangle << '\n';
		throw FlipError(msg.str());
	}
	std::vector<Node*> neibs = it1->refs;
	std::copy(begin(it2->refs), end(it2->refs), std::back_inserter(neibs));

	// erase t1 and t2 from the 'neibs'
	eraseVal(neibs, &(*it2));
	eraseVal(neibs, &(*it1));

	Triangle t1 = it1->triangle;
	Triangle t2 = it2->triangle;
	this->erase(it1);
	this->erase(it2);

	std::tie(t1, t2) = flip(t1, t2);
	this->nodes.push_back({t1,{}});
	Node& n1 = nodes.back();
	this->nodes.push_back({t2,{}});
	Node& n2 = nodes.back();

	// t1
	for (Node* n : neibs) {
		Edge ce = common_edge(t1, n->triangle);
		if (valid_edge(ce))
		{
			n1.refs.push_back(n);
			n->refs.push_back(&n1);
		}
	}
	// t2
	for (Node* n : neibs) {
		Edge ce = common_edge(t2, n->triangle);
		if (valid_edge(ce))
		{
			n2.refs.push_back(n);
			n->refs.push_back(&n2);
		}
	}
	n1.refs.push_back(&n2);
	n2.refs.push_back(&n1);
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