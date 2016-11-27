#pragma once
#include "geometry.h"
#include <vector>
#include <list>

template  <class T>
void eraseVal(std::vector<T>& v, T val)
{
	auto le = std::remove(v.begin(), v.end(), val);
	v.erase(le, v.end());
}

template <class T>
std::ostream& operator<< (std::ostream& os, std::vector<T> const& v)
{
	for (auto const& e : v)
		os << e << ' ';
	return os;
}

struct Graph
{
	struct Node {
		using Ref = std::list<Node>::iterator;
		Triangle triangle;
		std::vector<Ref> refs;
	};
	using NodesList = std::list<Node>;
	using Pair = std::tuple<Graph::Node::Ref, Graph::Node::Ref>;
	NodesList nodes;
	void insert(Triangle t);
	void erase(Node::Ref it);
	Pair Graph::flipNodes(Node::Ref it1, Node::Ref it2, Edge common_edge);
};

std::ostream& operator<< (std::ostream& os, Graph const& g);