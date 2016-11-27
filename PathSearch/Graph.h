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

struct Node {
	Triangle triangle;
	std::vector<Node*> refs;
};

struct Graph
{
	using NodesList = std::list<Node>;
	NodesList nodes;
	void insert(Triangle t);
	void erase(NodesList::iterator it);
	void flipNodes(NodesList::iterator it1, NodesList::iterator it2);
};

std::ostream& operator<< (std::ostream& os, Graph const& g);