#pragma once
#include <deque>
#include <set>
#include "Graph.h"
#include <random>

class BFSTree
{
public:
	std::mt19937 mt;
	std::deque<Graph::Node::Ref> opened;
	std::set<Graph::Node::Ref> closed;
	Graph::Node::Ref head;
	int lvl{0};
	float prob;
	explicit BFSTree(Graph::Node::Ref begin, float p=0.98);
	BFSTree(float p=0.98);
	void step();
	void clear();
	void setStart(Graph::Node::Ref begin);
	float& probability();
};