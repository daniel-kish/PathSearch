#include "Tree.h"

BFSTree::BFSTree(Graph::Node::Ref begin, float p)
	: opened{begin}, closed{}, head{begin}, prob{p}
{
}

BFSTree::BFSTree(float p)
	: prob{p}
{ }

void BFSTree::step()
{
	std::bernoulli_distribution d(0.98);
	if (opened.empty() /*|| lvl > 70*/) return;

	if (d(mt)) {
		head = opened.back();
		opened.pop_back();
	}
	else {
		head = opened.front();
		opened.pop_front();
	}
	auto res = closed.insert(head->triangle);
	if (!res.second) return;

	for (Graph::Node::Ref neib : head->refs)
		opened.push_back(neib);
	lvl++;
}

void BFSTree::clear()
{
	opened.clear();
	closed.clear();
	lvl = 0;
}

void BFSTree::setStart(Graph::Node::Ref begin)
{
	clear();
	opened.push_back(begin);
	head = opened.front();
}

float & BFSTree::probability()
{
	return prob;
}
