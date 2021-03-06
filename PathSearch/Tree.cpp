#include "Tree.h"

//bool operator<(TreeNode const& p, TreeNode const& q)
//{
//	return p.node < q.node;
//}

BFSTree::BFSTree(Graph::Node::Ref begin, float p)
	: mt{std::random_device{}()}, opened{begin}, closed{}, head{begin}, prob{p}
{
}

BFSTree::BFSTree(float p)
	: prob{p}
{ }

void BFSTree::step()
{
	std::bernoulli_distribution d(prob);
	if (opened.empty()) return;

	if (d(mt)) {
		head = opened.back();
		opened.pop_back();
	}
	else {
		head = opened.front();
		opened.pop_front();
	}
	auto res = closed.insert(head);
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
