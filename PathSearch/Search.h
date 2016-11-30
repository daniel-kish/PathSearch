#pragma once
#include "Graph.h"
#include "geometry.h"
#include <set>
#include <queue>
#include <functional>

struct Step {
	Graph::Node::Ref node;
	double distToSearched;
	Step(Graph::Node::Ref r, Point const& searchedPt, std::vector<Point> const& pts);
	Step();
	bool operator<(Step const& other) const
	{
		return distToSearched < other.distToSearched;
	}
	bool operator>(Step const& other) const
	{
		return distToSearched > other.distToSearched;
	}
};

class Opened 
	: public std::priority_queue<Step, std::vector<Step>, std::greater<Step>>
{
public:
	void clear()
	{
		c.clear();
	}
};

class GreedySearch
{
public:
	std::vector<Point> const& pts;
	Step curStep;
	Position pos;
	Point searchedPt;
	enum State{noPoint, inProgress, found};
	State state;
	Opened opened;
	std::set<Triangle> closed;

	GreedySearch(std::vector<Point> const& v, Graph& ref);
	void step();
	void setPoint(Point const& searchedPoint);
	void setStart(Graph::Node::Ref newStart);
};
