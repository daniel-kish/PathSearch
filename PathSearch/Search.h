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
	container_type::iterator begin()
	{
		return c.begin();
	}
	container_type::iterator end()
	{
		return c.end();
	}
	void make_heap()
	{
		std::make_heap(c.begin(), c.end(),comp);
	}
};

struct AStarSearch
{
	std::vector<Point> const& pts;
	Step curStep;
	Position pos;
	Point searchedPt;
	enum State{noPoint, inProgress, found};
	State state;
	Opened opened;
	std::set<Triangle> closed;
	void step();
	void setPoint(Point const& searchedPoint);
	void setStart(Graph::Node::Ref newStart);
public:
	AStarSearch(std::vector<Point> const& v, Graph& ref);
	bool localize(Point const& p);
	bool localizeFrom(Point const& p, Graph::Node::Ref r);
	std::tuple<Graph::Node::Ref, Position> results() const;
};
