#include "Search.h"

Step::Step(Graph::Node::Ref r, Point const& searchedPt, std::vector<Point> const& pts)
	: node{r}
{
	distToSearched = dist(searchedPt, triangleCenter(pts, node->triangle));
}

Step::Step()
{}

GreedySearch::GreedySearch(std::vector<Point> const & v, Graph & g)
	: pts{v}, curStep(g.nodes.begin(), Point{}, v),
	pos{Position::out}, searchedPt{}, state{noPoint}
{	}

void GreedySearch::step()
{
	if (state == noPoint || state == found)
		return;
	if (opened.empty()) {
		state = noPoint;
		return;
	}
	curStep = opened.top(); opened.pop();
	auto res = closed.insert(curStep.node->triangle);
	if (!res.second)
		return;
	pos = insideTriangle(pts, curStep.node->triangle, searchedPt);
	if (pos == Position::out)
	{
		for (Graph::Node::Ref neib : curStep.node->refs) 
			opened.push(Step(neib, searchedPt, pts));
	}
	else
		state = found;
}

void GreedySearch::setPoint(Point const & searchedPoint)
{
	if (state == inProgress) // still working
		return;
	state = inProgress;
	searchedPt = searchedPoint;
	curStep = Step(curStep.node, searchedPoint, pts);
	opened.clear();
	closed.clear();
	opened.push(curStep);
}

void GreedySearch::setStart(Graph::Node::Ref newStart)
{
	curStep = Step(newStart, searchedPt, pts);
}
