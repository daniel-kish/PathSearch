#include "Search.h"

Step::Step(Graph::Node::Ref r, Point const& searchedPt, std::vector<Point> const& pts)
	: node{r}
{
	distToSearched = dist(searchedPt, circumCenter(pts, node->triangle));
}

Step::Step()
{}

AStarSearch::AStarSearch(std::vector<Point> const & v, Graph & g)
	: pts{v}, curStep(g.nodes.begin(), Point{}, v),
	pos{Position::out}, searchedPt{}, state{noPoint}
{	}

void AStarSearch::step()
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

void AStarSearch::setPoint(Point const & searchedPoint)
{
	if (state == inProgress) // still working
		return;
	state = inProgress;
	searchedPt = searchedPoint;
	opened.clear();
	closed.clear();
	opened.push(Step(curStep.node, searchedPoint, pts));
}

void AStarSearch::setStart(Graph::Node::Ref newStart)
{
	closed.clear();
	opened.clear();
	opened.push(Step(newStart, searchedPt, pts));
}

bool AStarSearch::localize(Point const & p)
{
	setPoint(p);
	while (state == AStarSearch::State::inProgress) {
		step();
	}
	if (state == AStarSearch::State::found)
		return true;
	else
		return false;
}

bool AStarSearch::localizeFrom(Point const & p, Graph::Node::Ref from)
{
	setPoint(p);
	setStart(from);
	while (state == AStarSearch::State::inProgress) {
		step();
	}
	if (state == AStarSearch::State::found)
		return true;
	else
		return false;
}

std::tuple<Graph::Node::Ref, Position> AStarSearch::results() const
{
	if (state == found)
		return{curStep.node, pos};
	return {Graph::Node::Ref{}, Position::out};
}
