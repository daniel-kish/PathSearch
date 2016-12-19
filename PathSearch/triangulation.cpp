#include "triangulation.h"
#include "circ_iter.h"
#include <sstream>
#include <numeric>
#include <deque>
#include <set>
#include <random>

NotNeighbors::NotNeighbors(std::string s) : msg(std::move(s))
{}
std::string const& NotNeighbors::what() const {
	return msg;
}

Graph triangulatePolygon(std::vector<Point> const& pts)
{
	using std::begin; using std::end;
	using std::next; using std::prev;

	Graph g;
	if (pts.size() < 3)
		return g;

	std::vector<int> refs(pts.size());
	std::iota(begin(refs), end(refs), 0);

	for (circular_iterator<int> p(refs); refs.size() > 3; ++p)
	{
		Triangle t{*p,*(p + 1),*(p + 2)};
		Side s = side(pts[t[0]], pts[t[2]], pts[t[1]]);
		if (s != Side::right) continue;
		auto inside_t = [t, &pts](Point p) {return insideTriangle(pts, t, p) == Position::in; };
		bool clean = std::none_of(begin(pts), end(pts), inside_t);
		if (!clean) continue;
		std::sort(t.begin(), t.end());
		g.insert(t);
		p.get() = refs.erase((p + 1).get());
	}
	Triangle last{*(refs.begin()),*(next(refs.begin())),*(next(refs.begin(),2))};
	g.insert(last);
	return g;
}

void addNewInsidePoint(std::vector<Point>& pts, Graph& g, Point const& p, Graph::Node::Ref fnd)
{
	using Node = Graph::Node;

	Triangle t = fnd->triangle;
	std::vector<Node::Ref> neibs = fnd->refs;
	g.erase(fnd);

	std::sort(t.begin(), t.end());
	Simplex<1> s{int(pts.size() - 1)};
	std::array<Edge, 3> edges{
		{{t[0],t[1]},{t[1],t[2]},{t[0],t[2]}}
	};

	// make three new triangles
	std::array<Node, 3> nodes;
	for (unsigned i{0u}; i < 3; ++i)
		nodes[i].triangle = set_union(edges[i], s);

	for (Node const& n : nodes)
		g.nodes.push_back(n);

	const auto b = std::prev(g.nodes.end(), 3);
	const auto e = g.nodes.end();
	for (auto p = b; p != e; ++p) {
		// pick a neighbor from 'neibs'
		for (Node::Ref neib : neibs) {
			Edge ce = common_edge(p->triangle, neib->triangle);
			if (valid_edge(ce)) {
				p->refs.push_back(neib);
				neib->refs.push_back(p);
			}
		}
		// and among themselves
		for (auto q = b; q != e; ++q) {
			if (&(*p) == &(*q))  continue;
			Edge ce = common_edge(q->triangle, p->triangle);
			if (valid_edge(ce)) {
				p->refs.push_back(q);
			}
		}
	}
}

void addNewEdgePoint(std::vector<Point>& pts, Graph& g, Point const& p, Graph::Node::Ref p1, Edge e)
{
	using Node = Graph::Node;

	// find twin 
	auto pos = std::find_if(p1->refs.begin(), p1->refs.end(), [p1,e](Node::Ref neib) {
		Edge ce = common_edge(p1->triangle, neib->triangle);
		return ce == e;
	});
	Node::Ref p2; // twin
	bool has_twin = (pos != p1->refs.end());
	if (has_twin)
		p2 = *pos;

	std::vector<Node::Ref> neibs = p1->refs;
	if (has_twin) {
		std::copy(p2->refs.begin(), p2->refs.end(), std::back_inserter(neibs));
	}
	eraseVal(neibs, p1);
	if (has_twin) eraseVal(neibs, p2);

	// disconnect p1
	for (auto neib : p1->refs)
		eraseVal(neib->refs, p1);
	p1->refs.clear();

	if (has_twin) {
		// disconnect p2
		for (auto neib : p2->refs)
			eraseVal(neib->refs, p2);
		p2->refs.clear();
	}

	Simplex<1> newP{int(pts.size() - 1)};
	Simplex<1> out1 = set_difference(p1->triangle, e);
	p1->triangle = {out1[0], e[0], newP[0]};
	g.nodes.push_back(Node{}); Node::Ref p12 = std::prev(g.nodes.end());
	p12->triangle = {out1[0], e[1], newP[0]};

	p1->refs.push_back(p12);
	p12->refs.push_back(p1);

	//std::cout << p1->triangle << ' ';
	//std::cout << p12->triangle << ' ';
	
	Node::Ref p22;
	if (has_twin) {
		Simplex<1> out2 = set_difference(p2->triangle, e);
		p2->triangle = {out2[0], e[0], newP[0]};
		g.nodes.push_back(Node{}); p22 = std::prev(g.nodes.end());
		p22->triangle = {out2[0], e[1], newP[0]};
		p2->refs.push_back(p22);
		p22->refs.push_back(p2);

		//std::cout << p2->triangle << ' ';
		//std::cout << p22->triangle << ' ';
	}
	auto make_friends = [](std::vector<Node::Ref> neibs, Node::Ref p) {
		for (Node::Ref neib : neibs) {
			Edge e = common_edge(neib->triangle, p->triangle);
			if (valid_edge(e)) {
				neib->refs.push_back(p);
				p->refs.push_back(neib);
			}
		}
	};
	make_friends(neibs, p1);
	make_friends(neibs, p12);
	if (has_twin) {
		make_friends(neibs, p2);
		make_friends(neibs, p22);
		std::vector<Node::Ref> we{p2,p22};
		make_friends(we, p1);
		make_friends(we, p12);
	}	
	std::sort(p1->triangle.begin(), p1->triangle.end());
	std::sort(p12->triangle.begin(), p12->triangle.end());
	if (has_twin) {
		std::sort(p2->triangle.begin(), p2->triangle.end());
		std::sort(p22->triangle.begin(), p22->triangle.end());
	}
}

void addNewPoint(std::vector<Point>& pts, Graph& g, Point const& p)
{
	using Node = Graph::Node;

	pts.push_back(p);
	for (auto fnd = g.nodes.begin(); fnd != g.nodes.end(); ++fnd)
	{
		Position res = insideTriangle(pts, fnd->triangle, p);
		if (res == Position::out)
			continue;
		else if (res == Position::in) {
			addNewInsidePoint(pts, g, p, fnd);
			break;
		}
		else if (res == Position::edge01) {
			addNewEdgePoint(pts, g, p, fnd, Edge{fnd->triangle[0],fnd->triangle[1]});
			break;
		}
		else if (res == Position::edge12) {
			addNewEdgePoint(pts, g, p, fnd, Edge{fnd->triangle[1],fnd->triangle[2]});
			break;
		}
		else if (res == Position::edge02) {
			addNewEdgePoint(pts, g, p, fnd, Edge{fnd->triangle[0],fnd->triangle[2]});
			break;
		}
	}
}

bool localDelaunay(std::vector<Point> const& pts, Triangle const& t1, Triangle const& t2, Edge ce)
{
	Circle c1 = circumCircle(pts, t1);
		
	Simplex<1> out2 = set_difference(t2, ce);
	double d1 = dist(pts[out2[0]], c1.center);
	if (d1 < c1.rad)
		return false;

	return true;
}

std::vector<Graph::Node::Ref>
getNextBest(std::vector<Point> const& pts, Graph::Node::Ref tr, Point const& p)
{
	TriPos pos; Point clos;
	Point const& a = pts[tr->triangle[0]];
	Point const& b = pts[tr->triangle[1]];
	Point const& c = pts[tr->triangle[2]];
	std::tie(clos, pos) = ClosestPointOnTriangle(p, a, b, c);
	std::vector<Graph::Node::Ref> next;
	if (pos == TriPos::inside) {
		//std::cout << "found " << tr->triangle << '\n';
		return{tr};
	}
	else
	{
		std::vector<Edge> guiltyEdges;
		switch (pos)
		{
		case TriPos::edge01:
			guiltyEdges.push_back(Edge{tr->triangle[0],tr->triangle[1]});
			break;
		case TriPos::edge12:
			guiltyEdges.push_back(Edge{tr->triangle[1],tr->triangle[2]});
			break;
		case TriPos::edge20:
			guiltyEdges.push_back(Edge{tr->triangle[0],tr->triangle[2]});
			break;
		case TriPos::V0:
			guiltyEdges.push_back(Edge{tr->triangle[0],tr->triangle[1]});
			guiltyEdges.push_back(Edge{tr->triangle[0],tr->triangle[2]});
			break;
		case TriPos::V1:
			guiltyEdges.push_back(Edge{tr->triangle[0],tr->triangle[1]});
			guiltyEdges.push_back(Edge{tr->triangle[1],tr->triangle[2]});
			break;
		case TriPos::V2:
			guiltyEdges.push_back(Edge{tr->triangle[0],tr->triangle[2]});
			guiltyEdges.push_back(Edge{tr->triangle[1],tr->triangle[2]});
			break;
		}
		for (auto neib : tr->refs)
		{
			Edge ce = common_edge(tr->triangle, neib->triangle);
			bool yeah = std::any_of(begin(guiltyEdges), end(guiltyEdges), [ce](Edge const& e) {
				return e == ce;
			});
			if (yeah)
				next.push_back(neib);
		}
		return next;
	}
}


void toDelaunay(std::vector<Point> const& pts, Graph& g)
{
	using Node = Graph::Node;
	std::deque<Graph::Node::Ref> q;
	std::set<Triangle> closed;
	q.push_back(g.nodes.begin());

	while (!q.empty())
	{
		Node::Ref head = q.front();
		q.pop_front();
		auto res = closed.insert(head->triangle);
		if (!res.second)
			continue;

		std::vector<Node::Ref> const& neibs = head->refs;
		for (Node::Ref neib : neibs)
		{
			const Triangle &t1 = head->triangle, &t2 = neib->triangle;
			Edge e = common_edge(t1, t2);
			if (!localDelaunay(pts, t1, t2, e)) {
				g.flipNodes(head, neib, e);
				q.push_back(head); q.push_back(neib);
				break;
			}
			//if (closed.find(neib->triangle) == closed.end())
				q.push_back(neib);
		}
	}
}

std::vector<Point> rectHull(Rect r, int x_pts, int y_pts)
{
	double wid = std::abs(r.dir.x);
	double height = std::abs(r.dir.y);
	double xStep = wid / x_pts;
	double yStep = height / y_pts;

	std::vector<Point> pts; pts.reserve(2 * x_pts + 2 * x_pts);
	for (double x = 0.0; x < wid; x += xStep)
		pts.push_back(Point{x,0.0});
	for (double y = 0.0; y < height; y += yStep)
		pts.push_back(Point{wid,y});
	for (double x = wid; x > 0.0; x -= xStep)
		pts.push_back(Point{x,height});
	for (double y = height; y > 0.0; y -= yStep)
		pts.push_back(Point{0.0,y});
	
	pts.shrink_to_fit();

	if (r.origin != Point{0.0,0.0})
		for (Point& p : pts)
			p = p + r.origin;

	return pts;
}

std::vector<Point> rectInsides(Rect r, int x_pts, int y_pts)
{
	std::vector<Point> pts;
	double xb = r.origin.x, xe = r.origin.x + r.dir.x;
	double yb = r.origin.y, ye = r.origin.y + r.dir.y;
	double x_step = (xe - xb) / x_pts;
	double y_step = (ye - yb) / y_pts;

	for (double x = xb + x_step; x < xe; x += x_step)
		for (double y = yb + y_step; y < ye; y += y_step)
			pts.push_back({x,y});
	return pts;
}

std::vector<Point> rectInsidesRand(Rect r, int n_pts)
{
	std::vector<Point> pts; pts.reserve(n_pts);
	double xb = r.origin.x, xe = r.origin.x + r.dir.x;
	double yb = r.origin.y, ye = r.origin.y + r.dir.y;
	std::mt19937 mt{};
	std::uniform_real_distribution<double> xd(xb, xe);
	std::uniform_real_distribution<double> yd(yb, ye);

	while (n_pts--)
		pts.push_back({xd(mt),yd(mt)});
	return pts;
}