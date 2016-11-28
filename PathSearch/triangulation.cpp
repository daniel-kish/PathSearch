#include "triangulation.h"
#include "circ_iter.h"
#include <sstream>
#include <numeric>

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
		auto inside_t = [t, &pts](Point p) {return insideTriangle(pts, t, p) == 0; };
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
		int res = insideTriangle(pts, fnd->triangle, p);
		if (res == -1)
			continue;
		else if (res == 0) {
			addNewInsidePoint(pts, g, p, fnd);
			break;
		}
		else if (res == 1) {
			addNewEdgePoint(pts, g, p, fnd, Edge{fnd->triangle[0],fnd->triangle[1]});
			break;
		}
		else if (res == 2) {
			addNewEdgePoint(pts, g, p, fnd, Edge{fnd->triangle[1],fnd->triangle[2]});
			break;
		}
		else if (res == 3) {
			addNewEdgePoint(pts, g, p, fnd, Edge{fnd->triangle[0],fnd->triangle[2]});
			break;
		}
	}
}

bool localDelaunay(std::vector<Point> const& pts, Triangle const& t1, Triangle const& t2, Edge ce)
{
	Circle c1 = circumCircle(pts, t1);
	
	//Edge ce = common_edge(t1, t2);
	//if (!valid_edge(ce)) {
	//	std::ostringstream os;
	//	os << t1 << ' ' << t2;
	//	throw NotNeighbors{os.str()};
	//}
		
	Simplex<1> out2 = set_difference(t2, ce);
	double d1 = dist(pts[out2[0]], c1.center);
	if (d1 < c1.rad)
		return false;

	return true;
}