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
		auto inside_t = [t, &pts](Point p) {return insideTriangle(pts, t, p); };
		bool clean = std::none_of(begin(pts), end(pts), inside_t);
		if (!clean) continue;
		g.insert(t);
		p.get() = refs.erase((p + 1).get());
	}
	Triangle last{*(refs.begin()),*(next(refs.begin())),*(next(refs.begin(),2))};
	g.insert(last);
	return g;
}

void addNewPoint(std::vector<Point>& pts, Graph& g, Point const& p)
{
	using Node = Graph::Node;

	pts.push_back(p);
	auto fnd = std::find_if(begin(g.nodes), end(g.nodes), [&pts, p](const Node& n) {
		return insideTriangle(pts, n.triangle, p);
	});
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