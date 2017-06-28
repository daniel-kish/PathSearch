#include "Obstacles.h"
#include "Graph.h"
#include "Tree.h"
#include "triangulation.h"
#include <map>
#include <fstream>


ObstacleGrower::ObstacleGrower(Rect r_, int nx_, int ny_, float inner_to_hull_pts_ratio_)
	: r{r_}, nx{nx_}, ny{ny_}, inner_to_hull_pts_ratio{inner_to_hull_pts_ratio_},
	probability_balance{0.9f}, mt{/*std::random_device{}()*/}
{
	pts = rectHull(r, nx, ny);
	g = triangulatePolygon(pts);

	std::vector<Point> ins;
	//ins = rectInsidesRand(r, nx * ny * inner_to_hull_pts_ratio);
	ins = rectInsides(r, nx, ny);
	for (Point const& p : ins)
		addNewPoint(pts, g, p);

	toDelaunay(pts, g);
	max_tree_lvl = std::sqrt(g.nodes.size());
}

std::vector<Point> const & ObstacleGrower::points() const
{
	return pts;
}

Graph const & ObstacleGrower::graph() const
{
	return g;
}

void ObstacleGrower::growObstacle()
{
	auto root = g.nodes.begin();
	std::uniform_int_distribution<int> ud(0, g.nodes.size() - 1);
	int idx = ud(mt);
	std::advance(root, idx);
	
	for (auto& set : sets)
		if (set.find(root) != set.end())
			return;

	std::bernoulli_distribution d(probability_balance);
	BFSTree bfs(root);
	while (bfs.lvl < max_tree_lvl)
	{
		if (bfs.opened.empty()) break;

		if (d(bfs.mt)) {
			bfs.head = bfs.opened.back();
			bfs.opened.pop_back();
		}
		else {
			bfs.head = bfs.opened.front();
			bfs.opened.pop_front();
		}
		auto fnd = bfs.closed.find(bfs.head);
		if (fnd != bfs.closed.end()) continue;

		// check with other sets' boundaries
		bool interferes{false};
		for (auto const& boundary : boundaries)
		{
			for (auto node : boundary)
				if (have_common_vertices(bfs.head->triangle, node->triangle))
				{
					interferes = true;
					break;
				}
			if (interferes) break;
		}
		if (interferes) continue;

		auto res = bfs.closed.insert(bfs.head);

		for (Graph::Node::Ref neib : bfs.head->refs)
			bfs.opened.push_back(neib);
		bfs.lvl++;
	}
	if (bfs.closed.empty()) return;

	std::vector<Edge> edge_list = boundaryEdges(bfs.closed);
	if (!check_poly(edge_list)) return;

	edge_lists.push_back(std::move(edge_list));
	boundaries.push_back(boundary(bfs.closed));
	sets.push_back(std::move(bfs.closed));
}

std::vector<Point> ObstacleGrower::getPoly(int i)
{
	auto range = find_max_path(edge_lists[i]);
	std::vector<Edge> connected(range.first, range.second);
	std::vector<int> poly = edgesToPoly(connected);
	std::vector<Point> polyPts = polygonPts(pts, poly);
	return polyPts;
}

void ObstacleGrower::clear()
{
	sets.clear();
	boundaries.clear();
	edge_lists.clear();
}

int ObstacleGrower::num_obstacles()
{
	return sets.size();
}

float ObstacleGrower::density()
{
	std::size_t s = 0;
	for (auto const& set : sets)
		s += set.size();
	return float(s) / g.nodes.size();
}

NodeSet boundary(NodeSet& obs)
{
	std::deque<Graph::Node::Ref> q{*obs.begin()};
	std::set<Graph::Node::Ref> closed;
	auto in_obs = [&obs](Graph::Node::Ref neib) {
		return obs.find(neib) != obs.end();
	};
	std::set<Graph::Node::Ref> bnd;

	while (!q.empty())
	{
		auto h = q.front(); q.pop_front();
		auto res = closed.insert(h);
		if (!res.second) continue;

		for (auto neib : h->refs) {
			if (in_obs(neib))
				q.push_back(neib);
			else
				bnd.insert(h);
		}
		if (h->refs.size() < 3) bnd.insert(h);

	}
	return bnd;
}

std::vector<Edge> boundaryEdges(NodeSet& obs)
{
	std::deque<Graph::Node::Ref> q{*obs.begin()};
	std::set<Graph::Node::Ref> closed;
	auto in_obs = [&obs](Graph::Node::Ref neib) {
		return obs.find(neib) != obs.end();
	};
	std::vector<Edge> edges;

	while (!q.empty())
	{
		auto h = q.front(); q.pop_front();
		auto res = closed.insert(h);
		if (!res.second) continue;

		for (auto neib : h->refs) {
			if (in_obs(neib))
				q.push_back(neib);
			else
				edges.push_back(common_edge(h->triangle, neib->triangle));
		}
		if (h->refs.size() < 3) 
		{
			std::vector<Edge> myedges = {{h->triangle[0],h->triangle[1]},
			{h->triangle[1],h->triangle[2]},{h->triangle[0],h->triangle[2]}};
			for (auto neib : h->refs)
			{
				Edge ce = common_edge(h->triangle, neib->triangle);
				auto pe = std::find(myedges.begin(), myedges.end(), ce);
				myedges.erase(pe);
			}
			edges.insert(edges.end(), myedges.begin(), myedges.end());
		}
	}
	return edges;
}

bool check_poly(std::vector<Edge>& edges)
{
	std::map<int, int> count;
	
	for (Edge const& e : edges)
	{
		count[e[0]]++;
		if (count[e[0]] > 2) return false;
		count[e[1]]++;
		if (count[e[1]] > 2) return false;
	}
	return true;
}

iter<Edge> connect_first(iter<Edge> p, iter<Edge> e)
{
	if (p == e) return p;
	auto have_common_with_p = [&p](Edge const& e) {
		return have_common_vertices(e, *p);
	};
	while (true)
	{
		auto q = find_if(p + 1, e, have_common_with_p);
		if (q == e) break;
		std::iter_swap(p + 1, q);
		++p;
	}
	return p + 1;
}

std::pair<iter<Edge>, iter<Edge>> find_max_path(std::vector<Edge>& v)
{
	std::pair<iter<Edge>, iter<Edge>> max_range;
	std::size_t max_dist = std::size_t(0);

	auto prev_p = v.begin();
	auto p = v.begin();
	while (true)
	{
		prev_p = p;
		p = connect_first(p, end(v));

		auto d = std::distance(prev_p,p);
		if (d > max_dist) {
			max_dist = d;
			max_range = std::make_pair(prev_p, p);
		}
		if (p == end(v)) break;
	}
	return max_range;
}

std::vector<int> edgesToPoly(std::vector<Edge>& v)
{
	for (Edge& e : v) std::sort(e.begin(), e.end());

	std::vector<int> poly;
	
	Simplex<1> common01;
	auto p = v.begin() + 1;
	std::set_intersection(p->begin(), p->end(), (p - 1)->begin(), (p - 1)->end(), common01.begin());
	
	poly.push_back((v[0][0] == common01[0]) ? v[0][1] : v[0][0]);

	for (; p != v.end(); ++p)
	{
		Simplex<1> s;
		std::set_intersection(p->begin(), p->end(), (p - 1)->begin(), (p - 1)->end(), s.begin());
		poly.push_back(s[0]);
	}
	return poly;
}

std::vector<Poly> readPoly(std::ifstream& input)
{
	std::vector<std::vector<Point>> polys;

	while (!input.eof())
	{
		Poly poly;
		double x, y;
		int sz{0};
		input >> sz;
		poly.reserve(sz);

		while (sz--) {
			input >> x >> y;
			poly.push_back({x,y});
		}
		if (!poly.empty()) polys.push_back(poly);
	}
	return polys;
}

void writePoly(std::ofstream& os, std::vector<Poly>const& polys)
{
	for (Poly const& poly : polys)
	{
		os << poly.size() << '\n';
		for (Point const& p : poly)
			os << p.x << ' ' << p.y << '\n';
		os << '\n';
	}
}