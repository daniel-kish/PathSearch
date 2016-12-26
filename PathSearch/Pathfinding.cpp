#include "Pathfinding.h"
#include "triangulation.h"
#include <deque>
#include <queue>

DelaunayPathFinder::DelaunayPathFinder(Rect r, std::vector<Poly> const& polys)
	: boundingRect{r}, obstacles{polys}
{
	pts = rectHull(r, 10, 10);
	g = triangulatePolygon(pts);

	last_hull_pos = pts.size()-1;

	for (Poly& poly : obstacles)
	{
		for (Point& p : poly)
			addNewPoint(pts, g, p);
	}
	toDelaunay(pts, g);

	src_trg.reserve(2);
	tri_src_trg.reserve(2);

	find_obstacle_nodes();
}

bool DelaunayPathFinder::set_point(Point const& p)
{
	if (src_trg.size() == 2) {
		src_trg.clear();
		tri_src_trg.clear();
	}
	auto tri = std::find_if(g.nodes.begin(), g.nodes.end(), [this, &p](Graph::Node const& node) {
		return Position::in == insideTriangle(pts, node.triangle, p);
	});
	if (tri == g.nodes.end())
		return false;

	src_trg.push_back(p);
	tri_src_trg.push_back(tri);

	return true;
}

void DelaunayPathFinder::find_path()
{
	path.clear();
	path_points.clear();
	if (src_trg.size() < 2) return;

	if (obstacleNodes.find(tri_src_trg[0]) != obstacleNodes.end())
		return;
	if (obstacleNodes.find(tri_src_trg[1]) != obstacleNodes.end())
		return;

	std::set<PathNode> c;
	std::deque<PathNode> q;
	q.push_back({tri_src_trg[0], c.end()});

	bool found{false};
	std::set<PathNode>::iterator last;
	while (!q.empty())
	{
		auto h = q.front(); q.pop_front();

		auto r = c.insert(h);
		if (!r.second) continue;

		if (h.node->triangle == tri_src_trg[1]->triangle) {
			last = r.first;
			found = true;
			break;
		}

		for (auto neib : h.node->refs)
		{
			if (obstacleNodes.find(neib) != obstacleNodes.end()) continue;
			q.push_back({neib,r.first});
		}
	}
	path.clear();
	if (!found) return;
	// build path
	auto iter = last;
	while (iter != c.end()) {
		path.push_back(iter->node);
		iter = iter->par;
	}
	path_to_points();
}

void DelaunayPathFinder::find_obstacle_nodes()
{
	for (auto nodeIter = g.nodes.begin(); nodeIter != g.nodes.end(); nodeIter++)
	{
		bool inside_point_test = in_any_obstacle(triangleCenter(pts, nodeIter->triangle));
		if (inside_point_test) 
			obstacleNodes.insert(nodeIter);
	}
}

void DelaunayPathFinder::path_to_points()
{
	path_points.clear();
	path_points.push_back(src_trg[1]);
	
	int idx = 1;
	while (idx < path.size()-1)
	{
		Line general_line{path_points.back(), src_trg[0]};
		auto node = path[idx];
		
		Edge ce = common_edge(node->triangle, path[idx + 1]->triangle);
		Point origin = pts[ce[0]];
		Point dir = pts[ce[1]] - origin;

		double line_par, edge_par;
		std::tie(line_par, edge_par) = intersection(origin, dir, general_line);
		if (line_par >= 0.0 && line_par <= 1.0 && edge_par >= -0.2 && edge_par <= 1.2)
		{
			if (edge_par <= 0) edge_par = 0.01;
			else if (edge_par >= 1) edge_par = 0.99;
			Point inter = origin + dir*edge_par;
			path_points.push_back(inter);
		}
		else {
			Point clos; TriPos pos;
			std::tie(clos, pos) = ClosestPointOnTriangle(src_trg[0], 
				pts[node->triangle[0]], pts[node->triangle[1]], pts[node->triangle[2]]);
			auto cen = triangleCenter(pts, node->triangle);
			auto vec = cen - clos;
			vec = vec * (1.0 / norm(vec));
			clos = clos + vec*2.0;

			Line line{path_points.back(),clos};
			Edge ceprev = common_edge(node->triangle, path[idx - 1]->triangle);
			Point origin = pts[ceprev[0]];
			Point dir = pts[ceprev[1]] - origin;

			double line_par, edge_par;
			std::tie(line_par, edge_par) = intersection(origin, dir, line);
			if (edge_par <= 0)
				path_points.push_back(origin + dir*0.01);
			else if (edge_par >= 1.0)
				path_points.push_back(origin + dir*0.99);
			else
				path_points.push_back(clos);
		}
		idx++;
	}
	path_points.push_back(src_trg[0]);
}

bool DelaunayPathFinder::in_any_obstacle(Point const & p)
{
	for (Poly& poly : obstacles)
		if (insidePoly(poly, p)) return true;
	return false;
}

bool operator<(PathNode const & p, PathNode const & q)
{
	return p.node->triangle < q.node->triangle;
}
