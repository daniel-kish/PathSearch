#define _USE_MATH_DEFINES
#pragma once
#include <cmath>
#include "DCEL.h"
#include "geometry.h"
#include <tuple>
#include <queue>
#include <boost\math\constants\constants.hpp>


DCEL::EdgeList::iterator clip_ear(DCEL& d, DCEL::EdgeList::iterator h);
DCEL::EdgeList::iterator clip_delaunay_ear(DCEL& d, DCEL::EdgeList::iterator h);
void polygon_delaunay_triangulation(DCEL& d, DCEL::FaceList::iterator in_face);

bool localDelaunay(DCEL& dcel, DCEL::EdgeList::iterator h);
DCEL::EdgeList::iterator flip(DCEL& d, DCEL::EdgeList::iterator h);

// {h,true} if inside
std::tuple<DCEL::EdgeList::iterator, bool> localize(DCEL& d, Point const& p);
void insert_point(DCEL& d, Point const& p);
void insert_point_inside(DCEL& d, DCEL::EdgeList::iterator h, Point const& p);
void insert_point_on_edge(DCEL& d, DCEL::EdgeList::iterator h, Point const& p);

bool check_triangulation(DCEL& d);

//std::tuple<DCEL,treeNode*> initialize_Delaunay_Kirkpatrick(double coord);


class Queue : public std::queue<DCEL::EdgeList::iterator>
{
public:
	bool is_ok() {
		return std::all_of(c.begin(), c.end(), [](auto h) { return h->i > 0; });
	}
};

std::vector<Point> rectHull(Rect r, int x_pts, int y_pts);
std::vector<Point> rectInsides(Rect r, int x_pts, int y_pts);
std::vector<Point> rectInsidesRand(Rect r, int n_pts);
std::vector<Point> circleHull(Circle c, int nsteps);

template <class Fun>
std::vector<Point> polarHull(Fun f, Point const& center, int nsteps)
{
	double pi = boost::math::constants::pi<double>();

	double step = 2.0*pi / nsteps;
	std::vector<Point> pts; pts.reserve(nsteps);

	for (double phi = 0.0; phi < 2.0*pi; phi += step) {
		double rho = f(phi);
		pts.push_back(Point{rho*cos(phi), rho*sin(phi)} + center);
	}

	auto le = std::unique(pts.begin(), pts.end(), [](Point const& p, Point const& q) {
		if (abs(p.x - q.x) < 1.0e-5 && abs(p.y - q.y) < 1.0e-5)
			return true;
		return false;
	});
	pts.erase(le, pts.end());
	if (abs(pts.back().x - pts.front().x) < 1.0e-5 && abs(pts.back().y - pts.front().y) < 1.0e-5)
		pts.pop_back();

	pts.shrink_to_fit();
	return pts;
}