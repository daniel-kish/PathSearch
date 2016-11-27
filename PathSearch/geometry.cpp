#include "geometry.h"
#include <algorithm>
#include <sstream>
#include "circ_iter.h"
#include <numeric>

using Triangle = Simplex<3>;
using Edge = Simplex<2>;

LinesParallel::LinesParallel(std::string s) : msg(std::move(s))
{}
std::string const& LinesParallel::what() const {
	return msg;
}

Point Line::operator()(double t) const
{
	return a*(1 - t) + b*t;
}

std::tuple<double, double> intersection(Point const& origin, Point const& dir, Line const& l)
{
	Point p = l.a - l.b;
	Point q = l.a - origin;

	double det = p.x*dir.y - p.y*dir.x;

	if (det == 0.0) {
		std::ostringstream errMsg;
		errMsg << "intersection(): LinesParallel: " << origin <<' '<< dir <<' '<< l.a <<" - "<< l.b << '\n';
		throw LinesParallel(errMsg.str());
	}

	double det1 = q.x*dir.y - q.y*dir.x;
	double det2 = p.x*q.y - p.y*q.x;

	double t = det1 / det;
	double u = det2 / det;
	return{t,u};
}

Edge common_edge(Triangle t1, Triangle t2)
{
	std::sort(t1.begin(), t1.end());
	std::sort(t2.begin(), t2.end());
	// preconditions
	if (t1 == t2)
	{
		throw FlipError("equal triangles");
	}

	Edge e{-1,-1};
	std::set_intersection(t1.cbegin(), t1.cend(), t2.cbegin(), t2.cend(), e.begin());
	return e;
}

bool valid_edge(Edge const& e)
{
	return !(e[0] == -1 || e[1] == -1);
}

Simplex<1> set_difference(Triangle const& t, Edge const& e)
{
	Simplex<1> v;
	std::set_difference(t.cbegin(), t.cend(), e.cbegin(), e.cend(), v.begin());
	return v;
}

Triangle set_union(Edge& e, Simplex<1> s)
{
	Triangle t;
	std::set_union(e.begin(), e.end(), s.begin(), s.end(), t.begin());
	return t;
}

FlipError::FlipError(std::string s) : msg(s)
{}
std::string const& FlipError::what() const
{
	return msg;
}

std::tuple<Triangle, Triangle> flip(Triangle t1, Triangle t2)
{
	using namespace std;

	sort(t1.begin(), t1.end());
	sort(t2.begin(), t2.end());

	Edge e = common_edge(t1, t2);
	Simplex<1> v1 = set_difference(t1, e);
	Simplex<1> v2 = set_difference(t2, e);

	Edge ne{v1[0],v2[0]};
	if (ne[0] > ne[1])
		std::swap(ne[0], ne[1]);

	t1 = set_union(ne, Simplex<1>{e[0]});
	t2 = set_union(ne, Simplex<1>{e[1]});

	return {t1, t2};
}

std::ostream& operator<< (std::ostream& os, Side const& s)
{
	switch (s) {
	case Side::coin: os << "Side::coin"; break;
	case Side::left: os << "Side::left"; break;
	case Side::right: os << "Side::right"; break;
	}
	return os;
}

Side side(Point const& a, Point const& b, Point const& c)
{
	double det = (c.x-a.x)*(b.y-a.y) - (b.x-a.x)*(c.y-a.y);
	if (det == 0.0)
		return Side::coin;
	else if (det < 0.0)
		return Side::left;
	else
		return Side::right;
}

Point circumCenter(std::vector<Point> const& v, Triangle const& t)
{
	Point const& a = v[t[0]];
	Point const& b = v[t[1]];
	Point const& c = v[t[2]];
	double x = sqNorm(a)*(b.y - c.y) + sqNorm(b)*(c.y - a.y) + sqNorm(c)*(a.y - b.y);
	double y = sqNorm(a)*(c.x - b.x) + sqNorm(b)*(a.x - c.x) + sqNorm(c)*(b.x - a.x);
	double D = 2.0f * (a.x*(b.y - c.y) + b.x*(c.y - a.y) + c.x*(a.y - b.y));

	return Point{x / D, y / D};
}

double circumRadius(std::vector<Point> const& v, Triangle const& t)
{
	Point const& p1 = v[t[0]];
	Point const& p2 = v[t[1]];
	Point const& p3 = v[t[2]];
	double a = norm(p1 - p2);
	double b = norm(p2 - p3);
	double c = norm(p3 - p1);

	double d = a*b*c / sqrt((a + b + c)*(-a + b + c)*(a - b + c)*(a + b - c));
	return d;
}

Circle circumCircle(std::vector<Point> const& v, Triangle const& t)
{
	return Circle{circumCenter(v,t), circumRadius(v,t)};
}