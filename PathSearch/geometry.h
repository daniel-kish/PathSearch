#pragma once
#include "Point.h"
#include <tuple>
#include <array>
#include <string>
#include <exception>
#include <vector>

extern const double doubleEps;
bool zero(double val);
bool close(double v, double u);
bool close(Point const& p, Point const& q);

struct Line {
	Point a, b;
	Point operator()(double t) const;
};

struct Circle {
	Point center;
	double rad;
};

struct Rect {
	Point dir;
	Point origin;
	Rect (Point v, Point or = Point{})
		: dir(v), origin(or)
	{	}
	Rect() : dir{}, origin{}
	{}
};

struct LinesParallel
{
	std::string msg;
	LinesParallel(std::string s);
	std::string const& what() const;
};

std::tuple<double, double> intersection(Point const& origin, Point const& dir, Line const& l);

template <unsigned N>
struct Simplex
{
	using array = std::array<int, N>;
	using iterator = typename array::iterator;
	using const_iterator = typename array::const_iterator;
	using reference = typename array::reference;
	using const_reference = typename array::const_reference;
	using size_type = typename array::size_type;

	array pts;
	iterator begin() {
		return pts.begin();
	}
	iterator end() {
		return pts.end();
	}
	const_iterator cbegin() const {
		return pts.cbegin();
	}
	const_iterator cend() const {
		return pts.cend();
	}
	reference operator[](size_type pos) {
		return pts[pos];
	}
	const_reference operator[](size_type pos) const {
		return pts[pos];
	}
};

template <unsigned N>
std::ostream& operator<< (std::ostream& os, Simplex<N> const& s)
{
	os << "( ";
	for (auto p = s.cbegin(); p != s.cend(); ++p)
		os << *p << ' ';
	return os << ')';
}

using Triangle = Simplex<3>;
using Edge = Simplex<2>;

template <unsigned N>
bool operator==(Simplex<N> const& s1, Simplex<N> const& s2)
{
	return s1.pts == s2.pts;
}

template <unsigned N>
bool operator<(Simplex<N> const& s1, Simplex<N> const& s2)
{
	return s1.pts < s2.pts;
}

Edge common_edge(Triangle t1, Triangle t2);
int have_common_vertices(Triangle t1, Triangle t2);
bool have_common_vertices(Edge e1, Edge e2);
bool valid_edge(Edge const& e);
Simplex<1> set_difference(Triangle const& t, Edge const& e);
Triangle set_union(Edge& e, Simplex<1> s);

struct FlipError {
	std::string msg;
	FlipError(std::string s);
	std::string const& what() const;
};
std::tuple<Triangle, Triangle> flip(Triangle t1, Triangle t2, Edge e);

enum PointEdgePos { closestToV0, closestToV1, closestToEdge };
enum class TriPos {
	V0,
	V1,
	V2,
	edge01,
	edge12,
	edge20,
	inside
};
std::ostream& operator<< (std::ostream& os, TriPos const& p);

PointEdgePos pointAndEdge(Point const& a, Point const& b, Point const& p);

//TriPos pointAndTriangle(Point const& p, Point const& a, Point const& b, Point const& c);

Point projectOnEdge(Point const& a, Point const& b, Point const& p);
std::tuple<Point,TriPos> ClosestPointOnTriangle(Point const& p, Point const& a, Point const& b, Point const& c);

enum class Side {
	right, left, coin, collinear
};

std::ostream& operator<< (std::ostream& os, Side const& s);

Side side(Point const& a, Point const& b, Point const& c);

enum class Position {
	out, edge01, edge12, edge02, in
};
std::ostream& operator<< (std::ostream& os, Position const& p);

Position insideTriangle(Point const& a, Point const& b, Point const& c, Point const& p);

template <class Cont>
Position insideTriangle(Cont const& pts, Triangle const& t, Point const& p)
{
	auto s1 = side(pts[t[0]], pts[t[1]], p);
	auto s2 = side(pts[t[1]], pts[t[2]], p);
	auto s3 = side(pts[t[2]], pts[t[0]], p);
	if (s1 == s2 && s1 == s3) // in
		return Position::in;
	else if (s1 == Side::coin) {
		return Position::edge01;
	}
	else if (s2 == Side::coin) {
		return Position::edge12;
	}
	else if (s3 == Side::coin) {
		return Position::edge02;
	}
	return Position::out; // out
}

Point circumCenter(std::vector<Point> const& v, Triangle const& t);
double circumRadius(std::vector<Point> const& v, Triangle const& t);
Circle circumCircle(std::vector<Point> const& v, Triangle const& t);

Point triangleCenter(std::vector<Point> const& v, Triangle const& t);

enum PolyOrientation {
	CCW, CW
};
using Poly = std::vector<Point>;

PolyOrientation orientation(std::vector<Point> const& poly);
std::vector<Point> polygonPts(std::vector<Point> const& pts, std::vector<int> poly);

bool insidePoly(std::vector<Point> const& poly, Point const& p);

bool inCircle(Circle const& c, Point const& p);
Circle circumCircle(Point const& a, Point const& b, Point const& c);
Point circumCenter(Point const& a, Point const& b, Point const& c);
double circumRadius(Point const& p1, Point const& p2, Point const& p3);