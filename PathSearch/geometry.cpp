#include "geometry.h"
#include <algorithm>
#include <sstream>
#include "circ_iter.h"
#include <numeric>
#include <random>

using Triangle = Simplex<3>;
using Edge = Simplex<2>;

double doubleEps = 1.0e-2;

bool zero(double val) { return abs(val) < doubleEps; }

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
	return {t,u};
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

int have_common_vertices(Triangle t1, Triangle t2)
{
	std::sort(t1.begin(), t1.end());
	std::sort(t2.begin(), t2.end());

	std::vector<int> verts;
	std::set_intersection(t1.begin(), t1.end(), t2.begin(), t2.end(), std::back_inserter(verts));
	return verts.size();
}

bool have_common_vertices(Edge e1, Edge e2)
{
	if (e1[0] == e2[0] || e1[0] == e2[1] || e1[1] == e2[0] || e1[1] == e2[1])
		return true;
	return false;
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

std::tuple<Triangle, Triangle> flip(Triangle t1, Triangle t2, Edge e)
{
	using namespace std;

	sort(t1.begin(), t1.end());
	sort(t2.begin(), t2.end());

	//Edge e = common_edge(t1, t2);
	Simplex<1> v1 = set_difference(t1, e);
	Simplex<1> v2 = set_difference(t2, e);

	Edge ne{v1[0],v2[0]};
	if (ne[0] > ne[1])
		std::swap(ne[0], ne[1]);

	t1 = set_union(ne, Simplex<1>{e[0]});
	t2 = set_union(ne, Simplex<1>{e[1]});

	return {t1, t2};
}

std::ostream & operator<<(std::ostream & os, TriPos const & p)
{
	switch (p)
	{
	case TriPos::edge01: os << "edge01"; break;
	case TriPos::edge12: os << "edge12"; break;
	case TriPos::edge20: os << "edge20"; break;
	case TriPos::inside: os << "inside"; break;
	case TriPos::V0: os << "V0"; break;
	case TriPos::V1: os << "V1"; break;
	case TriPos::V2: os << "V2"; break;
	}
	return os;
}

PointEdgePos pointAndEdge(Point const& a, Point const& b, Point const& p)
{
	Point ab = b - a;
	Point ap = p - a;
	double s1 = ab*ap;
	if (s1 <= 0.0)
		return closestToV0;
	else {
		double s2 = (a - b)*(p - b);
		if (s2 <= 0.0)
			return closestToV1;
		else
			return closestToEdge;
	}
}

//TriPos pointAndTriangle(Point const& p, Point const& a, Point const& b, Point const& c)
//{
//	auto zero = [](double v) { return std::abs(v) < doubleEps; };
//	Point ap = p - a;
//	Point ab = b - a;
//	Point ac = c - a;
//	double detA = (ab*ab)*(ac*ac) - (ab*ac)*(ac*ab);
//	double det1 = (ap*ab)*(ac*ac) - (ap*ac)*(ac*ab);
//	double det2 = (ab*ab)*(ap*ac) - (ab*ac)*(ap*ab);
//	double u = det1/detA;
//	double v = det2/detA;
//	
//	if (u < 0.0)
//	{
//		if (v < 0.0)
//			return TriPos::closestToV0;
//		else if (zero(v))
//			TriPos::closestToEdge20;
//		else // D
//		{
//			if (v <= 1.0)
//				return TriPos::closestToEdge20;
//			else
//				return TriPos::closestToV2;
//		}
//	}
//	else if (zero(u))
//	{
//		if (v < 0.0)
//			return TriPos::closestToV0;
//		else if (zero(v))
//			TriPos::onV0;
//		else // E
//		{
//			if (v > 1.0) return TriPos::onEdge20;
//			else if (v == 1.0) return TriPos::onV2;
//			else return TriPos::closestToV2;
//		}
//	}
//	else // u > 0.0
//	{
//		if (v < 0.0)
//		{
//			if (u <= 1.0) return TriPos::closestToEdge01;
//			else return TriPos::closestToV1;
//		}
//		else if (zero(v))
//		{
//			if (u < 1.0) return TriPos::onEdge20;
//			else if (u == 1.0) return TriPos::onV1;
//			else return TriPos::closestToV1;
//		}
//		else
//		{
//			if (u + v < 1.0) return TriPos::inside;
//			else if (u + v == 1.0) return TriPos::onEdge12;
//			else {
//				Point bp = p - b, bc = c - b;
//				double w = (bp*bc) / (bc*bc);
//				if (w < 0.0) return TriPos::closestToV1;
//				else if (w < 1.0) return TriPos::closestToEdge12;
//				else if (w > 1.0) return TriPos::closestToV2;
//			}
//		}
//	}
//}


std::tuple<Point, TriPos> ClosestPointOnTriangle(Point const& p, Point const& a, Point const& b, Point const& c)
{
	Point ab = b - a;
	Point ac = c - a;
	Point ap = p - a;
	float d1 = ab * ap;
	float d2 = ac * ap;
	if (d1 <= 0.0f && d2 <= 0.0f) 
		return {a,TriPos::V0}; // onV0 || closestToV0
	Point bp = p - b;
	float d3 = ab * bp;
	float d4 = ac * bp;
	if (d3 >= 0.0f && d4 <= d3) 
		return {b,TriPos::V1}; // onV1 || closestToV1
	float vc = d1*d4 - d3*d2;
	if (vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f) {
		float v = d1 / (d1 - d3);
		return {a + ab * v, TriPos::edge01}; // onEdge01 || closestToEdge01
	}
	
	Point cp = p - c;
	float d5 = ab * cp;
	float d6 = ac * cp;
	if (d6 >= 0.0f && d5 <= d6) 
		return {c,TriPos::V2}; // onV2 || closestToV2
		
	float vb = d5*d2 - d1*d6;
	if (vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f) {
		float w = d2 / (d2 - d6);
		return {a + ac*w,TriPos::edge20}; // onEdge20 || closestToEdge20
	}
	
	float va = d3*d6 - d5*d4;
	if (va <= 0.0f && (d4 - d3) >= 0.0f && (d5 - d6) >= 0.0f) {
		float w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
		return {b + (c - b)*w, TriPos::edge12}; // onEdge12 || closestToEdge12
	}
	float denom = 1.0f / (va + vb + vc);
	float v = vb * denom;
	float w = vc * denom;
	return {a + ab * v + ac * w, TriPos::inside}; // inside
}

Point projectOnEdge(Point const& a, Point const& b, Point const& p)
{
	Point ap = p - a;
	Point ab = b - a;
	double eta = (ap*ab) / (ab*ab);
	return a*(1 - eta) + b*eta;
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

std::ostream& operator<< (std::ostream& os, Position const& p)
{
	switch (p) {
	case Position::out: os << "Position::out"; break;
	case Position::edge01: os << "Position::edge01"; break;
	case Position::edge12: os << "Position::edge12"; break;
	case Position::edge02: os << "Position::edge02"; break;
	case Position::in: os << "Position::in"; break;
	}
	return os;
}

Side side(Point const& a, Point const& b, Point const& c)
{
	double det = (c.x-a.x)*(b.y-a.y) - (b.x-a.x)*(c.y-a.y);
	if (det == 0.0)
	{
		double s1 = (b - a)*(c - a);
		double s2 = (a - b)*(c - b);
		if (s1 > 0.0 && s2 > 0.0)
			return Side::coin;
		return Side::collinear;
	}
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

Point triangleCenter(std::vector<Point> const& v, Triangle const& t)
{
	Point a = v[t[0]];
	Point b = v[t[1]];
	Point c = v[t[2]];

	Point center = (a + b + c) * (1.0/3.0);
	return center;
}


PolyOrientation orientation(std::vector<Point>& poly)
{
	auto x = [](Point const& p) {return p.x; };
	auto y = [](Point const& p) {return p.y; };

	auto p = std::min_element(poly.begin(), poly.end());

	Point a = (p == poly.begin()) ? poly.back() : *(p - 1);
	Point b = *p;
	Point c = (p == poly.end() - 1) ? poly.front() : *(p + 1);

	double det = (x(b) - x(a))*(y(c) - y(a)) - (x(c) - x(a))*(y(b) - y(a));

	if (det < 0.0) return PolyOrientation::CW;
	else return PolyOrientation::CCW;
}

std::vector<Point> polygonPts(std::vector<Point> const& pts, std::vector<int> poly)
{
	std::vector<Point> polyPts(poly.size());
	std::transform(poly.begin(), poly.end(), polyPts.begin(), [pts](int i) {
		return pts[i];
	});

	PolyOrientation or = orientation(polyPts);
	if (or == PolyOrientation::CW)
		std::reverse(polyPts.begin(), polyPts.end());
	return polyPts;
}

bool insidePoly(std::vector<Point> const& poly, Point const& p)
{
	std::mt19937 mt{};
	std::uniform_real_distribution<double> ud(0.0, 1.0);
	Point dir{ud(mt),ud(mt)};
	int intersections = 0;
	for (int i = 0; i < poly.size()-1; ++i)
	{
		Line edge{poly[i],poly[i + 1]};
		double t, u;
		std::tie(t,u) = intersection(p, dir, edge);
		if (t >= 0.0 && t <= 1.0 && u >= 0.0)
			intersections++;
	}
	Line edge{poly.back(),poly.front()};
	double t, u;
	std::tie(t, u) = intersection(p, dir, edge);
	if (t >= 0.0 && t <= 1.0 && u >= 0.0)
		intersections++;
	return intersections % 2;
}