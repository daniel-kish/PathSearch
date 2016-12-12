#define _USE_MATH_DEFINES
#include <iostream>
#include <array>
#include <vector>
#include <fstream>
#include <string>
#include <sstream>
#include <algorithm>
#include <cmath>
using namespace std;

template <typename T> int sgn(T val) {
	return (T(0) < val) - (val < T(0));
}

using point = tuple<double, double>;

double& X(point& p) { return std::get<0>(p); }
double const& X(point const& p) { return std::get<0>(p); }
double& Y(point& p) { return std::get<1>(p); }
double const& Y(point const& p) { return std::get<1>(p); }

point operator-(point const& p, point const& q) { return{X(p) - X(q),Y(p) - Y(q)}; }
point operator+(point const& p, point const& q) { return{X(p) + X(q),Y(p) + Y(q)}; }

double cross(point const& p, point const& q)
{
	return X(p)*Y(q) - X(q)*Y(p);
}

double operator*(point const& p, point const& q)
{
	return X(p)*X(q) + Y(p)*Y(q);
}

double signedArea(point const& a, point const& b, point const& p)
{
	point ab{X(b) - X(a),Y(b) - Y(a)};
	point ap{X(p) - X(a),Y(p) - Y(a)};
	return cross(ab, ap);
}

template <unsigned dim>
using simplex = std::array<std::size_t, dim + 1>;

simplex<1> common_edge(simplex<2> const& s1, simplex<2> const& s2)
{
	simplex<1> e;
	// std::sort(begin(s1),end(s1)); std::sort(begin(s2),end(s2));
	std::set_intersection(s1.begin(), s1.end(), s2.begin(), s2.end(), e.begin());
	return e;
}

struct Graphics
{
	double scale{50.0};
	double wid, height;
	double X0, Y0;
	std::string main_code;
	std::string ending;
	double pointRad{4.0};
	double lineWidth{2.0};
	Graphics(double w = 1000.0, double h = 500.0, double X = 0.0, double Y = 0.0)
		: X0{X}, Y0{Y}, wid{w}, height{h}
	{
		main_code =
			R"(<?xml version="1.0" encoding="UTF-8" standalone="no"?>)"
			"\n"
			R"(<svg version = "1.1" )" "\n"
			R"(baseProfile="full" )" "\n"
			R"(xmlns = "http://www.w3.org/2000/svg" )" "\n"
			R"(xmlns:xlink = "http://www.w3.org/1999/xlink" )" "\n"
			R"(xmlns:ev = "http://www.w3.org/2001/xml-events" )" "\n"
			R"(height = "1000px"  width = "1200px">)" "\n";

		std::ostringstream tmp;
		tmp << R"(<rect x=")" << X0 << R"(" y=")" << Y0 << R"(" width=")" << wid
			<< R"(" height=")" << height << R"(" fill="none" stroke="black" stroke-width="5px"/>)";
		main_code.append(tmp.str() + "\n");
		tmp.str(""); tmp.clear();
		tmp << R"(<g transform="translate)" << '(' << wid / 2 << ',' << height / 2 << ')'
			<< R"q( scale)q" << '(' << scale << ',' << -scale << ')' << R"q("> )q";
		main_code.append(tmp.str() + "\n");
		ending = "\n</g>\n</svg>";
	}

	void addLine(point const& p, point const& q)
	{
		std::ostringstream str;
		str << R"( <line x1=")" << X(p) << R"(" y1=")" << Y(p)
			<< R"(" x2=")" << X(q) << R"(" y2=")" << Y(q)
			<< R"(" style="stroke:black;stroke-width:)" << lineWidth / scale << R"("/> )"
			<< '\n';
		main_code.append(str.str());
	}
	void addPoint(point const& p)
	{
		std::ostringstream str;
		str << R"(<circle cx=")" << X(p)
			<< R"(" cy=")" << Y(p) << R"(" r=")" << pointRad / scale << R"("  fill="black" />)"
			<< '\n';
		main_code.append(str.str());
	}
	void output(std::ofstream& svgfile)
	{
		svgfile << main_code << ending;
	}
};


struct Complex {
	vector<point> nodes;
	vector<simplex<2>> cells;
	static double bigCoord;
	Complex()
		:nodes{{0,bigCoord},{-bigCoord,-bigCoord},{bigCoord,-bigCoord}}, cells{{0,1,2}}
	{}
	void add_point(point const& pt)
	{
		Pos pos;
		auto incident = [this, pt, &pos](simplex<2> const& s) {
			pos = pointAndTriangle(pt, s);
			return int(pos) > 0;
		};
		auto fnd = std::find_if(begin(cells), end(cells), incident);
		if (fnd == cells.end()) return;
		if (int(pos) > 1) {
			fnd = add_point_on_edge(fnd, pt, int(pos) - 2);
			auto fnd2 = std::find_if(fnd, end(cells), incident);
			if (fnd2 != end(cells)) 
				add_point_on_edge(fnd2, pt, int(pos) - 2);
		}
		else if (pos == Pos::inside)
			add_point_inside(fnd, pt);
	}
	vector<simplex<2>>::iterator add_point_on_edge(vector<simplex<2>>::iterator fnd, point const& p, int dividedEdgeIdx)
	{
		simplex<2> s = *fnd;
		auto ret = cells.erase(fnd);
		auto d = std::distance(cells.begin(), ret);
		nodes.push_back(p);
		auto np = nodes.size() - 1;
		simplex<1> edges[3] = {{s[0],s[1]},{s[1],s[2]},{s[0],s[2]}};
		std::swap(edges[dividedEdgeIdx], edges[2]); // move divided -> to the last place

		for (int i = 0; i < 2; ++i)
			cells.push_back(simplex<2>{edges[i][0], edges[i][1], np});
		return begin(cells)+d;
	}
	void add_point_inside(vector<simplex<2>>::iterator fnd, point const& p)
	{
		simplex<2> s = *fnd;
		cells.erase(fnd);
		nodes.push_back(p);
		auto np = nodes.size() - 1;
		simplex<1> edges[3] = {{s[0],s[1]},{s[1],s[2]},{s[0],s[2]}};
		std::for_each(begin(edges), end(edges), [this, np](simplex<1> const& edge) {
			cells.push_back(simplex<2>{edge[0], edge[1], np});
		});
	}
	enum Pos { out, inside, e01, e12, e20 };
	Pos pointAndTriangle(point const& pt, simplex<2> const& cell)
	{
		auto p = [this, cell](std::size_t n)->point const&
		{
			return nodes[cell[n]];
		};
		auto same_sign = [](auto s1, auto s2, auto s3) 
		{
			return sgn(s1) == sgn(s2) && sgn(s2) == sgn(s3); 
		};

		double s1, s2, s3;
		if ((s1 = signedArea(p(0), p(1), pt)) == 0.0) {
			double scp = (p(1) - p(0))*(pt - p(0)) / ((p(1) - p(0))*(p(1) - p(0)));
			if (scp < 0 || scp > 1) return Pos::out;
			else return Pos::e01;
		}
		else if ((s2 = signedArea(p(1), p(2), pt)) == 0.0) {
			double scp = (p(2) - p(1))*(pt - p(1)) / ((p(2) - p(1))*(p(2) - p(1)));
			if (scp < 0 || scp > 1) return Pos::out;
			else return Pos::e12;
		}
		else if ((s3 = signedArea(p(2), p(0), pt)) == 0.0) {
			double scp = (p(0) - p(2))*(pt - p(2)) / ((p(0) - p(2))*(p(0) - p(2)));
			if (scp < 0 || scp > 1) return Pos::out;
			else return Pos::e20;
		}
		else if (same_sign(s1, s2, s3)) return Pos::inside;
		else return Pos::out;
	}

};

void addTrian(Graphics & ga, Complex const& cmplx)
{
	for (auto const& p : cmplx.nodes)
		ga.addPoint(p);
	for (auto const& tr : cmplx.cells)
	{
		ga.addLine(cmplx.nodes[tr[0]], cmplx.nodes[tr[1]]);
		ga.addLine(cmplx.nodes[tr[1]], cmplx.nodes[tr[2]]);
		ga.addLine(cmplx.nodes[tr[2]], cmplx.nodes[tr[0]]);
	}
}

double Complex::bigCoord = 12;
int main()
{
	Complex c;
	Graphics ga;
	for (double f = 0.0; f < 2.0*M_PI; f += 2.0*M_PI / 20)
		c.add_point({2.0*cos(f),2.0*sin(f)});
	std::ofstream file("graph.xml");
	addTrian(ga, c);
	ga.output(file);
}