#include <iostream>
#include <iomanip>
#include "Point.h"
#include <list>
#include <vector>
#include "geometry.h"
#include <cassert>
#include "DCEL.h"
#include "Delaunay.h"
#include "Kirkpatrick.h"
#include <set>
#include <chrono>
#include <random>
#include <queue>
#include "boost\variant.hpp"
#include <stack>
#include <map>

Position point_triangle_position(Point const& a, Point const& b, Point const& c, Point const& p)
{
	Point clos; TriPos pos;
	std::tie(clos, pos) = ClosestPointOnTriangle(p, a, b, c);

	if (pos == TriPos::inside)
		return Position::in;

	double sq_dist = sqNorm(clos - p);
	const double sqeps = 1.0e-14;
	std::cout << std::setprecision(std::abs(log10(sqeps))) << sq_dist << '\n';
	bool close_enough = sq_dist < sqeps;

	switch (pos)
	{
	case TriPos::V0:
	case TriPos::V1:
	case TriPos::V2:
		if (close_enough)
			return Position::out;
	case TriPos::edge01:
		if (close_enough)
			return Position::edge01;
	case TriPos::edge12:
		if (close_enough)
			return Position::edge12;
	case TriPos::edge20:
		if (close_enough)
			return Position::edge02;
	}
}

int main()
try {
	using namespace std::chrono;

	double coord = 5'000;
	std::vector<Point> poly{{-coord,-coord},{coord,-coord},{0,coord}};
	DCEL d(mk_CCW_poly(poly));
	auto poly_face = std::next(d.faces.begin());
	d.out_face = d.faces.begin();
	d.out_face->hist = nullptr;

	auto root = std::make_unique<treeNode>(poly_face);
	poly_face->hist = root.get(); // hand-shaking

	int total_ins = 0;
	std::random_device rd;
	auto t1 = high_resolution_clock::now();

	{
		std::vector<Point> ins = rectHull(Rect{{400,400},{-200,-200}}, 400, 400);
		std::shuffle(ins.begin(), ins.end(), std::mt19937{rd()});
		total_ins += ins.size();
		for (Point const& p : ins)
			insert_point(d/*, root.get()*/, p);

		ins = rectHull(Rect{{100,100},{-150,-150}}, 300, 300);
		std::shuffle(ins.begin(), ins.end(), std::mt19937{rd()});
		total_ins += ins.size();
		for (Point const& p : ins)
			insert_point(d/*, root.get()*/, p);

		ins = rectHull(Rect{{10,120},{-50,0}}, 50, 500);
		std::shuffle(ins.begin(), ins.end(), std::mt19937{rd()});
		total_ins += ins.size();
		for (Point const& p : ins)
			insert_point(d/*, root.get()*/, p);

		ins = circleHull(Circle{{80,80},80}, 800);
		total_ins += ins.size();
		std::shuffle(ins.begin(), ins.end(), std::mt19937{rd()});
		for (Point const& p : ins)
			insert_point(d/*, root.get()*/, p);
	}
	auto t2 = high_resolution_clock::now();
	std::cout << total_ins << '\n';
	std::cout << duration_cast<milliseconds>(t2 - t1).count() << " ms\n";



	std::cout << "formula: " << d.faces.size() - 1 << ' ' << d.vertices.size() + total_ins - 2 << '\n';
	//for (auto f = d.faces.begin(); f != d.faces.end(); ++f) {
	//	if (f->hist)
	//		std::cout << f->i << ' ' << boost::get<DCEL::FaceList::iterator>(f->hist->face_data)->i
	//		<< ' ' << f->hist->successors.size() << '\n';
	//}
}
catch (std::exception const& e)
{
	std::cerr << e.what() << '\n';
}