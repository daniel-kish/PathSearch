#include <iostream>
#include <iomanip>
#include "Point.h"
#include <vector>
#include "geometry.h"
#include "DCEL.h"
#include "Delaunay.h"
#include "Kirkpatrick.h"
#include <chrono>
#include <random>


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
			insert_point(d, root.get(), p);

		ins = rectHull(Rect{{100,100},{-150,-150}}, 300, 300);
		std::shuffle(ins.begin(), ins.end(), std::mt19937{rd()});
		total_ins += ins.size();
		for (Point const& p : ins)
			insert_point(d, root.get(), p);

		ins = rectHull(Rect{{10,120},{-50,0}}, 50, 500);
		std::shuffle(ins.begin(), ins.end(), std::mt19937{rd()});
		total_ins += ins.size();
		for (Point const& p : ins)
			insert_point(d, root.get(), p);

		ins = circleHull(Circle{{80,80},80}, 800);
		total_ins += ins.size();
		std::shuffle(ins.begin(), ins.end(), std::mt19937{rd()});
		for (Point const& p : ins)
			insert_point(d, root.get(), p);
	}
	auto t2 = high_resolution_clock::now();
	std::cout << total_ins << '\n';
	std::cout << duration_cast<milliseconds>(t2 - t1).count() << " ms\n";

	std::cout << "formula: " << d.faces.size() - 1 << ' ' << d.vertices.size() + total_ins - 2 << '\n';

}
catch (std::exception const& e)
{
	std::cerr << e.what() << '\n';
}