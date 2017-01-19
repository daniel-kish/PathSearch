#include <iostream>
#include <fstream>
#include <iomanip>
#include "Point.h"
#include <vector>
#include "geometry.h"
#include "DCEL.h"
#include "Delaunay.h"
#include "Kirkpatrick.h"
#include <chrono>
#include <random>
#include <set>
#include <map>
#include <stack>


//std::chrono::milliseconds
//polar_hull_test(int n)
//{
//	using namespace std::chrono;
//	double coord = 5'000;
//	std::vector<Point> poly{{-coord,-coord},{coord,-coord},{0,coord}};
//	DCEL d(mk_CCW_poly(poly));
//	auto poly_face = d.faces.begin();
//	d.out_face = std::next(d.faces.begin());
//	d.out_face->hist = nullptr;
//	auto root = std::make_unique<treeNode>(poly_face);
//	poly_face->hist = root.get(); // hand-shaking
//
//
//	int total_ins = 0;
//	std::random_device rd;
//	std::vector<Point> pts = polarHull([](double f) {return 200.0 + 50.0*sin(5.0*f); },
//	{0,0}, n);
//	auto t1 = high_resolution_clock::now();
//	std::shuffle(pts.begin(), pts.end(), std::mt19937{});
//
//	total_ins = pts.size();
//
//
//	for (Point const& p : pts)
//		insert_point(d, root.get(), p);
//
//	auto t2 = high_resolution_clock::now();
//	//std::cout << total_ins << '\n';
//	std::cout << "formula: " << d.faces.size() - 1 << ' ' << d.vertices.size() + total_ins - 2 << '\n';
//
//	auto dur = duration_cast<milliseconds>(t2 - t1);
//	return dur;
//}
//
//std::tuple<int, std::chrono::milliseconds>
//rect_hull_test(int n)
//{
//	using namespace std::chrono;
//	double coord = 5'000;
//	std::vector<Point> poly{{-coord,-coord},{coord,-coord},{0,coord}};
//	DCEL d(mk_CCW_poly(poly));
//	auto poly_face = d.faces.begin();
//	d.out_face = std::next(d.faces.begin());
//	d.out_face->hist = nullptr;
//	auto root = std::make_unique<treeNode>(poly_face);
//	poly_face->hist = root.get(); // hand-shaking
//
//
//	int total_ins = 0;
//	std::random_device rd;
//	std::vector<Point> pts = rectHull(Rect({500,500}), n, n);
//	total_ins = pts.size();
//
//	auto t1 = high_resolution_clock::now();
//	std::shuffle(pts.begin(), pts.end(), std::mt19937{});
//
//	for (Point const& p : pts)
//		insert_point(d, root.get(), p);
//
//	auto t2 = high_resolution_clock::now();
//	//std::cout << total_ins << '\n';
//	std::cout << "formula: " << d.faces.size() - 1 << ' ' << d.vertices.size() + total_ins - 2 << '\n';
//
//	auto dur = duration_cast<milliseconds>(t2 - t1);
//	return{d.vertices.size(), dur};
//}
//
//std::chrono::milliseconds
//random_test(int n)
//{
//	using namespace std::chrono;
//	double coord = 5'000;
//	std::vector<Point> poly{{-coord,-coord},{coord,-coord},{0,coord}};
//	DCEL d(mk_CCW_poly(poly));
//	auto poly_face = d.faces.begin();
//	d.out_face = std::next(d.faces.begin());
//	d.out_face->hist = nullptr;
//	auto root = std::make_unique<treeNode>(poly_face);
//	poly_face->hist = root.get(); // hand-shaking
//
//
//	int total_ins = 0;
//	std::random_device rd;
//	std::vector<Point> pts = rectInsidesRand(Rect({500,500}), n);
//	total_ins = pts.size();
//
//	auto t1 = high_resolution_clock::now();
//
//	for (Point const& p : pts)
//		insert_point(d, root.get(), p);
//
//	auto t2 = high_resolution_clock::now();
//	//std::cout << total_ins << '\n';
//	std::cout << "formula: " << d.faces.size() - 1 << ' ' << d.vertices.size() + total_ins - 2 << '\n';
//
//	auto dur = duration_cast<milliseconds>(t2 - t1);
//	return dur;
//}

int get_depth(treeNode* root)
{
	struct entry { treeNode* ptr; int depth; };
	std::stack<entry> q;
	q.push({root,0});

	int max = 0;
	while (!q.empty())
	{
		entry current = q.top(); q.pop();
		if (current.ptr->successors.empty())
		{
			if (max < current.depth)
				max = current.depth;
		}
		for (auto const& succ : current.ptr->successors)
			q.push({succ.get(),current.depth+1});
	}
	return max;
}

int main()
try {
	double coord = 5'000;
	std::vector<Point> out_poly = {{-coord,-coord},{coord,-coord},{0,coord}};
	DCEL d(mk_CCW_poly(out_poly));
	auto poly_face = d.faces.begin();
	d.out_face = std::next(d.faces.begin());
	
	treeNode tn(poly_face,0);
	poly_face->hist = &tn;
	num_vertices = 1;
	depth = 0;

	//auto ins = rectHull(Rect({500,500}, {0,0}), 10, 10);
	auto ins = rectHull(Rect({500,500}, {-250,-250}), 12, 13);
	std::cout << "inserting " << ins.size() << '\n';
	std::shuffle(ins.begin(), ins.end(), std::mt19937{std::random_device{}()});
	for (const Point& p : ins) {
		insert_point(d, &tn, p);
	}

	std::cout << "|v| = " << num_vertices << '\n';
	std::cout << "d(g) = " << depth << '\n';

	//return 1;
	std::set<treeNode*> vertices;
	std::stack<treeNode*> q;
	q.push(&tn);
	while (!q.empty())
	{
		treeNode* current = q.top(); q.pop();
		vertices.insert(current);
		for (auto const& succ : current->successors)
			q.push(succ.get());
		//std::cout << vertices.size() << '\n';
	}
	std::cout << "done vertices: " << vertices.size() << '\n';
	
	int lbl = 0;
	std::map<treeNode*, int> labels;
	for (auto const& v : vertices)
		labels[v] = lbl++;

	vertices.clear();
	std::cout << "done labeling\n";

	std::set<std::tuple<int, int>> edges;

	q.push(&tn);
	while (!q.empty())
	{
		treeNode* current = q.top(); q.pop();
		for (auto const& succ : current->successors) {
			edges.insert({labels[current],labels[succ.get()]});
			q.push(succ.get());
		}
	}
	std::cout << "done edges\n";

	std::ofstream os{R"(C:\Users\Daniel\Documents\Visual Studio 2015\Projects\PathSearch\graph.dat)"};
	for (auto const& e : edges)
		os << std::get<0>(e) << ' ' << std::get<1>(e) << '\n';

	

}
catch (std::exception const& e)
{
	std::cerr << e.what() << '\n' << typeid(e).name() << '\n';
}