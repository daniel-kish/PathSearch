#pragma once
#include <vector>
#include "Point.h"
#include "geometry.h"
#include <random>

struct OriginVec {
	Point origin;
	Point dir;
	std::vector<double> inters;
	double uRight;
	double uLeft;

	Point operator()(double u);
};

class TrapezePathFinder {
	using PointRef = std::reference_wrapper<Point>;
	std::mt19937 mt;
public:
	Rect r;
	std::vector<Poly> obstacles;
	std::vector<PointRef> points;
	std::vector<OriginVec> lines;

	std::vector<Line> line_segments;

	void clear();

	TrapezePathFinder();
	void mkLines();
	Point minp;
};