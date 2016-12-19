#pragma once
#include <vector>
#include "Point.h"
#include "geometry.h"

class TrapezePathFinder {
	using PointRef = std::reference_wrapper<Point>;
	std::vector<PointRef> points;
public:
	TrapezePathFinder(std::vector<Poly> const& polys);
};