#include "Trapezoid.h"

TrapezePathFinder::TrapezePathFinder(std::vector<Poly> const& polys)
{
	for (Poly const& poly : polys)
	{
		points.insert(points.end(), poly.cbegin(), poly.cend());
	}
}
