#include "Trapezoid.h"

void TrapezePathFinder::clear()
{
	obstacles.clear();
	points.clear();
	lines.clear();
	line_segments.clear();
}

TrapezePathFinder::TrapezePathFinder()
	: mt{/*std::random_device{}()*/}
{

}


void TrapezePathFinder::mkLines()
{
	std::uniform_real_distribution<double> ud(0.0,1.0);

	for (Poly& poly : obstacles)
	{
		points.insert(points.end(), poly.begin(), poly.end());
	}
	std::sort(points.begin(), points.end());
	minp = points[0].get();

	Point dir{ud(mt),ud(mt)};
	for (PointRef p : points)
		lines.push_back({p,dir});

	for (Poly& poly : obstacles) {
		for (int i = 0; i < poly.size()-1; ++i)
			line_segments.push_back(Line{poly[i], poly[i + 1]});
		line_segments.push_back(Line{poly.back(),poly.front()});
	}
	Point a = r.origin;
	Point b = r.origin + Point{r.dir.x,0.0};
	Point c = b + Point{0.0f,r.dir.y};
	Point d = a + Point{0.0f,r.dir.y};
	line_segments.push_back(Line{a,b});
	line_segments.push_back(Line{b,c});
	line_segments.push_back(Line{c,d});
	line_segments.push_back(Line{d,a});


	for (OriginVec& ov : lines)
	{
		for (Line& l : line_segments)
		{
			double t, u;
			std::tie(t, u) = intersection(ov.origin, ov.dir, l);
			if (t >= 0.0 && t <= 1.0) {
				ov.inters.push_back(u);
			}
		}
	}
	for (OriginVec& ov : lines)
	{
		auto inside_any = [this](Point const& p) {
			return std::any_of(obstacles.begin(), obstacles.end(), [&p](Poly const& poly) {
				return insidePoly(poly, p);
			});
		};
		std::sort(ov.inters.begin(), ov.inters.end());
		auto le = std::unique(ov.inters.begin(), ov.inters.end(), close);
		ov.inters.erase(le, ov.inters.end());
		auto zero_pos = find_if(ov.inters.begin(), ov.inters.end(), zero);
		auto p = zero_pos;
		for (; p != ov.inters.end()-1; ++p)
		{
			Point start = ov(*p);
			Point fin = ov(*(p+1));
			Point mid = (start + fin)*0.5;
			if (inside_any(mid))
				break;
		}
		ov.uRight = *p;

		auto d = std::distance(ov.inters.begin(), zero_pos);
		auto q = ov.inters.rbegin();
		std::advance(q, ov.inters.size() - d - 1);
		for (; q != ov.inters.rend() - 1; ++q)
		{
			Point start = ov(*q);
			Point fin = ov(*(q+1));
			Point mid = (start + fin)*0.5;
			if (inside_any(mid))
				break;
		}
		ov.uLeft = *q;

		double len = dist(ov(ov.uLeft), ov(ov.uRight));
	}
}

Point OriginVec::operator()(double u)
{
	return origin + dir * u;
}
