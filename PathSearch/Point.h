#pragma once
#include <iostream>

struct Point {
	double x, y;
};

Point operator+(Point const& p, Point const& q);

Point operator-(Point const& p, Point const& q);

Point operator*(Point const& p, double f);

std::ostream& operator<< (std::ostream& os, Point const& p);

double norm(Point const& p);

double sqNorm(Point const& p);

double dist(Point const& p, Point const& q);