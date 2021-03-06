#pragma once
#include <iostream>

struct Point {
	double x, y;
};

double& x(Point& p);
double& y(Point& p);

const double& x(Point const& p);
const double& y(Point const& p);

bool operator==(Point const& p, Point const& q);

bool operator!=(Point const& p, Point const& q);

bool operator<(Point const& p, Point const& q);

Point operator+(Point const& p, Point const& q);

Point operator-(Point const& p, Point const& q);

Point operator*(Point const& p, double f);

double operator*(Point const& p, Point const& q); // scalar product
double crossProdZ(Point const& p, Point const& q); // vector product

std::ostream& operator<< (std::ostream& os, Point const& p);

double norm(Point const& p);

double sqNorm(Point const& p);

double dist(Point const& p, Point const& q);