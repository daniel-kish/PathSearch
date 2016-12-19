#include "cinder/app/App.h"
#include "cinder/app/RendererGl.h"
#include "cinder/gl/gl.h"
#include "Point.h"
#include "geometry.h"
#include "Graph.h"
#include "triangulation.h"
#include <random>
#include <fstream>
#include <deque>
#include <set>
#define _USE_MATH_DEFINES
#include <cmath>
#include "Search.h"
#include "Tree.h"
#include "Obstacles.h"

using namespace ci;
using namespace ci::app;
using namespace std::literals;
using Node = Graph::Node;
using MLine = ::Line;

bool insideRect(Rect const& r, Point const& p)
{
	Point leftBot = r.origin;
	Point rightTop = r.origin + r.dir;

	return p.x > leftBot.x && p.x < rightTop.x && p.y > leftBot.y && p.y < rightTop.y;
}

std::vector<std::vector<Point>> readPoly(std::ifstream& input)
{
	std::vector<std::vector<Point>> polys;

	while (!input.eof())
	{
		polys.push_back(std::vector<Point>{});
		double x, y;
		while (input >> x >> y) {
			polys.back().push_back(Point{x,y});
		}
	}
	return polys;
}

class BasicApp : public App {
public:
	void setup() override
	{
		r = Rect({500.0,300.0}, {-250.0,-150.0});

		grower = std::make_unique<ObstacleGrower>(r, 50, 30, 0.2f);
		auto src = DataSourcePath::create(
			R"(C:\Users\Daniel\Documents\Visual Studio 2015\Projects\PathSearch\BasicApp\fonts\Consolas.ttf)"
		);
		font = ci::Font(src, 18.f);
		graphEdgesCol = Color("slategray");
		graphVerticesCol = Color("white");

		a = toVec2(r.origin);
		b = toVec2(r.origin + Point{r.dir.x,0.0});
		c = b + vec2{0.0f,float(r.dir.y)};
		d = a + vec2{0.0f,float(r.dir.y)};
	}
	void mouseDown(MouseEvent event) override;
	void mouseMove(MouseEvent event) override;
	void keyDown(KeyEvent event) override;
	void mouseWheel(MouseEvent event) override;
	void draw() override;
private:
	vec2 toVec2(Point const& p) { return{float(p.x), float(p.y)}; }
	/*void grawGraph()
	{
		std::vector<Point> const& pts = grower->points();
		gl::lineWidth(2.0f);
		for (auto const& n : grower->graph().nodes)
		{
			gl::color(graphEdgesCol);
			gl::begin(GL_LINE_STRIP);
			gl::vertex(toVec2(pts[n.triangle[0]]));
			gl::vertex(toVec2(pts[n.triangle[1]]));
			gl::vertex(toVec2(pts[n.triangle[2]]));
			gl::vertex(toVec2(pts[n.triangle[0]]));
			gl::end();
		}
	}*/

	/*std::vector<MLine> voronoiLines;
	void mkVoronoi()
	{
		std::deque<Graph::Node::Ref> q{g.nodes.begin()};
		std::set<Graph::Node::Ref> c;

		while (!q.empty())
		{
			auto head = q.front(); q.pop_front();
			auto res = c.insert(head);
			if (!res.second) continue;

			for (auto neib : head->refs)
			{
				auto c = circumCenter(pts, neib->triangle);
				voronoiLines.push_back(MLine{
					circumCenter(pts,head->triangle),
					circumCenter(pts,neib->triangle)
				});
				q.push_back(neib);
			}
		}
	}*/

	Rect r;
	vec2 a, b, c, d;

	std::unique_ptr<ObstacleGrower> grower;

	std::vector<std::vector<Point>> polys;
	void drawPolys()
	{
		for (Poly const& poly : polys) 
			drawPoly(poly);
	}
	void drawPoly(Poly const& poly)
	{
		gl::begin(GL_LINE_STRIP);
		for (const Point& p : poly)
			gl::vertex(toVec2(p));
		gl::vertex(toVec2(poly.front()));
		gl::end();
	}
	//void drawObstacle()
	//{
	//	gl::lineWidth(1.0f);
	//	vec2 tri[3];
	//	for (auto& obs : grower->sets) 
	//	{
	//		for (auto treenode : obs)
	//		{
	//			gl::color(Color("darkturquoise"));
	//			for (int i = 0; i < 3; i++) tri[i] = toVec2(grower->points()[treenode.node->triangle[i]]);
	//			gl::drawSolidTriangle(tri);
	//		}
	//		/*for (auto treenode : obs) 
	//		{
	//			if (treenode.par != obs.end()) {
	//				gl::color(Color("black"));
	//				vec2 cen, parcen;
	//				cen = toVec2(triangleCenter(pts, treenode.node->triangle));
	//				parcen = toVec2(triangleCenter(pts, treenode.par->node->triangle));
	//				gl::drawLine(cen, parcen);
	//				gl::drawSolidCircle(cen, 2.0f);
	//			}
	//			else {
	//				gl::color(Color("green"));
	//				gl::drawSolidCircle(toVec2(triangleCenter(pts, treenode.node->triangle)), 2.0f);
	//			}
	//		}*/
	//	}
	//	for (auto& bnd : grower->boundaries)
	//	{
	//		for (auto node : bnd)
	//		{
	//			gl::color(Color("darkviolet"));
	//			for (int i = 0; i < 3; i++) tri[i] = toVec2(grower->points()[node->triangle[i]]);
	//			gl::drawSolidTriangle(tri);
	//		}
	//	}
	//}

	Point mousePos;
	int height, wid;
	float scaleFac{1.0};
	ci::Font font;
	vec2 textPos;
	std::string msg;
	Color graphEdgesCol;
	Color graphVerticesCol;
};

void BasicApp::mouseMove(MouseEvent event)
{
	auto imsPos = event.getPos();
	mousePos.x = double(imsPos.x);
	mousePos.y = double(imsPos.y);
	mousePos = mousePos - Point{wid*0.5f, height*0.5f};
	mousePos.y *= -1.0;
	mousePos = mousePos * (1.0 / scaleFac);
}

void BasicApp::mouseWheel(MouseEvent event)
{
	auto incr = event.getWheelIncrement();
	scaleFac *= std::pow(1.05, incr);
}

void BasicApp::mouseDown(MouseEvent event)
{
	
}

void BasicApp::keyDown(KeyEvent event)
{
	/*if (event.getCode() == 'o') {
		grower->growObstacle();
	}
	else if (event.getCode() == 'w') {
		if (grower->edge_lists.empty()) return;
		std::ofstream file{
			R"(C:\Users\Daniel\Documents\visual studio 2015\Projects\PathSearch\poly.dat)"
		};
		auto polypts = grower->getPoly(grower->edge_lists.size()-1);
		for (Point p : polypts)
			file << p.x << ' ' << p.y << '\n';
	}*/

	if (event.getCode() == 'o') {
		grower->clear();
		int n = 1;
		while (grower->sets.size() < n)
			grower->growObstacle();
		polys.clear();
		for (int i = 0; i < grower->sets.size(); ++i)
			polys.push_back(grower->getPoly(i));
	}
}

void BasicApp::draw()
{
	height = getWindowHeight();
	wid = getWindowWidth();
	textPos = {-wid*0.5f, -height*0.5f*0.8f};
	gl::pushModelMatrix();

	gl::translate({wid*0.5f,height*0.5f});
	gl::scale(vec2{1.f,-1.f}*scaleFac);
	gl::clear(Color("black"));

	gl::color(Color("white"));
	gl::drawCoordinateFrame(30.f);

	gl::drawLine(a, b); gl::drawLine(b, c);
	gl::drawLine(c, d); gl::drawLine(d, a);

	drawPolys();
	
	//drawObstacle();
	//grawGraph();
	//
	//gl::color(Color("orangered"));
	//gl::lineWidth(2.5f);
	//for (MLine const& l : voronoiLines)
	//{
	//	gl::drawLine(toVec2(l.a), toVec2(l.b));
	//}
	//
	//gl::color(graphVerticesCol);
	//for (auto const& v : grower->points())
	//	gl::drawSolidCircle(toVec2(v), 0.35f);

	{
		gl::pushModelMatrix();
		gl::scale(vec2{1,-1} / scaleFac);
		gl::drawString(msg, textPos, Color("white"), font);
		gl::popModelMatrix();
	}
	gl::popModelMatrix();
}

CINDER_APP(BasicApp, RendererGl)