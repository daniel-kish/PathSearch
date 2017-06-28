
#include "cinder/app/App.h"
#include "cinder/app/RendererGl.h"
#include "cinder/gl/gl.h"
#include "Point.h"
#include "geometry.h"
#include <fstream>
#define _USE_MATH_DEFINES
#include <cmath>
#include "Tree.h"
#include "Obstacles.h"
#include "Trapezoid.h"
#include "triangulation.h"
#include "Pathfinding.h"

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

class BasicApp : public App {
public:
	void setup() override
	{
		r = Rect({200,100});

		//grower = std::make_unique<ObstacleGrower>(r, 50, 30, 0.7f);
		//grower->max_tree_lvl *= 1.5;
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
	/*void drawGrowerTrian()
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
	void drawObstacle()
	{
		gl::lineWidth(2.0f);
		for (Poly& poly : polys)
		{
			ci::PolyLine2f pl;
			for (Point& p : poly)
				pl.push_back(toVec2(p));
			pl.push_back(toVec2(poly.front()));

			gl::color(Color("darkslategray"));
			gl::drawSolid(pl);

			gl::color(Color("black"));
			gl::begin(GL_LINE_STRIP);
			for (Point& p : poly)
				gl::vertex(toVec2(p));
			gl::vertex(toVec2(poly.front()));
			gl::end();
		}
	}
	/*void drawGrowerSets()
	{
		gl::color(Color("darkslategray"));
		for (NodeSet& set : grower->sets)
		{
			vec2 tri[3];
			for (auto node : set)
			{
				for (int i = 0; i < 3; i++) tri[i] = toVec2(grower->points()[node->triangle[i]]);
				gl::drawSolidTriangle(tri);
			}
		}
	}*/

	std::vector<MLine> voronoiLines;
	void mkVoronoi()
	{
		std::deque<Graph::Node::Ref> q{pathfinder->g.nodes.begin()};
		std::set<Graph::Node::Ref> c;

		while (!q.empty())
		{
			auto head = q.front(); q.pop_front();
			auto res = c.insert(head);
			if (!res.second) continue;

			for (auto neib : head->refs)
			{
				auto c = circumCenter(pathfinder->pts, neib->triangle);
				if (pathfinder->in_any_obstacle(c)) continue;
				voronoiLines.push_back(MLine{
					circumCenter(pathfinder->pts, head->triangle),
					c
				});
				q.push_back(neib);
			}
		}
	}

	Rect r;
	vec2 a, b, c, d;

	void drawPoint(Point const& p, float rad = 1.5f) {
		gl::drawSolidCircle(toVec2(p), rad/scaleFac);
	}
	void drawLine(Point& p, Point& q)
	{
		gl::drawLine(toVec2(p), toVec2(q));
	}
	
	//std::unique_ptr<ObstacleGrower> grower;

	std::vector<std::vector<Point>> polys;

	std::unique_ptr<DelaunayPathFinder> pathfinder;
	void drawTrian()
	{
		if (!pathfinder) return;
		const std::vector<Point>& pts = pathfinder->pts;
		const Graph& g = pathfinder->g;
		gl::lineWidth(1.5f);
		for (auto const& n : g.nodes)
		{
			gl::color(graphEdgesCol);
			gl::begin(GL_LINE_STRIP);
			gl::vertex(toVec2(pts[n.triangle[0]]));
			gl::vertex(toVec2(pts[n.triangle[1]]));
			gl::vertex(toVec2(pts[n.triangle[2]]));
			gl::vertex(toVec2(pts[n.triangle[0]]));
			gl::end();
		}
	}


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
	
	//std::ostringstream os;
	//os << mousePos;
	//msg = os.str();
}

void BasicApp::mouseWheel(MouseEvent event)
{
	auto incr = event.getWheelIncrement();
	scaleFac *= std::pow(1.05, incr);
}

void BasicApp::mouseDown(MouseEvent event)
{
	if (!pathfinder) return;
	bool res = pathfinder->set_point(mousePos);

	msg = res ? "enter another point" : "you've entered a wrong point";
}

void BasicApp::keyDown(KeyEvent event)
{
	/*if (event.getCode() == 'o') {
		grower->growObstacle();
		std::ostringstream os; os << grower->density();
		msg = os.str();
		
		polys.clear();
		for (int i = 0; i < grower->sets.size(); ++i)
			polys.push_back(grower->getPoly(i));
	}*/
	if (event.getCode() == 'w') {
		/*std::ofstream os(
		R"(C:\Users\Daniel\Documents\visual studio 2015\Projects\PathSearch\poly2.dat)"
		);
		for (Poly& poly : polys)
		{
			os << poly.size() << '\n';
			for (Point& p : poly)
				os << p.x << ' ' << p.y << '\n';
			os << '\n';
		}*/
		std::ofstream os(
			R"(C:\Users\Daniel\Documents\visual studio 2015\Projects\PathSearch\poly2.dat)"
		);
		writePoly(os, polys);
	}
	if (event.getCode() == 'r') {
		std::ifstream is(
			R"(C:\Users\Daniel\Documents\Visual Studio 2015\Projects\PathSearch\Release\file.txt)"
		);
		polys = readPoly(is);
	}
	if (event.getCode() == 't') {
		if (!pathfinder) {
			pathfinder = std::make_unique<DelaunayPathFinder>(r, polys);
		}
	}
	if (event.getCode() == 'v')
		mkVoronoi();
	if (event.getCode() == 'f') {
		pathfinder->find_path();
		if (pathfinder->path.empty())
			msg = "no path!";
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
	gl::clear(Color("white"));

	gl::color(Color("black"));
	gl::drawLine(a, b); gl::drawLine(b, c);
	gl::drawLine(c, d); gl::drawLine(d, a);

	drawObstacle();

	//drawTrian();
	//drawGrowerSets();
	//drawGrowerTrian();

	gl::color(Color("red"));
	for (MLine& l : voronoiLines)
		drawLine(l.a,l.b);

	vec2 tri[3];
	if (pathfinder) {
		for (int i = 0; i < pathfinder->src_trg.size(); ++i) {
			if (!i) gl::color(Color("red"));
			else gl::color(Color("blue"));
			drawPoint(pathfinder->src_trg[i], 2.5f);

			if (!i) gl::color(1.0f, 0.0f, 0.0f, 0.3f);
			else gl::color(0.0f, 0.0f, 1.0f, 0.3f);
			
			for (int j = 0; j < 3; j++) 
				tri[j] = toVec2(pathfinder->pts[pathfinder->tri_src_trg[i]->triangle[j]]);
			gl::drawSolidTriangle(tri);
		}
		gl::color(0.0f, 1.0f, 0.0f, 0.5f);
		for (auto node : pathfinder->path)
		{
			for (int j = 0; j < 3; j++)
				tri[j] = toVec2(pathfinder->pts[node->triangle[j]]);
			gl::drawSolidTriangle(tri);
		}
		gl::color(Color("orangered"));
		gl::lineWidth(2.0f);
		if (!pathfinder->path_points.empty()) {
			for (int i = 0; i < pathfinder->path_points.size() - 1; ++i)
			{
				drawLine(pathfinder->path_points[i], pathfinder->path_points[i + 1]);
				drawPoint(pathfinder->path_points[i],1.0f);
			}
		}
	}
	{
		gl::pushModelMatrix();
		gl::scale(vec2{1,-1} / scaleFac);
		gl::drawString(msg, textPos, Color("black"), font);
		gl::popModelMatrix();
	}
	gl::popModelMatrix();
}

CINDER_APP(BasicApp, RendererGl)