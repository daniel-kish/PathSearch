#include "cinder/app/App.h"
#include "cinder/app/RendererGl.h"
#include "cinder/gl/gl.h"
#include "Point.h"
#include <fstream>
#define _USE_MATH_DEFINES
#include <cmath>
#include "DCEL.h"
#include <random>
#include "geometry.h"
#include "Delaunay.h"
#include "Localization.h"

using namespace ci;
using namespace ci::app;
using namespace std::literals;

std::vector<Poly> readPoly(std::ifstream& input)
{
	std::vector<std::vector<Point>> polys;

	while (!input.eof())
	{
		Poly poly;
		double x, y;
		int sz{0};
		input >> sz;
		poly.reserve(sz);

		while (sz--) {
			input >> x >> y;
			poly.push_back({x,y});
		}
		if (!poly.empty()) polys.push_back(poly);
	}
	return polys;
}


class BasicApp : public App {
public:
	void setup() override
	{
		//std::ifstream is(R"(C:\Users\Daniel\Documents\Visual Studio 2015\Projects\PathSearch\poly1.dat)");
		//std::vector<Point> poly = readPoly(is)[8];
		//std::vector<Point> poly = circleHull(Circle{{0,0},100}, 10);
		std::vector<Point> poly = rectHull(Rect{{500,300},{-250,-150}}, 50, 30);

		dcel = std::make_unique<DCEL>(mk_CCW_poly(poly));
		auto poly_face = std::next(dcel->faces.begin());
		dcel->out_face = dcel->faces.begin();
		cur = dcel->halfedges.begin();
		//polygon_delaunay_triangulation(*dcel, poly_face);

		found = cur;

		auto src = DataSourcePath::create(
			R"(C:\Users\Daniel\Documents\Visual Studio 2015\Projects\PathSearch\BasicApp\fonts\Consolas.ttf)"
		);
		font = ci::Font(src, 18.f);
		graphEdgesCol = Color("slategray");
		graphVerticesCol = Color("white");
	}
	void mouseDown(MouseEvent event) override;
	void mouseMove(MouseEvent event) override;
	void keyDown(KeyEvent event) override;
	void mouseWheel(MouseEvent event) override;
	void draw() override;
private:
	vec2 toVec2(Point const& p) { return{float(p.x), float(p.y)}; }
	void drawPoint(Point const& p, float rad = 1.5f) {
		gl::drawSolidCircle(toVec2(p), rad/scaleFac, 20);
	}
	void drawLine(Point const& p, Point const& q)
	{
		gl::drawLine(toVec2(p), toVec2(q));
	}
	void drawHalfedge(DCEL::Halfedge const& h)
	{
		Point const& target = h.target->p;
		Point const& source = h.prev->target->p;
		Point mid = (source + target)*0.5;
		//setHalfedgeColor(h.i);
		gl::lineWidth(2.0f);
		drawLine(target, mid);
		gl::color(Color("black"));
		drawPoint(target,3.0f);
	}

	Point facecenter(DCEL::Face const& f)
	{
		auto h = f.halfedge;
		auto i = h;
		Point sum{0,0};
		int n = 0;
		while (true)
		{
			sum = sum + i->target->p;
			n++;
			i = i->next;
			if (i == h) break;
		}
		return sum * (1.0 / double(n));
	}
	void drawSolidFace(DCEL::Face const& f)
	{
		auto h = f.halfedge;
		auto i = h;
		
		ci::PolyLine2 pl;
		
		while (true) 
		{
			pl.push_back(toVec2(i->target->p));
			i = i->next;
			if (i == h) break;
		}
		gl::color(0.5f, 0.5f, 0.5f, 0.5f);
		gl::drawSolid(pl);
	}
	void drawSolidFace(DCEL::Face const& f, Color const& col)
	{
		auto h = f.halfedge;
		auto i = h;

		ci::PolyLine2 pl;

		while (true)
		{
			pl.push_back(toVec2(i->target->p));
			i = i->next;
			if (i == h) break;
		}
		gl::color(col);
		gl::drawSolid(pl);
	}

	std::unique_ptr<DCEL> dcel;
	DCEL::EdgeList::iterator cur;

	DCEL::EdgeList::iterator found;

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

	bool b;
	std::tie(found,b) = localize(*dcel, mousePos);
}

void BasicApp::mouseWheel(MouseEvent event)
{
	auto incr = event.getWheelIncrement();
	scaleFac *= std::pow(1.05, incr);

	std::ostringstream os;
	os << scaleFac;
	msg = os.str();
}

void BasicApp::mouseDown(MouseEvent event)
{
	insert_point(*dcel, mousePos);
}

void BasicApp::keyDown(KeyEvent event)
{
	if (event.getCode() == 'n')
		cur = cur->next;
	if (event.getCode() == 'p')
		cur = cur->prev;
	if (event.getCode() == 't')
		cur = cur->twin;
	if (event.getCode() == 'c') {
		cur = clip_delaunay_ear(*dcel, cur);
	}
	if (event.getCode() == 'a') {
		Point mid = (cur->target->p + cur->prev->target->p)*0.5;
		insert_point(*dcel, mid);
		cur = dcel->halfedges.begin();
	}
	if (event.getCode() == 'f') {
		cur = flip(*dcel, cur);
	}
	if (event.getCode() == 'd') {
		polygon_delaunay_triangulation(*dcel, cur->face);
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
	
	if (dcel) {
		for (DCEL::Halfedge const& h : dcel->halfedges)
			drawHalfedge(h);
		drawSolidFace(*cur->face);
		gl::color(Color("red"));
		gl::lineWidth(2.0f);
		gl::drawVector({cur->prev->target->p.x,cur->prev->target->p.y,0.0},
		{cur->target->p.x,cur->target->p.y,0.0}, 25.0,4.0f);
	}
	drawPoint(mousePos, 2.5f);
	if (found != dcel->halfedges.end())
		drawSolidFace(*found->face, Color("green"));
	{
		gl::pushModelMatrix();
		gl::scale(vec2{1,-1} / scaleFac);
		gl::drawString(msg, textPos, Color("black"), font);
		gl::popModelMatrix();
	}
	gl::popModelMatrix();
}

CINDER_APP(BasicApp, RendererGl)