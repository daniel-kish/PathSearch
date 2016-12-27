#include "cinder/app/App.h"
#include "cinder/app/RendererGl.h"
#include "cinder/gl/gl.h"
#include "Point.h"
#include <fstream>
#define _USE_MATH_DEFINES
#include <cmath>
#include "DCEL.h"
#include <random>

using namespace ci;
using namespace ci::app;
using namespace std::literals;


class BasicApp : public App {
public:
	void setup() override
	{
		dcel = std::make_unique<DCEL>(Point{100,100}, Point{0,0});

		dcel->add_vertex({300,0}, dcel->halfedges.begin());

		auto last = std::prev(dcel->halfedges.end());

		dcel->split_face(last, dcel->vertices.begin());
		dcel->split_edge(last, {50,50});

		last = last->next;
		Point const& a = last->target->p;
		Point const& b = last->next->target->p;
		Point const& c = last->next->next->target->p;
		Point center = (a + b + c)*0.33333f;
		dcel->add_vertex(center, last);
		auto v = last->next->target;
		dcel->split_face(last->prev->prev, v);

		last = last->next->twin->next;

		dcel->split_face(last, v);
		cur = dcel->halfedges.begin();
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
	

	void setHalfedgeColor(int i)
	{
		std::mt19937 mt(i);
		std::uniform_real_distribution<float> d(0.0f, 1.0f);
		int max = dcel->halfedges.size()+1;
		float f = float(i) / float(max);
		gl::color(d(mt), d(mt), d(mt));
	}
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
		setHalfedgeColor(h.i);
		gl::lineWidth(2.0f);
		drawLine(target, mid);
		gl::color(Color("black"));
		drawPoint(target,3.0f);
	}
	void drawArrow(Point const& p, Point const& q)
	{

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
	void drawFace(DCEL::Face const& f)
	{
		std::ostringstream os; os << 'f' << f.i;
		gl::drawString(os.str(), toVec2(facecenter(f)), Color("black"), font);
	}
	std::unique_ptr<DCEL> dcel;
	DCEL::EdgeList::iterator cur;

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

	std::ostringstream os;
	os << scaleFac;
	msg = os.str();
}

void BasicApp::mouseDown(MouseEvent event)
{
}

void BasicApp::keyDown(KeyEvent event)
{
	if (event.getCode() == 'n')
		cur = cur->next;
	if (event.getCode() == 'p')
		cur = cur->prev;
	if (event.getCode() == 't')
		cur = cur->twin;
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
	
	{
		gl::pushModelMatrix();
		gl::scale(vec2{1,-1} / scaleFac);
		gl::drawString(msg, textPos, Color("black"), font);
		gl::popModelMatrix();
	}
	gl::popModelMatrix();
}

CINDER_APP(BasicApp, RendererGl)