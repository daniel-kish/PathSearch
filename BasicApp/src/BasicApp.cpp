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
#include <algorithm>
#include "Kirkpatrick.h"

using namespace ci;
using namespace ci::app;
using namespace std::literals;

std::vector<Point> transform(std::vector<Point>& pts)
{
	std::vector<Point> newpts;
	newpts.reserve(pts.size());

	for (int i = 0; i < pts.size() - 1; ++i)
		newpts.push_back((pts[i] + pts[i + 1])*0.5);
	
	newpts.push_back((pts.front() + pts.back())*0.5);
	return newpts;
}

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
		double d = 800;
		std::vector<Point> poly{{-d,-d},{d,-d},{0,d}};

		dcel = std::make_unique<DCEL>(mk_CCW_poly(poly));
		auto poly_face = dcel->faces.begin();
		dcel->out_face = std::next(dcel->faces.begin());

		root = std::make_unique<treeNode>(poly_face,0);
		poly_face->hist = root.get(); // hand-shaking

		hf = dcel->halfedges.begin();

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
		gl::drawSolidCircle(toVec2(p), rad / scaleFac, 20);
	}
	void drawLine(Point const& p, Point const& q)
	{
		gl::drawLine(toVec2(p), toVec2(q));
	}
	void drawHalfedge(DCEL::Halfedge const& h)
	{
		if (h.target == std::prev(dcel->vertices.end())
			|| h.target == std::prev(dcel->vertices.end(), 2)
			|| h.target == std::prev(dcel->vertices.end(), 3))
			return;
		if (h.twin->target == std::prev(dcel->vertices.end())
			|| h.twin->target == std::prev(dcel->vertices.end(), 2)
			|| h.twin->target == std::prev(dcel->vertices.end(), 3))
			return;
		Point const& target = h.target->p;
		Point const& source = h.prev->target->p;
		Point mid = (source + target)*0.5;
		//setHalfedgeColor(h.i);
		gl::lineWidth(2.0f);
		drawLine(target, mid);
		gl::color(Color("black"));
		drawPoint(target, 3.0f);
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
		gl::color(col.r,col.g,col.b,0.5);
		gl::drawSolid(pl);
	}

	std::unique_ptr<DCEL> dcel;
	std::unique_ptr<treeNode> root;
	std::vector<std::pair<Point, Point>> voronoi_edges;
	std::vector<std::vector<Point>> polys;
	std::vector<Point> out_poly;

	DCEL::EdgeList::iterator hf;
	bool inside;

	std::vector<search_result> found;
	std::vector<Point> centers;

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

	//found = Kirkpatrick_localize(root.get(), mousePos);

	std::tie(hf,inside) = localize(*dcel, mousePos);
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
	insert_point(*dcel, root.get(), mousePos);
}
void BasicApp::keyDown(KeyEvent event)
{
	if (event.getCode() == 'v') {
		auto ins = polarHull([](double F) {
			int N = 3;
			double L = 100.0;
			double val = round(N*F / M_PI);
			double cosv = cos(2*M_PI/N * (0.5*val - round(0.5*val)) - F + M_PI/N*val);
			return L / cosv;
		}, {0,0}, 100);
		std::shuffle(ins.begin(), ins.end(), std::mt19937{});
		for (Point const& p : ins)
			insert_point(*dcel,root.get(), p);
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
	gl::lineWidth(2.0f);
	for (DCEL::Vertex const& v : dcel->vertices)
		drawPoint(v.p,2.5f);
	for (DCEL::Halfedge const& h : dcel->halfedges)
		drawLine(h.target->p, h.twin->target->p);
	//gl::color(Color("orange"));
	//for (DCEL::Face const& f : dcel->faces)
	//	drawPoint(circumCenter(f.halfedge->prev->target->p, f.halfedge->target->p, f.halfedge->next->target->p));


	/*for (DCEL::Face const& f : dcel->faces)
	{
		Point c = facecenter(f);
		if (insidePoly(out_poly, c))
			drawSolidFace(f,Color("green"));
	}*/

	//gl::color(Color("red"));
	//for (Point const& p : centers)
	//	drawPoint(p);

	//gl::lineWidth(1.0f);
	//for (Poly const& poly : polys)
	//{
	//	ci::PolyLine2 pl;
	//	for (Point const& p : poly)
	//		pl.push_back(toVec2(p));
	//	gl::color(0.1f, 0.1f, 0.75f, 0.5f);
	//	gl::drawSolid(pl);
	//}
	//gl::lineWidth(2.0f);
	//{// out_poly
	//	ci::PolyLine2 pl;
	//	for (Point const& p : out_poly)
	//		pl.push_back(toVec2(p));
	//	gl::color(0.2f, 0.2f, 0.2f, 0.2f);
	//	gl::drawSolid(pl);
	//}

	/*if (hf != dcel->halfedges.end()) {
		if (inside)
			drawSolidFace(*hf->face, Color("green"));
		else {
			gl::color(0.0f, 1.0f, 0.0f, 0.5f);
			drawLine(hf->target->p, hf->twin->target->p);
		}
	}*/

	/*gl::color(Color("darkgreen"));
	gl::lineWidth(3.0f);
	for (auto const& edge : voronoi_edges) {
		drawLine(edge.first, edge.second);
	}*/

	{
		gl::pushModelMatrix();
		gl::scale(vec2{1,-1} / scaleFac);
		gl::drawString(msg, textPos, Color("black"), font);
		gl::popModelMatrix();
	}
	gl::popModelMatrix();
}

CINDER_APP(BasicApp, RendererGl)