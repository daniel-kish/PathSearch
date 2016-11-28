#include "cinder/app/App.h"
#include "cinder/app/RendererGl.h"
#include "cinder/gl/gl.h"
#include "Point.h"
#include "geometry.h"
#include "Graph.h"
#include "triangulation.h"
#include <random>
#include <deque>
#include <set>
#define _USE_MATH_DEFINES
#include <cmath>

using namespace ci;
using namespace ci::app;
using namespace std::literals;
using Node = Graph::Node;

double rho(double t)
{
	return 200 + sin(4 * t - M_PI/2.0)*50.0;
}

class BasicApp : public App {
public:
	void setup() override
	{
		auto src = DataSourcePath::create(
			R"(C:\Users\Daniel\Documents\Visual Studio 2015\Projects\PathSearch\BasicApp\fonts\Consolas.ttf)"
		);
		font = ci::Font(src, 18.f);

		graphEdgesCol = Color("slategray");
		graphVerticesCol = Color("white");
		selectionCol = Color("mediumaquamarine");
		hoveredOn = nullptr;
		q.push_back(g.nodes.begin());
		file.open(R"(C:\Users\Daniel\Documents\Visual Studio 2015\Projects\PathSearch\file.txt)");
	}
	void mouseDown(MouseEvent event) override;
	void mouseMove(MouseEvent event) override;
	void keyDown(KeyEvent event) override;
	void mouseWheel(MouseEvent event) override;
	void draw() override;

private:
	vec2 toVec2(Point const& p)
	{
		return{float(p.x),float(p.y)};
	}
	int height, wid;
	float scaleFac{1.0};
	ci::Font font;
	vec2 textPos;
	std::string msg;
	Graph g;
	std::vector<Point> gpts;
	std::vector<Graph::Node::Ref> chosen;
	Color graphEdgesCol;
	Color graphVerticesCol;
	Color selectionCol;
	void grawGraph()
	{
		gl::lineWidth(2.25f);

		gl::color(graphEdgesCol);
		for (auto& n : g.nodes)
		{
			gl::begin(GL_LINE_STRIP);
			gl::vertex(toVec2(gpts[n.triangle[0]]));
			gl::vertex(toVec2(gpts[n.triangle[1]]));
			gl::vertex(toVec2(gpts[n.triangle[2]]));
			gl::vertex(toVec2(gpts[n.triangle[0]]));
			gl::end();
		}
	}
	ivec2 mousePos;
	bool selectionMode{false};
	bool insertionMode{false};
	Point newP;
	Triangle* hoveredOn;
	Point circum_center;
	double circum_radius;
	std::deque<Node::Ref> q;
	std::set<Triangle> closed;
	std::ofstream file;
	void trian() {
		q.clear();
		closed.clear();
		q.push_back(g.nodes.begin());
		while (!q.empty())
		{
			Node::Ref head = q.front();
			q.pop_front();
			auto res = closed.insert(head->triangle);
			if (!res.second)
				continue;

			std::vector<Node::Ref> neibs = head->refs;
			for (Node::Ref neib : neibs)
			{
				Triangle t1 = head->triangle, t2 = neib->triangle;
				Edge e = common_edge(t1, t2);
				if (!localDelaunay(gpts, t1, t2, e)) {
					g.flipNodes(head, neib, e);
					q.push_back(head); q.push_back(neib);
					break;
				}
				if (closed.find(neib->triangle) == closed.end())
					q.push_back(neib);
			}
		}
	}
};

void BasicApp::mouseMove(MouseEvent event)
{
	mousePos = event.getPos();
	mousePos -= vec2{wid*0.5f, height*0.5f};
	mousePos *= vec2{1.f, -1.f};
	
	Point msPt{double(mousePos.x), double(mousePos.y)};
	//if (hoveredOn != nullptr && insideTriangle(gpts, hoveredOn->triangle, msPt))
	//	return;
	
	auto pos = std::find_if(g.nodes.begin(), g.nodes.end(), [this,msPt](Node const& n) {
		return insideTriangle(gpts, n.triangle, msPt);
	});
	if (pos != g.nodes.end())
		hoveredOn = &pos->triangle;
	else
		hoveredOn = nullptr;
	if (hoveredOn != nullptr) {
		circum_center = circumCenter(gpts, *hoveredOn);
		circum_radius = circumRadius(gpts, *hoveredOn);
	}
}

void BasicApp::mouseWheel(MouseEvent event)
{
	auto incr = event.getWheelIncrement();
	scaleFac += incr;
}

void BasicApp::mouseDown(MouseEvent event)
{
	static bool b = true;
	static double rmax = 0.0;
	if (b) {
		for (double t = 0.0; t <= 2.0*M_PI; t += 2.0*M_PI / 100.0)
		{
			Point p;
			double r = rho(t);
			p.x = r*cos(t);
			p.y = r*sin(t);
			gpts.push_back(p);
		}
		g = triangulatePolygon(gpts);
		addNewPoint(gpts, g, {0,0});
		b = false;
	}
	else {
		static double t = 0.0;
		double h = 20.0;
		if (t >= 2.0*M_PI) return;
		Point orth;
		orth.x = cos(t);
		orth.y = sin(t);
		double r = rho(t);
		int m = std::floor(r / h);
		for (int j = 1; j <= m; ++j)
		{
			addNewPoint(gpts, g, orth*h*j);
		}
		trian();
		t += 2.0*M_PI / 100.0;
	}
	return;
	if (event.isLeftDown())
	{
		if (selectionMode)
		{
			auto p = g.nodes.begin();
			for (; p != g.nodes.end(); ++p) {
				if (insideTriangle(gpts, p->triangle, Point{double(mousePos.x), double(mousePos.y)})) {
					break;
				}
			}
			if (p != g.nodes.end()) {
				chosen.push_back(p);
				msg = "select second triangle";
			}
			if (chosen.size() == 2) {
				selectionMode = false;
				msg = "OK. Press 'f' to flip";
			}
		}
		else if (insertionMode) {
			gpts.push_back({double(mousePos.x),double(mousePos.y)});
			g = triangulatePolygon(gpts);
		}
		else if (!selectionMode && !insertionMode){
			Point p{double(mousePos.x),double(mousePos.y)};
			addNewPoint(gpts, g, p);
		}
	}
}

void BasicApp::keyDown(KeyEvent event)
{
	if (event.getCode() == 's') {
		if (chosen.empty()) {
			selectionMode = true;
			msg = "select two triangles";
		}
	}
	else if (event.getCode() == 'f')
	{
		if (!selectionMode && chosen.size() == 2) {
			try {
				hoveredOn = &g.nodes.front().triangle;
				Edge e = common_edge(chosen[0]->triangle, chosen[1]->triangle);
				g.flipNodes(chosen[0], chosen[1], e);
			}
			catch (FlipError& fe) {
				msg = fe.what();
			}
			chosen.clear();
			msg.clear();
		}
	}
	else if (event.getCode() == 'i')
	{
		if (insertionMode) {
			insertionMode = false;
			msg.clear();
		}
		else {
			insertionMode = true;
			msg = "insert a point";
		}
	}
	else if (event.getCode() == 't')
	{
		trian();
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

	gl::drawCoordinateFrame(30.f);
	vec2 tri[3];
	for (auto n : chosen)
	{
		gl::color(selectionCol);
		tri[0] = toVec2(gpts[n->triangle[0]]);
		tri[1] = toVec2(gpts[n->triangle[1]]);
		tri[2] = toVec2(gpts[n->triangle[2]]);
		gl::drawSolidTriangle(tri);
	}
	grawGraph();
	gl::color(graphVerticesCol);
	for (auto& v : gpts)
		gl::drawSolidCircle(toVec2(v), 1.5f);

	if (hoveredOn != nullptr)
		gl::drawStrokedCircle(toVec2(circum_center), circum_radius);

	{
		gl::pushModelMatrix();
		gl::scale(vec2{1,-1}*scaleFac);
		gl::drawString(msg, textPos, Color("white"), font);
		gl::popModelMatrix();
	}
	gl::popModelMatrix();
}

CINDER_APP(BasicApp, RendererGl)