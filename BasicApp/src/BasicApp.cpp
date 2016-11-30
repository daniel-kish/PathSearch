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
		cur = g.nodes.begin();
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
	Point mousePos;
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
	Node::Ref cur;
	Position searchState{Position::out};
	Point searchedPoint;
	void searchStep()
	{
		if (cur == g.nodes.end() && !g.nodes.empty())
			cur = g.nodes.begin();
		if (cur == g.nodes.end())
			return;
		Position res = insideTriangle(gpts, cur->triangle, searchedPoint);
		if (res == Position::out) {
			double dmin = std::numeric_limits<double>::max();
			Node::Ref trMin = cur;
			for (Node::Ref n : cur->refs)
			{
				Point center = triangleCenter(gpts, n->triangle);
				double d = dist(center, searchedPoint);
				if (d < dmin) {
					dmin = d;
					trMin = n;
				}
			}
			cur = trMin;
		}
		else if (res == Position::in) {
			searchState = Position::in;
		}
		else if (res == Position::edge01 || res == Position::edge12 
			     || res == Position::edge02) 
		{
			searchState = res;
		}
	}
};

void BasicApp::mouseMove(MouseEvent event)
{
	std::ostringstream os;
	auto imsPos = event.getPos();
	mousePos.x = double(imsPos.x);
	mousePos.y = double(imsPos.y);
	mousePos = mousePos - Point{wid*0.5f, height*0.5f};
	mousePos.y *= -1.0;
	mousePos = mousePos * (1.0/scaleFac);
	
	auto pos = std::find_if(g.nodes.begin(), g.nodes.end(), [this](Node const& n) {
		return insideTriangle(gpts, n.triangle, mousePos) == Position::in;
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
	scaleFac *= std::pow(1.05, incr);
	std::ostringstream os;
	os << scaleFac;
	msg = os.str();
}

void BasicApp::mouseDown(MouseEvent event)
{
	if (event.isLeftDown())
	{
		if (selectionMode)
		{
			auto p = g.nodes.begin();
			for (; p != g.nodes.end(); ++p) {
				if (insideTriangle(gpts, p->triangle, Point{double(mousePos.x), double(mousePos.y)}) == Position::in) {
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
			cur = g.nodes.begin();
		}
		else if (!selectionMode && !insertionMode){
			Point p{double(mousePos.x),double(mousePos.y)};
			addNewPoint(gpts, g, p);
			//searchedPoint = mousePos;
		}
	}
}

void BasicApp::keyDown(KeyEvent event)
{
	static double wid = 200.0;
	static double step = 25.0;
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
	else if (event.getCode() == 'h') {
		for (double x = 0.0; x < wid; x += step)
			gpts.push_back(Point{x,0.0});
		for (double y = 0.0; y < wid; y += step)
			gpts.push_back(Point{wid,y});
		for (double x = wid; x > 0.0; x -= step)
			gpts.push_back(Point{x,wid});
		for (double y = wid; y > 0.0; y -= step)
			gpts.push_back(Point{0.0,y});
		g = triangulatePolygon(gpts);
		cur = g.nodes.begin();
	}
	else if (event.getCode() == 'p') {
		for (double x = step; x < wid; x += step)
		{
			for (double y = step; y < wid; y += step)
			{
				addNewPoint(gpts, g, Point{x,y});
			}
			trian();
		}
	}
	else if (event.getCode() == ' ') {
		searchStep();
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
	gl::drawSolidCircle(toVec2(mousePos), 3.f / scaleFac);
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
		gl::drawSolidCircle(toVec2(v), 3.0f);

	if (hoveredOn != nullptr)
		gl::drawStrokedCircle(toVec2(circum_center), circum_radius);

	{
		gl::pushModelMatrix();
		gl::scale(vec2{1,-1}/scaleFac);
		gl::drawString(msg, textPos, Color("white"), font);
		gl::popModelMatrix();
	}
	gl::popModelMatrix();
}

CINDER_APP(BasicApp, RendererGl)