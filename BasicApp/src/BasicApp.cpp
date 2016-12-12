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
#include "Search.h"
#include "Tree.h"
#include "PathFinding.h"

using namespace ci;
using namespace ci::app;
using namespace std::literals;
using Node = Graph::Node;

std::vector<Point> mkSceneHull(int n=20)
{
	return rectHull(Rect({500.0,300.0}, {-250.0,-150.0}), n, n);
}

class BasicApp : public App {
public:
	void setup() override
	{
		r = Rect({500.0,300.0}, {-250.0,-150.0});
		auto src = DataSourcePath::create(
			R"(C:\Users\Daniel\Documents\Visual Studio 2015\Projects\PathSearch\BasicApp\fonts\Consolas.ttf)"
		);
		font = ci::Font(src, 18.f);
		graphEdgesCol = Color("slategray");
		graphVerticesCol = Color("white");

		pts = rectHull(r,40,24);
		g = triangulatePolygon(pts);
	}
	void mouseDown(MouseEvent event) override;
	void mouseMove(MouseEvent event) override;
	void keyDown(KeyEvent event) override;
	void mouseWheel(MouseEvent event) override;
	void draw() override;
private:
	vec2 toVec2(Point const& p) { return {float(p.x), float(p.y)}; }
	Rect r;
	Graph g;
	std::vector<Point> pts;
	void grawGraph()
	{
		gl::lineWidth(2.0f);
		for (auto& n : g.nodes)
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
	
	BFSTree bfs;
	void drawBFSTree()
	{
		vec2 tri[3];
		
		gl::color(Color("darkgreen"));
		for (auto node : bfs.opened)
		{
			for (int i=0; i<3; i++) tri[i] = toVec2(pts[node->triangle[i]]);
			gl::drawSolidTriangle(tri);
		}
		
		gl::color(Color("violet"));
		for (auto const& trian : bfs.closed)
		{
			for (int i = 0; i<3; i++) tri[i] = toVec2(pts[trian[i]]);
			gl::drawSolidTriangle(tri);
		}
		if (bfs.opened.empty()) return;
		gl::color(Color("red"));
		for (int i = 0; i<3; i++) tri[i] = toVec2(pts[bfs.head->triangle[i]]);
		gl::drawSolidTriangle(tri);
	}
	void drawQ()
	{
		vec2 tri[3];

		gl::color(Color("red"));
		for (auto node : q)
		{
			for (int i = 0; i<3; i++) tri[i] = toVec2(pts[node->triangle[i]]);
			gl::drawSolidTriangle(tri);
		}
		gl::color(Color("blue"));
		for (auto const& trian : inner)
		{
			for (int i = 0; i<3; i++) tri[i] = toVec2(pts[trian[i]]);
			gl::drawSolidTriangle(tri);
		}
	}
	std::deque<Graph::Node::Ref> q;
	std::set<Triangle> inner;

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
	auto fnd = std::find_if(g.nodes.begin(), g.nodes.end(), [this](Graph::Node const& node) {
		return insideTriangle(pts, node.triangle, mousePos) == Position::in;
	});
	if (fnd == g.nodes.end()) return;
	bfs.setStart(fnd);
}

void BasicApp::keyDown(KeyEvent event)
{
	if (event.getCode() == 't')
		toDelaunay(pts, g);
	else if (event.getCode() == 'r') {
		bfs.setStart(g.nodes.begin());
	}
	else if (event.getCode() == 'b') {
		bfs.step();
	}
	else if (event.getCode() == 'k') {
		auto root = bfs.head;
		while (bfs.lvl < 20)
			bfs.step();
		for (auto node : bfs.opened)
			bfs.closed.insert(node->triangle);
		bfs.opened.clear();

		q.push_back(root);
	}
	else if (event.getCode() == 'g') {
		
		auto in_closed = [this](Graph::Node::Ref neib) {
			return bfs.closed.find(neib->triangle) != bfs.closed.end();
		};
		auto in_inner = [this](Graph::Node::Ref neib) {
			return inner.find(neib->triangle) != inner.end();
		};

		if (!q.empty())
		{
			auto h = q.front(); q.pop_front();
			if (in_inner(h)) return;
			bool all_in = std::all_of(begin(h->refs), end(h->refs), in_closed);
			if (all_in) {
				inner.insert(h->triangle);
				for (auto neib : h->refs) q.push_back(neib);
			}
		}
		else {
			std::set<Triangle> bnd;
			std::set_difference(bfs.closed.begin(), bfs.closed.end(), inner.begin(), inner.end(),
				std::inserter(bnd, bnd.end()));
			bfs.closed = bnd;
			inner.clear();
		}
			
	}
	else if (event.getCode() == 'p') {
		auto ins = rectInsides(r,40,24);
		for (Point const& p : ins)
			addNewPoint(pts, g, p);
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

	drawBFSTree();
	drawQ();

	grawGraph();
	gl::color(graphVerticesCol);
	for (auto& v : pts)
		gl::drawSolidCircle(toVec2(v), 1.0f);

	{
		gl::pushModelMatrix();
		gl::scale(vec2{1,-1} / scaleFac);
		gl::drawString(msg, textPos, Color("white"), font);
		gl::popModelMatrix();
	}
	gl::popModelMatrix();
}

CINDER_APP(BasicApp, RendererGl)