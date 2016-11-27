#include "cinder/app/App.h"
#include "cinder/app/RendererGl.h"
#include "cinder/gl/gl.h"
#include "Point.h"
#include "geometry.h"
#include "Graph.h"
#include "triangulation.h"
#include <random>

using namespace ci;
using namespace ci::app;
using namespace std::literals;

class BasicApp : public App {
public:
	void setup() override
	{
		auto src = DataSourcePath::create(
			R"(C:\Users\Daniel\Documents\Visual Studio 2015\Projects\PathSearch\BasicApp\fonts\Consolas.ttf)"
		);
		font = ci::Font(src, 18.f);

		gpts = std::vector<Point>{{100,0},{100,80},{-100,80},{-100,0}};
		g = triangulatePolygon(gpts);

		graphEdgesCol = Color("slategray");
		graphVerticesCol = Color("white");
		selectionCol = Color("mediumaquamarine");
		hoveredOn = nullptr;
	}
	void mouseDown(MouseEvent event) override;
	void mouseMove(MouseEvent event) override;
	void keyDown(KeyEvent event) override;
	void draw() override;

private:
	vec2 toVec2(Point const& p)
	{
		return{float(p.x),float(p.y)};
	}
	int height, wid;
	ci::Font font;
	vec2 textPos;
	std::string msg;
	Graph g;
	std::vector<Point> gpts;
	std::vector<Graph::NodesList::iterator> chosen;
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

void BasicApp::mouseDown(MouseEvent event)
{
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
				g.flipNodes(chosen[0], chosen[1]);
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
}

void BasicApp::draw()
{
	height = getWindowHeight();
	wid = getWindowWidth();
	textPos = {-wid*0.5f, -height*0.5f*0.8f};
	gl::pushModelMatrix();

	gl::translate({wid*0.5f,height*0.5f});
	gl::scale({1.f,-1.f});
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
		gl::drawSolidCircle(toVec2(v), 4.f);

	if (hoveredOn != nullptr)
		gl::drawStrokedCircle(toVec2(circum_center), circum_radius);

	{
		gl::pushModelMatrix();
		gl::scale({1,-1});
		gl::drawString(msg, textPos, Color("white"), font);
		gl::popModelMatrix();
	}
	gl::popModelMatrix();
}

CINDER_APP(BasicApp, RendererGl)