#pragma once
#include "DCEL.h"
#include "geometry.h"
#include <queue>
#include <set>

class Lozalization {
public:
	struct Node {
		Node(DCEL::FaceList::iterator f, Point const& p);
		DCEL::FaceList::iterator face;
		double dist_to_p;
		TriPos pos;
	};
	struct dist_greater_comp {
		bool operator()(Node const& n, Node const& m);
	};
	struct AStarQueue : public std::priority_queue<Node,std::vector<Node>,dist_greater_comp> 
	{
		using std::priority_queue<Node, std::vector<Node>, dist_greater_comp>::priority_queue;
		container_type::iterator begin();
		container_type::iterator end();
	};
	

	Lozalization(DCEL::FaceList::iterator f, Point const& p, 
		DCEL::FaceList::iterator out_face);

	Point const& p;
	Node current;
	AStarQueue queue;
	std::set<Node> closed;
	
	enum State{ done, ignore, in_progress};
	State state;
	DCEL::FaceList::iterator out_face;
	void step();
};

bool operator<(Lozalization::Node const& n, Lozalization::Node const& m);