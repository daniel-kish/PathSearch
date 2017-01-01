#include "Localization.h"

Lozalization::Node::Node(DCEL::FaceList::iterator _f, Point const & p)
	: face{_f}
{
	auto h = face->halfedge;
	Point const& a = h->prev->target->p;
	Point const& b = h->target->p;
	Point const& c = h->next->target->p;
	Point closest;
	std::tie(closest, pos) = ClosestPointOnTriangle(p, a, b, c);
	dist_to_p = dist(closest, p);
}



Lozalization::AStarQueue::container_type::iterator 
Lozalization::AStarQueue::begin()
{
	return c.begin();
}

Lozalization::AStarQueue::container_type::iterator 
Lozalization::AStarQueue::end()
{
	return c.end();
}

bool Lozalization::dist_greater_comp::operator()(Node const & n, Node const & m)
{
	return n.dist_to_p > m.dist_to_p;
}

Lozalization::Lozalization(DCEL::FaceList::iterator f, Point const& _p,
	DCEL::FaceList::iterator _out_face)
	: p{_p}, current(f, p), queue(dist_greater_comp{}), closed{}, state{in_progress}, out_face{_out_face}
{
	queue.push(current);
}

void Lozalization::step()
{
	if (state != in_progress) 
		return;
	if (queue.empty()) {
		state = done;
		return;
	}
	current = queue.top();
	queue.pop();

	auto res = closed.insert(current);
	if (!res.second) return;

	if (zero(current.dist_to_p)) {
		if (current.pos == TriPos::V0 || current.pos == TriPos::V1
			|| current.pos == TriPos::V2) { // on a vertex
			state = ignore;
			return;
		}
		state = done;
		return;
	}
	auto h = current.face->halfedge;
	auto i{h};
	do {
		auto fneib = i->twin->face;
		if (fneib != out_face) queue.push(Node(fneib,p));
		i = i->next;
	} while (i != h);
}

bool operator<(Lozalization::Node const& n, Lozalization::Node const& m)
{
	return n.face->i < m.face->i;
}