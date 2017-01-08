#include "Kirkpatrick.h"
#include "Delaunay.h"
#include <stack>
#include <queue>
#include <cassert>
#include <iomanip>

Point const& point(triangle_vertices const& tp, int i)
{
	switch (i)
	{
	case 0: return std::get<0>(tp)->p;
	case 1: return std::get<1>(tp)->p;
	default: return std::get<2>(tp)->p;
	}
}

print_visitor::print_visitor(std::ostream & os_) : os{os_}
{}

void print_visitor::operator()(DCEL::FaceList::iterator f) const {
	os << f->i;
}

void print_visitor::operator()(triangle_vertices const & vs) const {
	os << point(vs, 0) << ';' << point(vs, 1) << ';' << point(vs, 2);
}

std::ostream& operator<< (std::ostream& os, const treeNode* tn)
{
	struct stack_element {
		const treeNode* node_ptr;
		int lvl;
	};
	std::stack<stack_element> q;
	q.push({tn,0});
	while (!q.empty())
	{
		stack_element n = q.top(); q.pop();
		os << std::string(n.lvl, '*');
		boost::apply_visitor(print_visitor(os), n.node_ptr->face_data);
		os << ' ' << n.node_ptr->successors.size() << '\n';
		for (auto succ_iter = n.node_ptr->successors.rbegin();
			succ_iter != n.node_ptr->successors.rend(); ++succ_iter) {
			q.push({succ_iter->get(),n.lvl + 1});
		}
	}
	return os;
}

point_inside_visitor::point_inside_visitor(Point const & p_, Position & pos_)
	: p{p_}, pos{pos_}
{}

void point_inside_visitor::operator()(DCEL::FaceList::iterator f) const {
	auto h = f->halfedge;
	Point const& a = h->prev->target->p;
	Point const& b = h->target->p;
	Point const& c = h->next->target->p;
	pos = insideTriangle(a, b, c, p);
}

void point_inside_visitor::operator()(triangle_vertices const & vs) const {
	Point const& a = point(vs, 0);
	Point const& b = point(vs, 1);
	Point const& c = point(vs, 2);
	pos = insideTriangle(a, b, c, p);
}

treeNode::treeNode(DCEL::FaceList::iterator f) : face_data(f)
{}

std::vector<search_result> Kirkpatrick_localize(treeNode* root, Point const& point)
{
	{  // preconditions
		Position root_pos;
		boost::apply_visitor(point_inside_visitor{point,root_pos}, root->face_data);
		assert(root_pos != Position::out);
	}

	std::vector<search_result> stack{{root,Position::in}};
	std::vector<search_result> found{};
	while (!stack.empty())
	{
		// assert: point is inside current->face_data
		search_result current = stack.back(); stack.pop_back();
		if (current.node->successors.empty())
			found.push_back(current);
		for (auto const& succ : current.node->successors) {
			Position pos;
			boost::apply_visitor(point_inside_visitor{point,pos}, succ->face_data);
			if (pos != Position::out)
				stack.push_back({succ.get(),pos});
		}
	}
	return found;
}

DCEL::EdgeList::iterator flip_history(DCEL& d, DCEL::EdgeList::iterator h1)
{
	auto h2 = h1->twin;
	auto f1 = h1->face;
	auto f2 = h2->face;

	treeNode* hist1 = f1->hist;
	assert(hist1->successors.empty());
	triangle_vertices vs1 = {h1->prev->target,h1->target,h1->next->target};
	hist1->face_data = vs1;

	treeNode* hist2 = f2->hist;
	assert(hist2->successors.empty());
	triangle_vertices vs2 = {h2->prev->target,h2->target,h2->next->target};
	hist2->face_data = vs2;

	h1 = flip(d, h1);

	auto f1_new = h1->face;
	auto f2_new = h1->twin->face;

	hist1->successors.push_back(std::make_shared<treeNode>(f1_new));
	hist1->successors.push_back(std::make_shared<treeNode>(f2_new));
	hist2->successors.push_back(hist1->successors[0]);
	hist2->successors.push_back(hist1->successors[1]);

	f1_new->hist = hist1->successors[0].get();
	f2_new->hist = hist1->successors[1].get();

	return h1;
}

void insert_point_inside_history(DCEL& d, treeNode* node, Point const& point)
{
	auto h = boost::get<DCEL::FaceList::iterator>(node->face_data)->halfedge;
	triangle_vertices vs = {h->prev->target,h->target,h->next->target};
	node->face_data = vs;

	insert_point_inside(d, h, point);

	node->successors.push_back(std::make_shared<treeNode>(h->face));
	node->successors.push_back(std::make_shared<treeNode>(h->prev->twin->face));
	node->successors.push_back(std::make_shared<treeNode>(h->next->twin->face));
	h->face->hist = node->successors[0].get();
	h->prev->twin->face->hist = node->successors[1].get();
	h->next->twin->face->hist = node->successors[2].get();
}

DCEL::EdgeList::iterator insert_point_on_edge_history(DCEL& d, std::vector<search_result> found, Point const& point)
{
	if (found.size() != 2)
	{
		std::cout << std::setprecision(12) << '\n';
		std::cout << point << '\n';
		std::cout << "oy\n";
		for (search_result const& s : found) {
			std::cout << s.pos << ' ';
			auto h = boost::get<DCEL::FaceList::iterator>(s.node->face_data)->halfedge;
			std::cout << '{' << h->prev->target->p << ',' << h->target->p
				<< ',' << h->next->target->p << '}' << '\n';
		}
	}
	assert(found[0].pos != Position::in || found[0].pos != Position::out);
	assert(found[1].pos != Position::in || found[1].pos != Position::out);

	auto f1 = boost::get<DCEL::FaceList::iterator>(found[0].node->face_data);
	auto f2 = boost::get<DCEL::FaceList::iterator>(found[1].node->face_data);
	
	auto h = f1->halfedge;
	bool are_neibs{false};
	do {
		if (h->twin->face == f2) {
			are_neibs = true;
			break;
		}
		h = h->next;
	} while (h != f1->halfedge);
	if (!are_neibs) {
		std::cerr << "\tSOMETHING WENT WRONG!!!!\n";
		std::exit(1);
	}

	treeNode* f1_hist = f1->hist;
	triangle_vertices vs1 = {h->prev->target,h->target,h->next->target};
	f1_hist->face_data = vs1;
	
	auto t = h->twin;
	treeNode* f2_hist = f2->hist;
	triangle_vertices vs2 = {t->prev->target,t->target,t->next->target};
	f2_hist->face_data = vs2;

	insert_point_on_edge(d, h, point);

	auto f5 = h->face;
	auto f3 = h->next->twin->face;

	auto f4 = t->face;
	auto f6 = t->next->twin->face;

	f1_hist->successors.push_back(std::make_shared<treeNode>(f5));
	f5->hist = f1_hist->successors[0].get();
	f1_hist->successors.push_back(std::make_shared<treeNode>(f3));
	f3->hist = f1_hist->successors[1].get();

	f2_hist->successors.push_back(std::make_shared<treeNode>(f4));
	f4->hist = f2_hist->successors[0].get();
	f2_hist->successors.push_back(std::make_shared<treeNode>(f6));
	f6->hist = f2_hist->successors[1].get();

	return h->prev;
}

void insert_point(DCEL& d, treeNode* root, Point const& point)
{
	std::queue<DCEL::EdgeList::iterator> q;

	auto found = Kirkpatrick_localize(root, point);

	std::sort(found.begin(), found.end(), [](search_result const& s1, search_result const& s2)
	{
		return s1.node < s2.node;
	});
	auto le = std::unique(found.begin(), found.end(),[](search_result const& s1, search_result const& s2)
	{
		return s1.node == s2.node;
	});
	found.erase(le, found.end());

	if (found.size() == 0) return;
	else if (found.size() == 1 
		|| found.size() == 2 && found[0].node == found[1].node) 
	{
		treeNode* to_change = found[0].node; // face to change
		DCEL::EdgeList::iterator h;

		h = boost::get<DCEL::FaceList::iterator>(to_change->face_data)->halfedge;
		insert_point_inside_history(d, to_change, point);

		q.push(h);
		h = h->next->twin->next;
		q.push(h);
		h = h->next->twin->next;
		q.push(h);
	}
	else {
		/*std::cerr << found.size() << ": ";
		for (search_result const& res : found) {
			auto h = boost::get<DCEL::FaceList::iterator>(res.node->face_data)->halfedge;
			std::cerr << h->face->i <<":\n\t" << h->prev->target->p << ' ' << h->target->p << ' '
				<< h->next->target->p << ", "
				<< ' ' << res.pos << ' ';
		}
		
		auto f1 = boost::get<DCEL::FaceList::iterator>(found[0].node->face_data);
		auto f2 = boost::get<DCEL::FaceList::iterator>(found[1].node->face_data);
		bool are_neibs{false};
		auto h = f1->halfedge;
		do {
			if (h->twin->face == f2) {
				are_neibs = true;
				break;
			}
			h = h->next;
		} while (h != f1->halfedge);
		if (!are_neibs)	std::cerr << " not neibs";
		else std::cerr << " neibs";
		std::cerr << '\n';*/
		auto h = insert_point_on_edge_history(d, found, point);
		q.push(h);
		h = h->next->twin->next;
		q.push(h);
		h = h->next->twin->next;
		q.push(h);
		h = h->next->twin->next;
		q.push(h);
	}
	while (!q.empty())
	{
		auto h = q.front(); q.pop();
		if (h->twin->face == d.out_face) continue;
		if (!localDelaunay(d, h))
		{
			h = flip_history(d, h);
			q.push(h->prev);
			q.push(h->twin->next);
		}
	}
}