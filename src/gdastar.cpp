#include "a_star.hpp"

#include "core/math/geometry_3d.h"
#include "core/object/script_language.h"

int64_t GDAstar3D::get_available_point_id() const {
	if (points.has(last_free_id)) {
		int64_t cur_new_id = last_free_id + 1;
		while (points.has(cur_new_id)) {
			cur_new_id++;
		}
		const_cast<int64_t&>(last_free_id) = cur_new_id;
	}

	return last_free_id;
}

void GDAstar3D::add_point(int64_t p_id, const Vector3& p_pos, real_t p_weight_scale) {
	ERR_FAIL_COND_MSG(p_id < 0, vformat("Can't add a point with negative id: %d.", p_id));
	ERR_FAIL_COND_MSG(p_weight_scale < 0.0, vformat("Can't add a point with weight scale less than 0.0: %f.", p_weight_scale));

	Point* found_pt;
	bool p_exists = points.lookup(p_id, found_pt);

	if (!p_exists) {
		Point* pt = memnew(Point);
		pt->id = p_id;
		pt->pos = p_pos;
		pt->weight_scale = p_weight_scale;
		pt->prev_point = nullptr;
		pt->open_pass = 0;
		pt->closed_pass = 0;
		pt->enabled = true;
		points.set(p_id, pt);
	}
	else {
		found_pt->pos = p_pos;
		found_pt->weight_scale = p_weight_scale;
	}
}

Vector3 GDAstar3D::get_point_position(int64_t p_id) const {
	Point* p;
	bool p_exists = points.lookup(p_id, p);
	ERR_FAIL_COND_V_MSG(!p_exists, Vector3(), vformat("Can't get point's position. Point with id: %d doesn't exist.", p_id));

	return p->pos;
}

void GDAstar3D::set_point_position(int64_t p_id, const Vector3& p_pos) {
	Point* p;
	bool p_exists = points.lookup(p_id, p);
	ERR_FAIL_COND_MSG(!p_exists, vformat("Can't set point's position. Point with id: %d doesn't exist.", p_id));

	p->pos = p_pos;
}

real_t GDAstar3D::get_point_weight_scale(int64_t p_id) const {
	Point* p;
	bool p_exists = points.lookup(p_id, p);
	ERR_FAIL_COND_V_MSG(!p_exists, 0, vformat("Can't get point's weight scale. Point with id: %d doesn't exist.", p_id));

	return p->weight_scale;
}

void GDAstar3D::set_point_weight_scale(int64_t p_id, real_t p_weight_scale) {
	Point* p;
	bool p_exists = points.lookup(p_id, p);
	ERR_FAIL_COND_MSG(!p_exists, vformat("Can't set point's weight scale. Point with id: %d doesn't exist.", p_id));
	ERR_FAIL_COND_MSG(p_weight_scale < 0.0, vformat("Can't set point's weight scale less than 0.0: %f.", p_weight_scale));

	p->weight_scale = p_weight_scale;
}

void GDAstar3D::remove_point(int64_t p_id) {
	Point* p;
	bool p_exists = points.lookup(p_id, p);
	ERR_FAIL_COND_MSG(!p_exists, vformat("Can't remove point. Point with id: %d doesn't exist.", p_id));

	for (OAHashMap<int64_t, Point*>::Iterator it = p->neighbors.iter(); it.valid; it = p->neighbors.next_iter(it)) {
		Segment s(p_id, (*it.key));
		segments.erase(s);

		(*it.value)->neighbors.remove(p->id);
		(*it.value)->unlinked_neighbours.remove(p->id);
	}

	for (OAHashMap<int64_t, Point*>::Iterator it = p->unlinked_neighbours.iter(); it.valid; it = p->unlinked_neighbours.next_iter(it)) {
		Segment s(p_id, (*it.key));
		segments.erase(s);

		(*it.value)->neighbors.remove(p->id);
		(*it.value)->unlinked_neighbours.remove(p->id);
	}

	memdelete(p);
	points.remove(p_id);
	last_free_id = p_id;
}

void GDAstar3D::connect_points(int64_t p_id, int64_t p_with_id, bool bidirectional) {
	ERR_FAIL_COND_MSG(p_id == p_with_id, vformat("Can't connect point with id: %d to itself.", p_id));

	Point* a;
	bool from_exists = points.lookup(p_id, a);
	ERR_FAIL_COND_MSG(!from_exists, vformat("Can't connect points. Point with id: %d doesn't exist.", p_id));

	Point* b;
	bool to_exists = points.lookup(p_with_id, b);
	ERR_FAIL_COND_MSG(!to_exists, vformat("Can't connect points. Point with id: %d doesn't exist.", p_with_id));

	a->neighbors.set(b->id, b);

	if (bidirectional) {
		b->neighbors.set(a->id, a);
	}
	else {
		b->unlinked_neighbours.set(a->id, a);
	}

	Segment s(p_id, p_with_id);
	if (bidirectional) {
		s.direction = Segment::BIDIRECTIONAL;
	}

	HashSet<Segment, Segment>::Iterator element = segments.find(s);
	if (element) {
		s.direction |= element->direction;
		if (s.direction == Segment::BIDIRECTIONAL) {
			// Both are neighbors of each other now
			a->unlinked_neighbours.remove(b->id);
			b->unlinked_neighbours.remove(a->id);
		}
		segments.remove(element);
	}

	segments.insert(s);
}

void GDAstar3D::disconnect_points(int64_t p_id, int64_t p_with_id, bool bidirectional) {
	Point* a;
	bool a_exists = points.lookup(p_id, a);
	ERR_FAIL_COND_MSG(!a_exists, vformat("Can't disconnect points. Point with id: %d doesn't exist.", p_id));

	Point* b;
	bool b_exists = points.lookup(p_with_id, b);
	ERR_FAIL_COND_MSG(!b_exists, vformat("Can't disconnect points. Point with id: %d doesn't exist.", p_with_id));

	Segment s(p_id, p_with_id);
	int remove_direction = bidirectional ? (int)Segment::BIDIRECTIONAL : (int)s.direction;

	HashSet<Segment, Segment>::Iterator element = segments.find(s);
	if (element) {
		// s is the new segment
		// Erase the directions to be removed
		s.direction = (element->direction & ~remove_direction);

		a->neighbors.remove(b->id);
		if (bidirectional) {
			b->neighbors.remove(a->id);
			if (element->direction != Segment::BIDIRECTIONAL) {
				a->unlinked_neighbours.remove(b->id);
				b->unlinked_neighbours.remove(a->id);
			}
		}
		else {
			if (s.direction == Segment::NONE) {
				b->unlinked_neighbours.remove(a->id);
			}
			else {
				a->unlinked_neighbours.set(b->id, b);
			}
		}

		segments.remove(element);
		if (s.direction != Segment::NONE) {
			segments.insert(s);
		}
	}
}

bool GDAstar3D::has_point(int64_t p_id) const {
	return points.has(p_id);
}

PackedInt64Array GDAstar3D::get_point_ids() {
	PackedInt64Array point_list;

	for (OAHashMap<int64_t, Point*>::Iterator it = points.iter(); it.valid; it = points.next_iter(it)) {
		point_list.push_back(*(it.key));
	}

	return point_list;
}

Vector<int64_t> GDAstar3D::get_point_connections(int64_t p_id) {
	Point* p;
	bool p_exists = points.lookup(p_id, p);
	ERR_FAIL_COND_V_MSG(!p_exists, Vector<int64_t>(), vformat("Can't get point's connections. Point with id: %d doesn't exist.", p_id));

	Vector<int64_t> point_list;

	for (OAHashMap<int64_t, Point*>::Iterator it = p->neighbors.iter(); it.valid; it = p->neighbors.next_iter(it)) {
		point_list.push_back((*it.key));
	}

	return point_list;
}

bool GDAstar3D::are_points_connected(int64_t p_id, int64_t p_with_id, bool bidirectional) const {
	Segment s(p_id, p_with_id);
	const HashSet<Segment, Segment>::Iterator element = segments.find(s);

	return element &&
		(bidirectional || (element->direction & s.direction) == s.direction);
}

void GDAstar3D::clear() {
	last_free_id = 0;
	for (OAHashMap<int64_t, Point*>::Iterator it = points.iter(); it.valid; it = points.next_iter(it)) {
		memdelete(*(it.value));
	}
	segments.clear();
	points.clear();
}

int64_t GDAstar3D::get_point_count() const {
	return points.get_num_elements();
}

int64_t GDAstar3D::get_point_capacity() const {
	return points.get_capacity();
}

void GDAstar3D::reserve_space(int64_t p_num_nodes) {
	ERR_FAIL_COND_MSG(p_num_nodes <= 0, vformat("New capacity must be greater than 0, new was: %d.", p_num_nodes));
	ERR_FAIL_COND_MSG((uint32_t)p_num_nodes < points.get_capacity(), vformat("New capacity must be greater than current capacity: %d, new was: %d.", points.get_capacity(), p_num_nodes));
	points.reserve(p_num_nodes);
}

int64_t GDAstar3D::get_closest_point(const Vector3& p_point, bool p_include_disabled) const {
	int64_t closest_id = -1;
	real_t closest_dist = 1e20;

	for (OAHashMap<int64_t, Point*>::Iterator it = points.iter(); it.valid; it = points.next_iter(it)) {
		if (!p_include_disabled && !(*it.value)->enabled) {
			continue; // Disabled points should not be considered.
		}

		// Keep the closest point's ID, and in case of multiple closest IDs,
		// the smallest one (makes it deterministic).
		real_t d = p_point.distance_squared_to((*it.value)->pos);
		int64_t id = *(it.key);
		if (d <= closest_dist) {
			if (d == closest_dist && id > closest_id) { // Keep lowest ID.
				continue;
			}
			closest_dist = d;
			closest_id = id;
		}
	}

	return closest_id;
}

Vector3 GDAstar3D::get_closest_position_in_segment(const Vector3& p_point) const {
	real_t closest_dist = 1e20;
	Vector3 closest_point;

	for (const Segment& E : segments) {
		Point* from_point = nullptr, * to_point = nullptr;
		points.lookup(E.key.first, from_point);
		points.lookup(E.key.second, to_point);

		if (!(from_point->enabled && to_point->enabled)) {
			continue;
		}

		Vector3 segment[2] = {
			from_point->pos,
			to_point->pos,
		};

		Vector3 p = Geometry3D::get_closest_point_to_segment(p_point, segment);
		real_t d = p_point.distance_squared_to(p);
		if (d < closest_dist) {
			closest_point = p;
			closest_dist = d;
		}
	}

	return closest_point;
}

bool GDAstar3D::_solve(Point* begin_point, Point* end_point) {
	pass++;

	if (!end_point->enabled) {
		return false;
	}

	bool found_route = false;

	Vector<Point*> open_list;
	SortArray<Point*, SortPoints> sorter;

	begin_point->g_score = 0;
	begin_point->f_score = _estimate_cost(begin_point->id, end_point->id);
	open_list.push_back(begin_point);

	while (!open_list.is_empty()) {
		Point* p = open_list[0]; // The currently processed point

		if (p == end_point) {
			found_route = true;
			break;
		}

		sorter.pop_heap(0, open_list.size(), open_list.ptrw()); // Remove the current point from the open list
		open_list.remove_at(open_list.size() - 1);
		p->closed_pass = pass; // Mark the point as closed

		for (OAHashMap<int64_t, Point*>::Iterator it = p->neighbors.iter(); it.valid; it = p->neighbors.next_iter(it)) {
			Point* e = *(it.value); // The neighbor point

			if (!e->enabled || e->closed_pass == pass) {
				continue;
			}

			real_t tentative_g_score = p->g_score + _compute_cost(p->id, e->id) * e->weight_scale;

			bool new_point = false;

			if (e->open_pass != pass) { // The point wasn't inside the open list.
				e->open_pass = pass;
				open_list.push_back(e);
				new_point = true;
			}
			else if (tentative_g_score >= e->g_score) { // The new path is worse than the previous.
				continue;
			}

			e->prev_point = p;
			e->g_score = tentative_g_score;
			e->f_score = e->g_score + _estimate_cost(e->id, end_point->id);

			if (new_point) { // The position of the new points is already known.
				sorter.push_heap(0, open_list.size() - 1, 0, e, open_list.ptrw());
			}
			else {
				sorter.push_heap(0, open_list.find(e), 0, e, open_list.ptrw());
			}
		}
	}

	return found_route;
}

real_t GDAstar3D::_estimate_cost(int64_t p_from_id, int64_t p_to_id) {
	real_t scost;
	if (GDVIRTUAL_CALL(_estimate_cost, p_from_id, p_to_id, scost)) {
		return scost;
	}

	Point* from_point;
	bool from_exists = points.lookup(p_from_id, from_point);
	ERR_FAIL_COND_V_MSG(!from_exists, 0, vformat("Can't estimate cost. Point with id: %d doesn't exist.", p_from_id));

	Point* to_point;
	bool to_exists = points.lookup(p_to_id, to_point);
	ERR_FAIL_COND_V_MSG(!to_exists, 0, vformat("Can't estimate cost. Point with id: %d doesn't exist.", p_to_id));

	return from_point->pos.distance_to(to_point->pos);
}

real_t GDAstar3D::_compute_cost(int64_t p_from_id, int64_t p_to_id) {
	real_t scost;
	if (GDVIRTUAL_CALL(_compute_cost, p_from_id, p_to_id, scost)) {
		return scost;
	}

	Point* from_point;
	bool from_exists = points.lookup(p_from_id, from_point);
	ERR_FAIL_COND_V_MSG(!from_exists, 0, vformat("Can't compute cost. Point with id: %d doesn't exist.", p_from_id));

	Point* to_point;
	bool to_exists = points.lookup(p_to_id, to_point);
	ERR_FAIL_COND_V_MSG(!to_exists, 0, vformat("Can't compute cost. Point with id: %d doesn't exist.", p_to_id));

	return from_point->pos.distance_to(to_point->pos);
}

Vector<Vector3> GDAstar3D::get_point_path(int64_t p_from_id, int64_t p_to_id) {
	Point* a;
	bool from_exists = points.lookup(p_from_id, a);
	ERR_FAIL_COND_V_MSG(!from_exists, Vector<Vector3>(), vformat("Can't get point path. Point with id: %d doesn't exist.", p_from_id));

	Point* b;
	bool to_exists = points.lookup(p_to_id, b);
	ERR_FAIL_COND_V_MSG(!to_exists, Vector<Vector3>(), vformat("Can't get point path. Point with id: %d doesn't exist.", p_to_id));

	if (a == b) {
		Vector<Vector3> ret;
		ret.push_back(a->pos);
		return ret;
	}

	Point* begin_point = a;
	Point* end_point = b;

	bool found_route = _solve(begin_point, end_point);
	if (!found_route) {
		return Vector<Vector3>();
	}

	Point* p = end_point;
	int64_t pc = 1; // Begin point
	while (p != begin_point) {
		pc++;
		p = p->prev_point;
	}

	Vector<Vector3> path;
	path.resize(pc);

	{
		Vector3* w = path.ptrw();

		Point* p2 = end_point;
		int64_t idx = pc - 1;
		while (p2 != begin_point) {
			w[idx--] = p2->pos;
			p2 = p2->prev_point;
		}

		w[0] = p2->pos; // Assign first
	}

	return path;
}

Vector<int64_t> GDAstar3D::get_id_path(int64_t p_from_id, int64_t p_to_id) {
	Point* a;
	bool from_exists = points.lookup(p_from_id, a);
	ERR_FAIL_COND_V_MSG(!from_exists, Vector<int64_t>(), vformat("Can't get id path. Point with id: %d doesn't exist.", p_from_id));

	Point* b;
	bool to_exists = points.lookup(p_to_id, b);
	ERR_FAIL_COND_V_MSG(!to_exists, Vector<int64_t>(), vformat("Can't get id path. Point with id: %d doesn't exist.", p_to_id));

	if (a == b) {
		Vector<int64_t> ret;
		ret.push_back(a->id);
		return ret;
	}

	Point* begin_point = a;
	Point* end_point = b;

	bool found_route = _solve(begin_point, end_point);
	if (!found_route) {
		return Vector<int64_t>();
	}

	Point* p = end_point;
	int64_t pc = 1; // Begin point
	while (p != begin_point) {
		pc++;
		p = p->prev_point;
	}

	Vector<int64_t> path;
	path.resize(pc);

	{
		int64_t* w = path.ptrw();

		p = end_point;
		int64_t idx = pc - 1;
		while (p != begin_point) {
			w[idx--] = p->id;
			p = p->prev_point;
		}

		w[0] = p->id; // Assign first
	}

	return path;
}

void GDAstar3D::set_point_disabled(int64_t p_id, bool p_disabled) {
	Point* p;
	bool p_exists = points.lookup(p_id, p);
	ERR_FAIL_COND_MSG(!p_exists, vformat("Can't set if point is disabled. Point with id: %d doesn't exist.", p_id));

	p->enabled = !p_disabled;
}

bool GDAstar3D::is_point_disabled(int64_t p_id) const {
	Point* p;
	bool p_exists = points.lookup(p_id, p);
	ERR_FAIL_COND_V_MSG(!p_exists, false, vformat("Can't get if point is disabled. Point with id: %d doesn't exist.", p_id));

	return !p->enabled;
}

void GDAstar3D::_bind_methods() {
	ClassDB::bind_method(D_METHOD("get_available_point_id"), &GDAstar3D::get_available_point_id);
	ClassDB::bind_method(D_METHOD("add_point", "id", "position", "weight_scale"), &GDAstar3D::add_point, DEFVAL(1.0));
	ClassDB::bind_method(D_METHOD("get_point_position", "id"), &GDAstar3D::get_point_position);
	ClassDB::bind_method(D_METHOD("set_point_position", "id", "position"), &GDAstar3D::set_point_position);
	ClassDB::bind_method(D_METHOD("get_point_weight_scale", "id"), &GDAstar3D::get_point_weight_scale);
	ClassDB::bind_method(D_METHOD("set_point_weight_scale", "id", "weight_scale"), &GDAstar3D::set_point_weight_scale);
	ClassDB::bind_method(D_METHOD("remove_point", "id"), &GDAstar3D::remove_point);
	ClassDB::bind_method(D_METHOD("has_point", "id"), &GDAstar3D::has_point);
	ClassDB::bind_method(D_METHOD("get_point_connections", "id"), &GDAstar3D::get_point_connections);
	ClassDB::bind_method(D_METHOD("get_point_ids"), &GDAstar3D::get_point_ids);

	ClassDB::bind_method(D_METHOD("set_point_disabled", "id", "disabled"), &GDAstar3D::set_point_disabled, DEFVAL(true));
	ClassDB::bind_method(D_METHOD("is_point_disabled", "id"), &GDAstar3D::is_point_disabled);

	ClassDB::bind_method(D_METHOD("connect_points", "id", "to_id", "bidirectional"), &GDAstar3D::connect_points, DEFVAL(true));
	ClassDB::bind_method(D_METHOD("disconnect_points", "id", "to_id", "bidirectional"), &GDAstar3D::disconnect_points, DEFVAL(true));
	ClassDB::bind_method(D_METHOD("are_points_connected", "id", "to_id", "bidirectional"), &GDAstar3D::are_points_connected, DEFVAL(true));

	ClassDB::bind_method(D_METHOD("get_point_count"), &GDAstar3D::get_point_count);
	ClassDB::bind_method(D_METHOD("get_point_capacity"), &GDAstar3D::get_point_capacity);
	ClassDB::bind_method(D_METHOD("reserve_space", "num_nodes"), &GDAstar3D::reserve_space);
	ClassDB::bind_method(D_METHOD("clear"), &GDAstar3D::clear);

	ClassDB::bind_method(D_METHOD("get_closest_point", "to_position", "include_disabled"), &GDAstar3D::get_closest_point, DEFVAL(false));
	ClassDB::bind_method(D_METHOD("get_closest_position_in_segment", "to_position"), &GDAstar3D::get_closest_position_in_segment);

	ClassDB::bind_method(D_METHOD("get_point_path", "from_id", "to_id"), &GDAstar3D::get_point_path);
	ClassDB::bind_method(D_METHOD("get_id_path", "from_id", "to_id"), &GDAstar3D::get_id_path);

	GDVIRTUAL_BIND(_estimate_cost, "from_id", "to_id")
		GDVIRTUAL_BIND(_compute_cost, "from_id", "to_id")
}

GDAstar3D::~GDAstar3D() {
	clear();
}

/////////////////////////////////////////////////////////////

int64_t GDAstar2D::get_available_point_id() const {
	return astar.get_available_point_id();
}

void GDAstar2D::add_point(int64_t p_id, const Vector2& p_pos, real_t p_weight_scale) {
	astar.add_point(p_id, Vector3(p_pos.x, p_pos.y, 0), p_weight_scale);
}

Vector2 GDAstar2D::get_point_position(int64_t p_id) const {
	Vector3 p = astar.get_point_position(p_id);
	return Vector2(p.x, p.y);
}

void GDAstar2D::set_point_position(int64_t p_id, const Vector2& p_pos) {
	astar.set_point_position(p_id, Vector3(p_pos.x, p_pos.y, 0));
}

real_t GDAstar2D::get_point_weight_scale(int64_t p_id) const {
	return astar.get_point_weight_scale(p_id);
}

void GDAstar2D::set_point_weight_scale(int64_t p_id, real_t p_weight_scale) {
	astar.set_point_weight_scale(p_id, p_weight_scale);
}

void GDAstar2D::remove_point(int64_t p_id) {
	astar.remove_point(p_id);
}

bool GDAstar2D::has_point(int64_t p_id) const {
	return astar.has_point(p_id);
}

Vector<int64_t> GDAstar2D::get_point_connections(int64_t p_id) {
	return astar.get_point_connections(p_id);
}

PackedInt64Array GDAstar2D::get_point_ids() {
	return astar.get_point_ids();
}

void GDAstar2D::set_point_disabled(int64_t p_id, bool p_disabled) {
	astar.set_point_disabled(p_id, p_disabled);
}

bool GDAstar2D::is_point_disabled(int64_t p_id) const {
	return astar.is_point_disabled(p_id);
}

void GDAstar2D::connect_points(int64_t p_id, int64_t p_with_id, bool p_bidirectional) {
	astar.connect_points(p_id, p_with_id, p_bidirectional);
}

void GDAstar2D::disconnect_points(int64_t p_id, int64_t p_with_id, bool p_bidirectional) {
	astar.disconnect_points(p_id, p_with_id, p_bidirectional);
}

bool GDAstar2D::are_points_connected(int64_t p_id, int64_t p_with_id, bool p_bidirectional) const {
	return astar.are_points_connected(p_id, p_with_id, p_bidirectional);
}

int64_t GDAstar2D::get_point_count() const {
	return astar.get_point_count();
}

int64_t GDAstar2D::get_point_capacity() const {
	return astar.get_point_capacity();
}

void GDAstar2D::clear() {
	astar.clear();
}

void GDAstar2D::reserve_space(int64_t p_num_nodes) {
	astar.reserve_space(p_num_nodes);
}

int64_t GDAstar2D::get_closest_point(const Vector2& p_point, bool p_include_disabled) const {
	return astar.get_closest_point(Vector3(p_point.x, p_point.y, 0), p_include_disabled);
}

Vector2 GDAstar2D::get_closest_position_in_segment(const Vector2& p_point) const {
	Vector3 p = astar.get_closest_position_in_segment(Vector3(p_point.x, p_point.y, 0));
	return Vector2(p.x, p.y);
}

real_t GDAstar2D::_estimate_cost(int64_t p_from_id, int64_t p_to_id) {
	real_t scost;
	if (GDVIRTUAL_CALL(_estimate_cost, p_from_id, p_to_id, scost)) {
		return scost;
	}

	GDAstar3D::Point* from_point;
	bool from_exists = astar.points.lookup(p_from_id, from_point);
	ERR_FAIL_COND_V_MSG(!from_exists, 0, vformat("Can't estimate cost. Point with id: %d doesn't exist.", p_from_id));

	GDAstar3D::Point* to_point;
	bool to_exists = astar.points.lookup(p_to_id, to_point);
	ERR_FAIL_COND_V_MSG(!to_exists, 0, vformat("Can't estimate cost. Point with id: %d doesn't exist.", p_to_id));

	return from_point->pos.distance_to(to_point->pos);
}

real_t GDAstar2D::_compute_cost(int64_t p_from_id, int64_t p_to_id) {
	real_t scost;
	if (GDVIRTUAL_CALL(_compute_cost, p_from_id, p_to_id, scost)) {
		return scost;
	}

	GDAstar3D::Point* from_point;
	bool from_exists = astar.points.lookup(p_from_id, from_point);
	ERR_FAIL_COND_V_MSG(!from_exists, 0, vformat("Can't compute cost. Point with id: %d doesn't exist.", p_from_id));

	GDAstar3D::Point* to_point;
	bool to_exists = astar.points.lookup(p_to_id, to_point);
	ERR_FAIL_COND_V_MSG(!to_exists, 0, vformat("Can't compute cost. Point with id: %d doesn't exist.", p_to_id));

	return from_point->pos.distance_to(to_point->pos);
}

Vector<Vector2> GDAstar2D::get_point_path(int64_t p_from_id, int64_t p_to_id) {
	GDAstar3D::Point* a;
	bool from_exists = astar.points.lookup(p_from_id, a);
	ERR_FAIL_COND_V_MSG(!from_exists, Vector<Vector2>(), vformat("Can't get point path. Point with id: %d doesn't exist.", p_from_id));

	GDAstar3D::Point* b;
	bool to_exists = astar.points.lookup(p_to_id, b);
	ERR_FAIL_COND_V_MSG(!to_exists, Vector<Vector2>(), vformat("Can't get point path. Point with id: %d doesn't exist.", p_to_id));

	if (a == b) {
		Vector<Vector2> ret = { Vector2(a->pos.x, a->pos.y) };
		return ret;
	}

	GDAstar3D::Point* begin_point = a;
	GDAstar3D::Point* end_point = b;

	bool found_route = _solve(begin_point, end_point);
	if (!found_route) {
		return Vector<Vector2>();
	}

	GDAstar3D::Point* p = end_point;
	int64_t pc = 1; // Begin point
	while (p != begin_point) {
		pc++;
		p = p->prev_point;
	}

	Vector<Vector2> path;
	path.resize(pc);

	{
		Vector2* w = path.ptrw();

		GDAstar3D::Point* p2 = end_point;
		int64_t idx = pc - 1;
		while (p2 != begin_point) {
			w[idx--] = Vector2(p2->pos.x, p2->pos.y);
			p2 = p2->prev_point;
		}

		w[0] = Vector2(p2->pos.x, p2->pos.y); // Assign first
	}

	return path;
}

Vector<int64_t> GDAstar2D::get_id_path(int64_t p_from_id, int64_t p_to_id) {
	GDAstar3D::Point* a;
	bool from_exists = astar.points.lookup(p_from_id, a);
	ERR_FAIL_COND_V_MSG(!from_exists, Vector<int64_t>(), vformat("Can't get id path. Point with id: %d doesn't exist.", p_from_id));

	GDAstar3D::Point* b;
	bool to_exists = astar.points.lookup(p_to_id, b);
	ERR_FAIL_COND_V_MSG(!to_exists, Vector<int64_t>(), vformat("Can't get id path. Point with id: %d doesn't exist.", p_to_id));

	if (a == b) {
		Vector<int64_t> ret;
		ret.push_back(a->id);
		return ret;
	}

	GDAstar3D::Point* begin_point = a;
	GDAstar3D::Point* end_point = b;

	bool found_route = _solve(begin_point, end_point);
	if (!found_route) {
		return Vector<int64_t>();
	}

	GDAstar3D::Point* p = end_point;
	int64_t pc = 1; // Begin point
	while (p != begin_point) {
		pc++;
		p = p->prev_point;
	}

	Vector<int64_t> path;
	path.resize(pc);

	{
		int64_t* w = path.ptrw();

		p = end_point;
		int64_t idx = pc - 1;
		while (p != begin_point) {
			w[idx--] = p->id;
			p = p->prev_point;
		}

		w[0] = p->id; // Assign first
	}

	return path;
}

bool GDAstar2D::_solve(GDAstar3D::Point* begin_point, GDAstar3D::Point* end_point) {
	astar.pass++;

	if (!end_point->enabled) {
		return false;
	}

	bool found_route = false;

	LocalVector<GDAstar3D::Point*> open_list;
	SortArray<GDAstar3D::Point*, GDAstar3D::SortPoints> sorter;

	begin_point->g_score = 0;
	begin_point->f_score = _estimate_cost(begin_point->id, end_point->id);
	open_list.push_back(begin_point);

	while (!open_list.is_empty()) {
		GDAstar3D::Point* p = open_list[0]; // The currently processed point.

		if (p == end_point) {
			found_route = true;
			break;
		}

		sorter.pop_heap(0, open_list.size(), open_list.ptr()); // Remove the current point from the open list.
		open_list.remove_at(open_list.size() - 1);
		p->closed_pass = astar.pass; // Mark the point as closed.

		for (OAHashMap<int64_t, GDAstar3D::Point*>::Iterator it = p->neighbors.iter(); it.valid; it = p->neighbors.next_iter(it)) {
			GDAstar3D::Point* e = *(it.value); // The neighbor point.

			if (!e->enabled || e->closed_pass == astar.pass) {
				continue;
			}

			real_t tentative_g_score = p->g_score + _compute_cost(p->id, e->id) * e->weight_scale;

			bool new_point = false;

			if (e->open_pass != astar.pass) { // The point wasn't inside the open list.
				e->open_pass = astar.pass;
				open_list.push_back(e);
				new_point = true;
			}
			else if (tentative_g_score >= e->g_score) { // The new path is worse than the previous.
				continue;
			}

			e->prev_point = p;
			e->g_score = tentative_g_score;
			e->f_score = e->g_score + _estimate_cost(e->id, end_point->id);

			if (new_point) { // The position of the new points is already known.
				sorter.push_heap(0, open_list.size() - 1, 0, e, open_list.ptr());
			}
			else {
				sorter.push_heap(0, open_list.find(e), 0, e, open_list.ptr());
			}
		}
	}

	return found_route;
}

void GDAstar2D::_bind_methods() {
	ClassDB::bind_method(D_METHOD("get_available_point_id"), &GDAstar2D::get_available_point_id);
	ClassDB::bind_method(D_METHOD("add_point", "id", "position", "weight_scale"), &GDAstar2D::add_point, DEFVAL(1.0));
	ClassDB::bind_method(D_METHOD("get_point_position", "id"), &GDAstar2D::get_point_position);
	ClassDB::bind_method(D_METHOD("set_point_position", "id", "position"), &GDAstar2D::set_point_position);
	ClassDB::bind_method(D_METHOD("get_point_weight_scale", "id"), &GDAstar2D::get_point_weight_scale);
	ClassDB::bind_method(D_METHOD("set_point_weight_scale", "id", "weight_scale"), &GDAstar2D::set_point_weight_scale);
	ClassDB::bind_method(D_METHOD("remove_point", "id"), &GDAstar2D::remove_point);
	ClassDB::bind_method(D_METHOD("has_point", "id"), &GDAstar2D::has_point);
	ClassDB::bind_method(D_METHOD("get_point_connections", "id"), &GDAstar2D::get_point_connections);
	ClassDB::bind_method(D_METHOD("get_point_ids"), &GDAstar2D::get_point_ids);

	ClassDB::bind_method(D_METHOD("set_point_disabled", "id", "disabled"), &GDAstar2D::set_point_disabled, DEFVAL(true));
	ClassDB::bind_method(D_METHOD("is_point_disabled", "id"), &GDAstar2D::is_point_disabled);

	ClassDB::bind_method(D_METHOD("connect_points", "id", "to_id", "bidirectional"), &GDAstar2D::connect_points, DEFVAL(true));
	ClassDB::bind_method(D_METHOD("disconnect_points", "id", "to_id", "bidirectional"), &GDAstar2D::disconnect_points, DEFVAL(true));
	ClassDB::bind_method(D_METHOD("are_points_connected", "id", "to_id", "bidirectional"), &GDAstar2D::are_points_connected, DEFVAL(true));

	ClassDB::bind_method(D_METHOD("get_point_count"), &GDAstar2D::get_point_count);
	ClassDB::bind_method(D_METHOD("get_point_capacity"), &GDAstar2D::get_point_capacity);
	ClassDB::bind_method(D_METHOD("reserve_space", "num_nodes"), &GDAstar2D::reserve_space);
	ClassDB::bind_method(D_METHOD("clear"), &GDAstar2D::clear);

	ClassDB::bind_method(D_METHOD("get_closest_point", "to_position", "include_disabled"), &GDAstar2D::get_closest_point, DEFVAL(false));
	ClassDB::bind_method(D_METHOD("get_closest_position_in_segment", "to_position"), &GDAstar2D::get_closest_position_in_segment);

	ClassDB::bind_method(D_METHOD("get_point_path", "from_id", "to_id"), &GDAstar2D::get_point_path);
	ClassDB::bind_method(D_METHOD("get_id_path", "from_id", "to_id"), &GDAstar2D::get_id_path);

	GDVIRTUAL_BIND(_estimate_cost, "from_id", "to_id")
		GDVIRTUAL_BIND(_compute_cost, "from_id", "to_id")
}