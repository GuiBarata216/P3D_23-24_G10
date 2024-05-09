#include "rayAccelerator.h"
#include "macros.h"

using namespace std;

BVH::BVHNode::BVHNode(void) {}

void BVH::BVHNode::setAABB(AABB& bbox_) { this->bbox = bbox_; }

void BVH::BVHNode::makeLeaf(unsigned int index_, unsigned int n_objs_) {
	this->leaf = true;
	this->index = index_; 
	this->n_objs = n_objs_; 
}

void BVH::BVHNode::makeNode(unsigned int left_index_) {
	this->leaf = false;
	this->index = left_index_; 
	//this->n_objs = n_objs_; 
}


BVH::BVH(void) {}

int BVH::getNumObjects() { return objects.size(); }


void BVH::Build(vector<Object *> &objs) {

		
			BVHNode *root = new BVHNode();

			Vector min = Vector(FLT_MAX, FLT_MAX, FLT_MAX), max = Vector(-FLT_MAX, -FLT_MAX, -FLT_MAX);
			AABB world_bbox = AABB(min, max);

			for (Object* obj : objs) {
				AABB bbox = obj->GetBoundingBox();
				world_bbox.extend(bbox);
				objects.push_back(obj);
			}
			world_bbox.min.x -= EPSILON; world_bbox.min.y -= EPSILON; world_bbox.min.z -= EPSILON;
			world_bbox.max.x += EPSILON; world_bbox.max.y += EPSILON; world_bbox.max.z += EPSILON;
			root->setAABB(world_bbox);
			nodes.push_back(root);
			build_recursive(0, objects.size(), root); // -> root node takes all the 
		}

void BVH::build_recursive(int left_index, int right_index, BVHNode* node) {
	//PUT YOUR CODE HERE

	if ((right_index - left_index) <= 2) node->makeLeaf(left_index, right_index - left_index); // Check index

	else {
		int split_index;

		if (this -> SAH_splits < 3) {
			split_index = this->SAH(left_index, right_index, node);
			this -> SAH_splits++;
		}

		else split_index = this->find_split(left_index, right_index, node);

		AABB left_bbox = this->build_bbox(left_index, split_index);
		AABB right_bbox = this->build_bbox(split_index, right_index);

		BVHNode* left_node = new BVHNode();
		BVHNode* right_node = new BVHNode();

		node->makeNode(this->nodes.size());

		left_node->setAABB(left_bbox);
		right_node->setAABB(right_bbox);

		nodes.push_back(left_node);
		nodes.push_back(right_node);

		this->build_recursive(left_index, split_index, left_node);
		this->build_recursive(split_index, right_index, right_node);

		//right_index, left_index and split_index refer to the indices in the objects vector
		// do not confuse with left_nodde_index and right_node_index which refer to indices in the nodes vector. 
		// node.index can have a index of objects vector or a index of nodes vector
	}
}

int BVH::SAH(int left_index, int right_index, BVHNode* node) {
	float cost_traversal = 1.0f;
	float cost_intersection = 10.0f;
	float total_cost = 0.0f;
	float min_cost = FLT_MAX;
	float split_index;
	float cost_parent = (right_index - left_index) * cost_intersection;
	float sa_parent = node->getAABB().surface_area();
	float sa_left, sa_rigth;

	BVH::Comparator comparator;

	for (int n = 0; n < 3; n++) {
		int k = 1;
		comparator.dimension = 0;
		std::sort(this->objects.begin() + left_index, this->objects.begin() + right_index, comparator);

		vector<float> right_surface_areas;
		Vector min = Vector(FLT_MAX, FLT_MAX, FLT_MAX);
		Vector max = Vector(-FLT_MAX, -FLT_MAX, -FLT_MAX);

		AABB right_bbox = AABB(min, max);
		AABB left_bbox = AABB(min, max);

		for (int i = right_index - 1; i >= left_index + 1; i--) {
			Object* obj = this->objects[i];
			right_bbox.extend(obj->GetBoundingBox());
			right_surface_areas.push_back(right_bbox.surface_area());
		}

		for (int i = left_index; i < right_index; i++, k++) {
			Object* obj = this->objects[i];
			left_bbox.extend(obj->GetBoundingBox());

			sa_left = left_bbox.surface_area();
			sa_rigth = right_surface_areas[right_index - 2 - i];

			total_cost = cost_traversal + (sa_left / sa_parent) * k * cost_intersection + (sa_rigth / sa_parent) * (right_index - (left_index + k)) * cost_intersection;

			if (total_cost < cost_parent) {
				if (total_cost < min_cost) {
					min_cost = total_cost;
					split_index = left_index + k;
				}
				break;
			}
		}
	}

	if (split_index >= right_index) return (right_index + left_index) / 2;

	return split_index;
}

int BVH::find_split(int left_index, int right_index, BVHNode* node) {

	AABB bbox = node->getAABB();

	int largest_coor = (bbox.max - bbox.min).largest_coordinate();
	float largest_dist = (bbox.max - bbox.min).getAxisValue(largest_coor);

	BVH::Comparator comparator;
	comparator.dimension = largest_coor;
	std::sort(this->objects.begin() + left_index, this->objects.begin() + right_index, comparator);

	int split_index = left_index;

	if (this->objects[left_index]->getCentroid().getAxisValue(largest_coor) > largest_dist / 2 || 
		this->objects[right_index - 1]->getCentroid().getAxisValue(largest_coor) < largest_dist / 2) {
		return (right_index + left_index) / 2;
	}

	while (split_index < right_index &&
		this->objects[split_index]->getCentroid().getAxisValue(largest_coor) < largest_dist / 2) {

		split_index++;
	}
	
	if (split_index < right_index && split_index > left_index) return split_index;

}

AABB BVH::build_bbox(int left_index, int right_index) {
	Vector min = Vector(FLT_MAX, FLT_MAX, FLT_MAX);
	Vector max = Vector(-FLT_MAX, -FLT_MAX, -FLT_MAX);
	AABB bbox = AABB(min, max);

	for (int i = left_index; i < right_index; i++) {
		AABB obj_bbox = this->objects[i]->GetBoundingBox();
		bbox.extend(obj_bbox);
	}

	return bbox;
}

bool BVH::Traverse(Ray& ray, Object** hit_obj, Vector& hit_point) {
	float tmp;
	float tmin = FLT_MAX;  //contains the closest primitive intersection
	bool hit = false;

	BVHNode* current_node = nodes[0];
	AABB current_bbox = current_node->getAABB();

	if (!current_bbox.intercepts(ray, tmp)) return false;

	while (true) {
		if (!current_node->isLeaf()) {
			BVHNode* left_child = this->nodes[current_node->getIndex()];
			BVHNode* right_child = this->nodes[current_node->getIndex() + 1];

			float tmp_1 = 0;
			float tmp_2 = 0;

			bool left_hit = left_child->getAABB().intercepts(ray, tmp_1);
			bool right_hit = right_child->getAABB().intercepts(ray, tmp_2);

			if (left_hit && right_hit) {
				StackItem stack_item(left_child, tmp_1);
				current_node = right_child;

				if (tmp_2 > tmp_1) {
					stack_item.ptr = right_child;
					stack_item.t = tmp_2;
					current_node = left_child;
				}

				this->hit_stack.push(stack_item);
				continue;
			}

			else if (left_hit || right_hit) {
				current_node = left_hit ? left_child : right_child;
				continue;
			}
		}

		else {
			for (int i = current_node->getIndex(); i < current_node->getIndex() + current_node->getNObjs(); i++) {
				Object* obj = this->objects[i];
				float tmp;

				if (obj->GetBoundingBox().intercepts(ray, tmp)) {
					if (obj->intercepts(ray, tmp) && tmp < tmin) {
						tmin = tmp;
						*hit_obj = obj;
					}
				}
			}
		}

		while (true) {
			if (this->hit_stack.empty()) {
				if (tmin == FLT_MAX) {
					return false;
				}

				hit_point = ray.origin + ray.direction * tmin;
				return true;
			}

			StackItem stack_item = this->hit_stack.top();

			if (stack_item.t < tmin) {
				current_node = stack_item.ptr;
				this->hit_stack.pop();
				break;
			}

			this->hit_stack.pop();
		}
	}

	hit_point = ray.origin + ray.direction * tmin;
	return true;
}

bool BVH::Traverse(Ray& ray) {  //shadow ray with length
			float temp;
			double length = ray.direction.length(); //distance between light and intersection point
			ray.direction.normalize();

			BVHNode* current_node = nodes[0];
			AABB current_bbox = current_node->getAABB();

			if (!current_bbox.intercepts(ray, temp)) return false;

			while (true) {
				if (!current_node->isLeaf()) {
					BVHNode* left_child = this->nodes[current_node->getIndex()];
					BVHNode* right_child = this->nodes[current_node->getIndex() + 1];

					float temp1 = 0;
					float temp2 = 0;

					bool left_hit = left_child->getAABB().intercepts(ray, temp1);
					bool right_hit = right_child->getAABB().intercepts(ray, temp2);

					if (left_hit && right_hit) {
						StackItem stack_item(left_child, temp1);
						current_node = right_child;

						if (temp2 > temp1) {
							stack_item.ptr = right_child;
							stack_item.t = temp2;
							current_node = left_child;
						}

						this->hit_stack.push(stack_item);
						continue;
					}

					else if (left_hit || right_hit) {
						current_node = left_hit ? left_child : right_child;
						continue;
					}
				}

				else {
					for (int i = current_node->getIndex(); i < current_node->getIndex() + current_node->getNObjs(); i++) {
						Object* obj = this->objects[i];
						float temp;

						if (obj->GetBoundingBox().intercepts(ray, temp)) {
							if (obj->intercepts(ray, temp)) {
								return true;
							}
						}
					}
				}

				while (true) {
					if (this->hit_stack.empty()) {
						return false;
					}

					StackItem stack_item = this->hit_stack.top();
					current_node = stack_item.ptr;
					this->hit_stack.pop();
					break;
				}
			}

			return(false);
}
