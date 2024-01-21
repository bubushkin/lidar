/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"


// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	void rinsert(Node **node, int depth, std::vector<float> point, int id){
		
		if(*node == NULL){
			*node = new Node(point, id);
		} else{
			if(point[depth % 2] <= (*node)->point[depth % 2]){
				rinsert(&(*node)->left, ++depth, point, id);
			} else {
				rinsert(&(*node)->right, ++depth, point, id);
			}
		}
	}

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		
		rinsert(&root, 0, point, id);

	}

	// return a list of point ids in the tree that are within distance of target
	void rsearch(std::vector<int> &ids, Node *node, int depth, std::vector<float>target, float distanceTol){

		if(node != NULL){
			//std::cout << is_inside(node->point, target, distanceTol) << std::endl;
			if(is_inside(node->point, target, distanceTol)){
				std::cout << distance(node->point, target) << std::endl;
				if(distance(node->point, target) <= distanceTol)
				ids.push_back(node->id);
			}
			if((target[depth % 2] - distanceTol) < node->point[depth % 2]){
				rsearch(ids, node->left, depth + 1, target, distanceTol);
			}
			if((target[depth % 2] + distanceTol) > node->point[depth % 2]){
				rsearch(ids, node->right, depth + 1, target, distanceTol);
			}

		}

	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol) {
		std::vector<int> ids;

		rsearch(ids, root, 0, target, distanceTol);
		//std::cout << ids.size() << std::endl;

		
		return ids;
	}


};




