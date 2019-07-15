/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"


// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id{-1};
	Node* left{nullptr};
	Node* right{nullptr};

	Node(const std::vector<float>& arr, int setId)
	:	point(arr), id(setId), left(nullptr), right(nullptr)
	{}

	~Node() = default;
};

struct KdTree
{
	Node* root{nullptr};

	KdTree()
	: root(nullptr)
	{}

	~KdTree()
	{
		freeMemory(root);
	}

	private:

	void freeMemory(Node* curNode)
	{
		if (curNode == nullptr)
		{
			return;
		}

		freeMemory(curNode->left);
		freeMemory(curNode->right);

		delete curNode;
	}

	void insertHelper(
		Node** node,
		std::size_t depth,
		const std::vector<float>& point,
		int idx)
	{
		if (*node == nullptr)
		{
			*node = new Node(point, idx);
		}
		else
		{
			const std::size_t cd = depth % 2;

			if (point.at(cd) < (*node)->point.at(cd))
			{
				insertHelper(&((*node)->left), depth + 1u, point, idx);
			}
			else
			{
				insertHelper(&((*node)->right), depth + 1u, point, idx);
			}

		}
		
	}

	void searchHelper(
		const std::vector<float>& target,
		Node* node,
		int depth,
		float distanceTol,
		std::vector<int>& ids) const
	{
		if (node != nullptr)
		{

			const float diffX = node->point[0]- target[0];
			const float diffY = node->point[1]- target[1];

			if (diffX >= -distanceTol &&
				diffX <= distanceTol &&
				diffY >= -distanceTol &&
				diffY <= distanceTol)
			{
				const float distance = std::sqrt(diffX * diffX + diffY * diffY);
				if (distance <= distanceTol)
				{
					ids.push_back(node->id);
				}
			}


			//const std::size_t cd = depth % 2;

			if (target[depth % 2] - distanceTol < node->point[depth % 2])
			{
				searchHelper(target, node->left, depth + 1, distanceTol, ids);
			}

			if (target[depth % 2] + distanceTol > node->point[depth % 2])
			{
				searchHelper(target, node->right, depth + 1, distanceTol, ids);
			}	
		}
	}

	public:

	void insert(const std::vector<float>& point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		insertHelper(&root, 0, point, id);

	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(const std::vector<float>& target, float distanceTol) const
	{
		std::vector<int> ids;

		searchHelper(target, root, 0, distanceTol, ids);

		return ids;
	}		

};




