#ifndef RRT_STAR
#define RRT_STAR
#include <iostream>
#include <ctime>   // time()
#include <cstdlib> // rand()
#include <vector>
#include <cmath>
#include <algorithm> // reverse()

using namespace std;

class Cor {
    public:
        double x, y;
        Cor() : Cor(0, 0) {}
        Cor(double _x, double _y) : x(_x), y(_y) {}
        double dist(Cor other) {
            return sqrt((x - other.x) * (x - other.x) + (y - other.y) * (y - other.y));
        }
        bool operator==(const Cor& c) {
            return x == c.x && y == c.y;
        }
};

class Node {
    public:
        double cost_sum; // cost sum from the start
        Cor location;
        Node* parent;
        std::vector<Node *> children;
        Node() : Node(0, Cor(), nullptr) {}
        Node(Cor _location) : Node(0, _location, nullptr) {}
        Node(double _cost_sum, Cor _location, Node* _parent) : cost_sum(_cost_sum), location(_location), parent(_parent), children(std::vector<Node*>()) {}
		void cost_update(double diff) {cost_sum-=diff;}
        void insert_child(Node* ptr) { children.push_back(ptr); }
};

class Tree {
    private:
        std::vector<Node> nodes;
        int node_cnt;
        int size;
    public:
        Tree() {}
        Tree(int _size) : node_cnt(0), size(_size) {
            nodes = std::vector<Node>();
            nodes.resize(_size);
        }
        ~Tree() {
            std::vector<Node>().swap(nodes); // memory free
        }
	const int arrsize() {return node_cnt;}
	Node& operator[](int idx) {return nodes[idx];}
        Node* insert(Node node) {
            if (node_cnt == size) {
                size <<= 1;
                nodes.resize(size);
            }
            nodes[node_cnt++] = node;
			return &nodes[node_cnt-1];
        }
};


// optimal path = cost가 가장 작은 path
class RRT {
    public:
        std::vector<std::vector<double>> cost_map;
        int size = 1<<13; // intialize tree size
        const int map_length = 200; // map length
        int iternum = 1000; // iteration number
        const double radius = 30; // radius to find near nodes
        const double stepsize = 1; // step size to check obstacle and steer (cost(), steer())
        const double threshold = 100; // threshold to check obstacle
	const double threshold2 = 50; // threshold used in straightCheck

        RRT() {}
        ~RRT() {}

		// cost between two points
		double cost(Cor start, Cor dest);

        // pick random point in map
        Cor random_point();

        // pick nearest point
        Cor nearest(Tree& tree, Cor point);

        // cost가 threshold이상이거나 거리가 radius이상이면 stop, 만약 dest까지 갔다면 check = true
        // 위의 조건문을 step 마다 확인
        Cor steer(const Cor start, const Cor dest, bool& check);

        // pick every near points
        void near(std::vector<Node*>& Q_near, Tree& tree, Cor q_new);

        // near points들 중 가장 cost가 작게 되는 parent
        Node chooseParent(std::vector<Node*>& Q_near, const Cor child);

        // near points들의 cost update
        void rewire(std::vector<Node*>& Q_near, Node* q_new_node, Tree& tree);

		// cost update
		void cost_update(Node* parent, const double diff, Tree& tree);

        // path발견시 부모를 따라가면서 parent, cost update
        void pathOptimization(Node* q_cur, Tree& tree);

	// check straight path from start to dest
	bool straightCheck(Cor start, Cor dest);

        void solve(std::vector<Cor>& path, std::vector<std::vector<double>>& _cost_map, Cor start, Cor goal, int _iternum);


		bool isobstacle(Cor a, Cor b);

		bool isobstacle(Cor c);
};

#endif
