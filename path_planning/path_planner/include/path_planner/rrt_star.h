#ifndef RRT_STAR
#define RRT_STAR
#include <iostream>
#include <ctime>   // time()
#include <cstdlib> // rand()
#include <vector>
#include <cmath>
#include <algorithm> // reverse()

using namespace std;


enum taskState{
    DRIVING_SECTION,
    INTERSECTION_STRAIGHT,
    INTERSECTION_LEFT,
    INTERSECTION_RIGHT,
    INTERSECTION_STRAIGHT_UNSIGNED,
    INTERSECTION_LEFT_UNSIGNED,
    INTERSECTION_RIGHT_UNSIGNED,
    OBSTACLE_STATIC,
    OBSTACLE_SUDDEN,
    CROSSWALK,
    PARKING
};

enum lightState{
    GREEN_LIGHT,
    LEFT_LIGHT,
    YELLOW_LIGHT,
    RED_LIGHT
};

enum motionState{
    FORWARD_MOTION,
    FORWARD_SLOW_MOTION,
    HALT_MOTION,
    LEFT_MOTION,
    RIGHT_MOTION,
    PARKING_MOTION
};

enum parkingState{
    SEARCHING_PARKING_SPOT,
    PARKING_SPOT_0,
    PARKING_SPOT_1,
    PARKING_SPOT_2,
    PARKING_SPOT_3,
    PARKING_SPOT_4,
    PARKING_SPOT_5
};


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
        int iternum; // iteration number
        double radius; // radius to find near nodes
        double stepsize; // step size to check obstacle and steer (cost(), steer())
        double threshold; // threshold to check obstacle
	double threshold2 = 50; // threshold used in straightCheck

        RRT(int _iternum, double _radius, double _stepsize, double _threshold, double _threshold2) {
            iternum = _iternum; radius = _radius; stepsize = _stepsize; threshold = _threshold; threshold2 = _threshold2;
        }
        ~RRT() {}
        //test
	    // void print_RRT(){
		//     cout << "iternum: " << iternum << endl;
		//     cout << "radius: " << radius << endl;
		//     cout << "stepsize: " << stepsize << endl;
		//     cout << "threshold: " << threshold << endl;
	    // };
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

        void solve(std::vector<Cor>& path, std::vector<std::vector<double>>& _cost_map, Cor start, Cor goal, bool isObstacleSudden=false);


		bool isobstacle(Cor a, Cor b);

		bool isobstacle(Cor c);
};

#endif
