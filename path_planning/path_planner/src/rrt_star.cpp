#include "rrt_star.h"
#include "ros/ros.h"
#include <vector>
#include <iostream>
#include <ctime>
#include <queue>
//#include <opencv2/opencv.hpp> // for visualization

// for debugging
void debug(bool reset = false) {
	static int cnt = 0;
	if(reset) cnt = 0;	
	//ROS_INFO("%d",cnt++);
}

int elapsed_time[8] = { 0, };

void debugtime(int flag = -1) {
	static int i = 0;
	static int prev = 0;
	if (flag == -2) {
		prev = clock();
		return;
	}
	if (flag != -1) i = flag;
	else i++;
	int cur = clock();
	elapsed_time[i] += cur - prev;
	prev = cur;
}


// cost between two points
// return (average cost between two points) * distance
double RRT::cost(Cor start, Cor dest) 
{
    Cor marcher{start};
    double ds{ start.dist(dest) }, ret = 0, temp;
    double dx{ (dest.x - start.x) * stepsize / ds }, dy{ (dest.y - start.y) * stepsize / ds };
    int step_times = 0;
    while (1)
    {
        // march!!

	// if marcher is out of map, break
	if (static_cast<int>(marcher.x) < 0 || static_cast<int>(marcher.x) >= map_length || static_cast<int>(marcher.y) < 0 || static_cast<int>(marcher.y) >= map_length) break;

        step_times++;
        ret += cost_map[static_cast<int>(marcher.x)][static_cast<int>(marcher.y)];
        if (step_times * stepsize > ds) break;
	marcher.x += dx;
	marcher.y += dy;
    }
    return ds * ret / step_times + 1E-6;
}

// pick random point in map
Cor RRT::random_point()
{
	double x = (double)rand() / RAND_MAX * map_length; // x : 0~200 /////Rand_max?????
	double y = (double)rand() / RAND_MAX * map_length; // y : 0~200
	return Cor(x, y);
}

// pick nearest point
Cor RRT::nearest(Tree& tree, Cor point)
{
	double min_distance{100000000000};
	int min_idx{-1};
	double temp_distance{0};
	for (int i{0}; i<tree.arrsize(); ++i)
	{
		temp_distance = point.dist(tree[i].location);
		if (temp_distance < min_distance)
		{
			min_distance = temp_distance;
			min_idx = i;
		}
	}
	return tree[min_idx].location;
}

// cost가 threshold이상이거나 거리가 radius이상이면 stop, 만약 dest까지 갔다면 check = true
// 위의 조건문을 step 마다 확인
Cor RRT::steer(Cor start, Cor dest, bool& check)
{	
	// step 밟아가며 나아가는 point
	Cor marcher{Cor()};
	Cor marcher_saved{Cor()};
	double ds{start.dist(dest)};
	int step_times = 0;
	bool first = true;
	while (1)
	{
		// march!!
		marcher.x = start.x + (dest.x-start.x)*stepsize*step_times/ds;
		marcher.y = start.y + (dest.y-start.y)*stepsize*step_times/ds;
		// 첫번째 loop인 경우 marcher_saved에 저장된 값이 없다. 
		// 또한 start 점과 동일하기 때문에 다양한 조건을 체크할 필요가 없다.
		if (first){
			first = false;
			marcher_saved = marcher;
			step_times++;
			continue;
		}

		// out of map! return!
		if (marcher.x < 0 || marcher.x >= map_length || marcher.y < 0 || marcher.y >= map_length) {
			return marcher_saved;
		}

		// 1. 직전 marcher point와 marcher point 간에 장애물이 있을 경우 
		// 직전 marcher point를 return
		// 두점을 비교하는 이유 : 해당 위치에 장애물이 있는지 보다 
		// 점과 점간에 장애물이 있는지 확인하는게 더 정확함. 
		// (step을 밟아가다보면 장애물을 건너 뛰어 버릴 수 있음 
		// step이 충분히 작은 경우 그 점에서의 장애물만 확인해도 됨)
		// 중앙선의 경우 폭이 15cm라고하면 step이 5보다 커지면 중앙선을 건너 뛰어버릴 수 있음
		// start 지점과 비교하지 않는 이유 : costmap 특성상 비교하는 점과의 거리가 
		// 멀면 멀 수록 연산시간이 오래걸릴 가능성이 높음
		// 따라서 isobstacle function을 overload해 놓으면 step의 size에 따라서 선택을 할 수 있음
		if ( stepsize >= 5 ? isobstacle(marcher_saved,marcher) : isobstacle(marcher))
		{
			return marcher_saved;
		}
		// 2. radius 영역을 벗어나면 직전 marcher return
		if (step_times*stepsize > radius)
			return marcher_saved;
		// 3. destination을 넘어가면 check을 true로 바꾸고 destination을 return
		if (step_times*stepsize >= ds){
			check = true;
			return dest;
		}
		// udate variable
		step_times++;
		marcher_saved = marcher;
	}
}

// pick every near points
void RRT::near(std::vector<Node*>& Q_near, Tree& tree, Cor q_new)
{
	double temp_distance{0};
	int sz = tree.arrsize();
	for (int i{0}; i<sz; ++i)
	{
		temp_distance = q_new.dist(tree[i].location);
		if (temp_distance < radius + 1e-6)
			Q_near.push_back(&(tree[i]));
	}
	return;
}

// near points들 중 가장 cost가 작게 되는 parent
Node RRT::chooseParent(std::vector<Node*>& Q_near, const Cor child)
{
	Cor a = child;
	
	Node new_node{Node()};
	Node* min_cost_parent{nullptr};
	double min_cost{10000000};
	double temp_cost{0};
	for(Node* temp:Q_near)
	{
		temp_cost =temp->cost_sum + cost(child,temp->location);
		if (temp_cost<min_cost)
		{
			min_cost = temp_cost;
			min_cost_parent = temp;
		}
	}
	new_node.cost_sum = min_cost;
	new_node.location = child;
	new_node.parent = min_cost_parent;
	return new_node;
}

// near points들의 cost update
void RRT::rewire(std::vector<Node*>& Q_near, Node* q_new_node, Tree& tree)
{
	double temp_cost{0};
	for(Node* node:Q_near)
	{
		temp_cost = q_new_node->cost_sum + cost(node->location,q_new_node->location);
		if (temp_cost < node->cost_sum)
		{
			double diff{node->cost_sum-temp_cost};
			cost_update(node,diff,tree);
			node->parent = q_new_node;
			q_new_node->insert_child(node);
		}
	}
	return;
}

// cost update
void RRT::cost_update(Node* parent, const double diff, Tree& tree) // possibility of time cost reduction.
{
	queue<Node*> q;
	q.push(parent);
	while (!q.empty()) {
		Node* cur = q.front(); q.pop();
		cur->cost_update(diff);
		for (Node* child : cur->children) {
			if (child->parent != cur) continue;
			q.push(child);
		}
	}
}


// path발견시 부모를 따라가면서 parent, cost update
void RRT::pathOptimization(Node* q_cur, Tree& tree)
{
	Node* curr{ q_cur };
	Node* temp_parent;
	Node* original_parent;
	bool path_optimization{ false };
	double original_cost{ 0 };
	double temp_cost;
	double curr_cost;
	while (!path_optimization)
	{
		original_parent = curr->parent;
		original_cost = curr->cost_sum;
		curr_cost = curr->cost_sum;
		if (!original_parent) break;
		while (1)
		{
			temp_parent = curr->parent->parent;
			if (!temp_parent) {
				path_optimization = true;
				break;
			}
			temp_cost = temp_parent->cost_sum + cost(curr->location, temp_parent->location);
			if (temp_cost >= curr_cost)
			{
				if(curr->parent != original_parent)
					cost_update(curr, original_cost - curr_cost, tree);
				curr = curr->parent;
				break;
				
			}
			else
			{
				curr_cost = temp_cost;
				curr->parent = temp_parent;
			}
		}
		// move to next marching point
		curr = curr->parent;
	}
	return;
}

// check straight path from start to dest
bool RRT::straightCheck(Cor start, Cor dest){
	Cor marcher{start};
	int step_times = 0;
    	double ds{ start.dist(dest) };
	double dx{ (dest.x - start.x) * stepsize / ds }, dy{ (dest.y - start.y) * stepsize / ds };
	while (1)
	{
	        // march!!

		// if out of map, break;
		if (static_cast<int>(marcher.x) < 0 || static_cast<int>(marcher.x) >= map_length || static_cast<int>(marcher.y) < 0 || static_cast<int>(marcher.y) >= map_length) break;

		if(threshold2 <= cost_map[static_cast<int>(marcher.x)][static_cast<int>(marcher.y)]) return false;
		step_times++;
        	if (step_times * stepsize > ds) break;
		marcher.x += dx;
		marcher.y += dy;
	}
	return true;	
}

bool RRT::solve(std::vector<Cor>& path, std::vector<std::vector<double>>& _cost_map, Cor start, Cor goal, bool isObstacleSudden) {
	Node* middle_start = new Node(), * middle_goal = new Node();
	bool find_path = false;
	// initialize
	cost_map = _cost_map;

	// in case present position or goal is on obstacle
	if(isobstacle(start)){
		threshold = 120;
	}
	else if(isobstacle(goal)){
		threshold = 120;
	}
	else threshold = 100;

	// straight check
	if(straightCheck(start, goal)){
		path.push_back(start);
		path.push_back(goal);
		return false;
	}

	/*
	if(isObstacleSudden){
		cout << "shit\n";
		return;
	}
	*/

	Tree start_tree = Tree(size); 
	start_tree.insert(Node(start));
	Tree goal_tree = Tree(size); goal_tree.insert(Node(goal));
	// iteration start
	int t = clock();
	srand(t); ROS_INFO("srand %d",t);
	for (int i = 0; (!find_path || i < iternum) && i < 1000 ; i++) {
		Cor q_rand = random_point();
		bool check_start = false, check_goal = false;

		// rrt star for start_tree
debug(true); //0
		Cor q_near_start = nearest(start_tree, q_rand);
debug();	//1
		Cor q_new_start = steer(q_near_start, q_rand, check_start);
debug();	//2
		std::vector<Node*> Q_near_start;
debug();	//3
		Q_near_start.reserve(1000);
debug();	//4
		near(Q_near_start, start_tree, q_new_start);
debug();	//5
		Node q_newnode_start = chooseParent(Q_near_start, q_new_start);
debug();	//6
		Node* q_newnode_ptr_start = start_tree.insert(q_newnode_start);
debug();	//7
		q_newnode_ptr_start->parent->insert_child(q_newnode_ptr_start);
debug();	//8
		rewire(Q_near_start, q_newnode_ptr_start, start_tree);
debug();	//9

		// rrt star for goal_tree
		Cor q_near_goal = nearest(goal_tree, q_rand);
debug();	//10
		Cor q_new_goal = steer(q_near_goal, q_rand, check_goal);
debug();	//11
		std::vector<Node*> Q_near_goal;
debug();	//12
		Q_near_goal.reserve(1000);
debug();	//13
		near(Q_near_goal, goal_tree, q_new_goal);
debug();	//14
		Node q_newnode_goal = chooseParent(Q_near_goal, q_new_goal);
debug();	//15
		Node* q_newnode_ptr_goal = goal_tree.insert(q_newnode_goal);
debug();	//16
		q_newnode_ptr_goal->parent->insert_child(q_newnode_ptr_goal);
debug();	//17
		rewire(Q_near_goal, q_newnode_ptr_goal, goal_tree);
debug();	//18

		// if (q_rand == q_new_start == q_new_goal),then try connect = path found
		if (check_start && check_goal) {
debug();	//19
			pathOptimization(q_newnode_ptr_start,start_tree);
debug();	//20
			pathOptimization(q_newnode_ptr_goal,goal_tree);
debug();	//21
			if (find_path) {

				double current_path_cost = middle_start->cost_sum + middle_goal->cost_sum;
				double new_path_cost = q_newnode_ptr_start->cost_sum + q_newnode_ptr_goal->cost_sum;
				if (current_path_cost > new_path_cost) {
					middle_start = q_newnode_ptr_start;
					middle_goal = q_newnode_ptr_goal;
				}
			}
			else {
				find_path = true;
				middle_start = q_newnode_ptr_start;
				middle_goal = q_newnode_ptr_goal;
			}
		}
		
	}

/*
	bool rrt_image = false;
	if (rrt_image) {
		cv::Mat image = cv::Mat::zeros(200, 200, CV_8UC3);
		int sz = start_tree.arrsize();
		for (int i = 0; i < sz; i++) {
			if (!start_tree[i].parent) continue;
			Cor a = start_tree[i].location;
			Cor b = start_tree[i].parent->location;
			line(image, cv::Point(a.x,a.y), cv::Point(b.x, b.y), cv::Scalar(0, 100, 200), 1, 8);
		}

		sz = goal_tree.arrsize();
		for (int i = 0; i < sz; i++) {
			if (!goal_tree[i].parent) continue;
			Cor a = goal_tree[i].location;
			Cor b = goal_tree[i].parent->location;
			line(image, cv::Point(a.x, a.y), cv::Point(b.x, b.y), cv::Scalar(200, 0, 100), 1, 8);
		}
		imshow("rrt tree", image);
	}
*/
	
	// cant find path
	if(!find_path) return true;
	
	// make path

	// from start to middle
	Node* cur = middle_start;
	while (cur->parent != nullptr) {
		cur = cur->parent;
		path.push_back(cur->location);
	}
	std::reverse(path.begin(), path.end());
	// from middle to goal
	cur = middle_goal;
	while (cur != nullptr) {
		path.push_back(cur->location);
		cur = cur->parent;
	}
	return true;
}


bool RRT::isobstacle(Cor a, Cor b)
{
	double dist = a.dist(b);
	int step_times{0};
	double x{a.x};
	double y{a.y};
	double cost{0};
	while (step_times*stepsize<dist)
	{
		if(threshold<=cost_map[static_cast<int>(x)][static_cast<int>(y)])
		{	
			return true;
		}
		else
		{
			x += (b.x-a.x)*stepsize/dist;
			y += (b.y-a.y)*stepsize/dist;
			step_times++;
		}
	}
	return false;
}

bool RRT::isobstacle(Cor c)
{
	if(threshold<=cost_map[static_cast<int>(c.x)][static_cast<int>(c.y)])
		return true;
	else
		return false;
}


