#ifndef ASTAR_H_
#define ASTAR_H_

#include <vector>
#include <queue>
#include <math.h>

class Pose {
public:
	float x, y, ox, oy,vel;

	Pose(float _x, float _y, float _ox, float _oy,float _vel) :
		x(_x), y(_y), ox(_ox), oy(_oy),vel(_vel) {};

	Pose(Pose &p, float v, float w) {
		ox = p.oy>0?p.ox-w:p.oy<0?p.ox+w:p.ox==1?p.ox-w:p.ox+w;
		oy = p.ox>0?p.oy+w:p.ox<0?p.oy-w:p.oy==1?p.oy-w:p.oy+w;
		float n = sqrt(ox*ox + oy*oy);
		x  = p.x + v * ox / n;
		y  = p.y + v * oy / n;
		vel=v;
	};
};

class Node {
public:
	Pose pose;
	Node *parent;
	double costs_g, costs_f;

	Node(Pose &_pose, Node *_parent,
			double (*g)(Pose &start, Pose &goal),
			double (*h)(Pose &pose));
	Node(Pose &_pose, double (*h)(Pose &pose));
};

class C {
public:
	bool operator()(Node *n1, Node *n2) {
		return n1->costs_f > n2->costs_f;
	}
};

class AStar {
public:
	std::priority_queue<Node*, std::vector<Node*>, C> open;

	AStar(Pose &pose_start, Pose &pose_goal,
			std::vector<Pose> (*neighbors)(Pose &parent),
			double (*g)(Pose &start, Pose &goal),
			double (*h)(Pose &pose),
			bool (*goalReached)(Pose &pose));
};

#endif /* ASTAR_H_ */
