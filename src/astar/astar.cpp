#include "astar.h"

#include <stdio.h>
#include <stdlib.h>

Node::Node(Pose &_pose, Node *_parent,
		double (*g)(Pose &start, Pose &goal),
		double (*h)(Pose &pose)) :
			pose(_pose),
			parent(_parent)
{
	costs_g = parent->costs_g + (*g)(parent->pose, pose);
	costs_f = costs_g + (*h)(pose);
}

Node::Node(Pose &_pose, double (*h)(Pose &pose)) :
			pose(_pose),
			parent(NULL),
			costs_g(0)
{
	costs_f = (*h)(pose);
}

AStar::AStar(Pose &pose_start, Pose &pose_goal,
		std::vector<Pose> (*neighbors)(Pose &parent),
		double (*g)(Pose &start, Pose &goal),
		double (*h)(Pose &pose),
		bool (*goalReached)(Pose &pose)) {
	Node *n = new Node(pose_start, h);
	open.push(n);

	while(!open.empty()) {
		n = open.top();

		if((*goalReached)(n->pose))
			return;

		open.pop();
		std::vector<Pose> nbs = (*neighbors)(n->pose);

		for(std::vector<Pose>::iterator i=nbs.begin(); i!=nbs.end(); ++i) {
			Node *nb = new Node(*i, n, g, h);
			open.push(nb);
		}
	}
}
