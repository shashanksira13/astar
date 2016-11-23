#include "ros/ros.h"
#include "ros/console.h"
#include "tf/tf.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Quaternion.h"
#include "nav_msgs/Path.h"
#include "visualization_msgs/Marker.h"
#include "astar.h"
#include <teb_local_planner/TrajectoryMsg.h>


Pose start(0, 0, 1, 0,1);
Pose  goal(0, 0, 1, 0,0);
float v, w;

double h(Pose &p) {
	double dx = goal.x - p.x;
	double dy = goal.y - p.y;
	return sqrt(dx*dx + dy*dy);
}

inline bool insideObstacle(Pose &p) {
	// HACK: Dubin curves needed instead of artificially expanded obstacle size
	if(p.x < 15-w || p.x > 20+w || p.y < -5-w || p.y > 20+w)
		return false;
	if (p.y < -5-w || p.y > 5)
		return false;
	return true;
}

std::vector<Pose> neighbors(Pose &p) {
	std::vector<Pose> ns;
    Pose l(p, p.vel-1,  w);
    Pose r(p, p.vel-1, -w);
    Pose c(p, p.vel-1,  0);
    Pose l1(p, p.vel,  w);
    Pose r1(p, p.vel, -w);
    Pose c1(p, p.vel,  0);
    Pose l2(p, p.vel+1,  w);
    Pose r2(p, p.vel+1, -w);
    Pose c2(p, p.vel+1,  0);
    if(!insideObstacle(l)&&(l.vel>=1)&&(l.vel<3))
        ns.push_back(l);
      
    if(!insideObstacle(r)&&(r.vel>=1)&&(r.vel<3))
        ns.push_back(r);

    if(!insideObstacle(c)&&(c.vel>=1)&&(c.vel<3))
        ns.push_back(c);

    if(!insideObstacle(l)&&!insideObstacle(l1)&&(l1.vel>=1)&&(l1.vel<3))
        ns.push_back(l1);

    if(!insideObstacle(r)&&!insideObstacle(r1)&&(r1.vel>=1)&&(r1.vel<3))
        ns.push_back(r1);

    if(!insideObstacle(c)&&!insideObstacle(c1)&&(c1.vel>=1)&&(c1.vel<3))
        ns.push_back(c1);

    if(!insideObstacle(l)&&!insideObstacle(l2)&&!insideObstacle(l1)&&(l2.vel>=1)&&(l2.vel<3))
        ns.push_back(l);

    if(!insideObstacle(r)&&!insideObstacle(r2)&&!insideObstacle(r1)&&(r2.vel>=1)&&(r2.vel<3))
        ns.push_back(r);

    if(!insideObstacle(c)&&!insideObstacle(c2)&&!insideObstacle(c1)&&(c2.vel>=1)&&(c2.vel<3))
        ns.push_back(c);

	return ns;
}

double g(Pose &p1, Pose &p2) {
	return 1;
}

bool goalReached(Pose &p) {
	if((abs(goal.x - p.x) + abs(goal.y - p.y)) < 6.25
			&& abs(atan2(goal.oy, goal.ox) - atan2(p.oy, p.ox)) < 1)
		return true;

	return false;
}

int main(int argc, char **argv) {
	goal = Pose(atof(argv[1]), atof(argv[2]), atof(argv[3]), atof(argv[4]),1);
	v    = M_PI * atof(argv[5]);
	w    = atof(argv[5]) / 2;
	ros::init(argc, argv, "show_path");
	ros::NodeHandle n;
	ros::Publisher pub_path = n.advertise<nav_msgs::Path>("/path", 2048);
	ros::Publisher pub_obst = n.advertise<visualization_msgs::Marker>("/obstacle", 1);
    ros::Publisher pub_open_nodes = n.advertise<visualization_msgs::Marker>("/open_nodes", 1);

	ros::Publisher mPublisher;
  mPublisher = n.advertise<teb_local_planner::TrajectoryMsg>("planned_path", 1);

	visualization_msgs::Marker msg_obst;
	msg_obst.header.frame_id    = "/map";
	msg_obst.type               = visualization_msgs::Marker::CUBE;
	msg_obst.action             = visualization_msgs::Marker::ADD;
	msg_obst.pose.position.x    = 17.5;
	msg_obst.pose.position.y    = 7.5;
	msg_obst.pose.orientation.w = 1.0;
	msg_obst.scale.x            = 5;
	msg_obst.scale.y            = 25;
	msg_obst.scale.z            = 1.0;
	msg_obst.color.a            = 1.0;
	msg_obst.color.r            = 0.0;
	msg_obst.color.g            = 0.0;
	msg_obst.color.b            = 1.0;
	// std::vector<geometry_msgs::PoseStamped> plan;
			///create trajectoryMsg
	teb_local_planner::TrajectoryMsgPtr trajectoryMsgPtr = boost::make_shared<teb_local_planner::TrajectoryMsg>();
	trajectoryMsgPtr->header.frame_id = "map";
	trajectoryMsgPtr->header.stamp = ros::Time::now();

	AStar astar(start, goal, &neighbors, &g, &h, &goalReached);
	Node *node = astar.open.top();

  teb_local_planner::TrajectoryPointMsg point;

	while(node != NULL) {
		ROS_INFO("PATH: %.1f, %.1f; %.1f, %.1f, %.1f", node->pose.x, node->pose.y, node->pose.ox, node->pose.oy, node->pose.vel);
		teb_local_planner::TrajectoryPointMsg step;
		tf::Quaternion quat     = tf::createQuaternionFromYaw(atan2(node->pose.oy, node->pose.ox));
		step.pose.position.x    = node->pose.x;
		step.pose.position.y    = node->pose.y;
		step.pose.position.z = 0;
		step.pose.orientation.x = quat.x();
		step.pose.orientation.y = quat.y();
		step.pose.orientation.z = quat.z();
		step.pose.orientation.w = quat.w();
		step.velocity.linear.x = node->pose.vel;
		trajectoryMsgPtr->trajectory.push_back(step);
		node = node->parent;
	}

	ROS_INFO("Nodes in OPEN: %lu", astar.open.size());
	  // visualization_msgs::Marker msg_open;
	visualization_msgs::Marker open_nodes;
	open_nodes.id = 1;
	open_nodes.header.frame_id = "/map";
	open_nodes.header.stamp = ros::Time::now();
	open_nodes.action = visualization_msgs::Marker::ADD;
	open_nodes.pose.orientation.w = 1.0;
	open_nodes.type = visualization_msgs::Marker::SPHERE_LIST;
	open_nodes.scale.x = 0.3;
	open_nodes.scale.y = 0.3;
	open_nodes.scale.z = 0.3;
	open_nodes.color.r = 0.0;
	open_nodes.color.g = 0.0;
	open_nodes.color.b = 0.6;
	open_nodes.color.a = 1;

	// nav_msgs::Path msg_path;
	// msg_path.poses.resize(plan.size());
	// msg_path.header.frame_id = "map";
	// msg_path.header.stamp = plan[0].header.stamp;

	// for(int i = 0; i < plan.size(); ++i)
	// 	msg_path.poses[i] = plan[i];
   //  while(!astar.open.empty())
   //  {
			// geometry_msgs::Point p;
			// p.x = astar.open.top()->pose.x;
			// p.y = astar.open.top()->pose.y;
			// open_nodes.points.push_back(p);
   //    astar.open.pop();

   //  }
    	ros::Rate rate(1);


	while (ros::ok()) {
				mPublisher.publish(trajectoryMsgPtr);	
        // pub_path.publish(msg_path);
        pub_obst.publish(msg_obst);
       // pub_open_nodes.publish(open_nodes);
        ros::spinOnce();
        rate.sleep();
	}

	return 0;
}
