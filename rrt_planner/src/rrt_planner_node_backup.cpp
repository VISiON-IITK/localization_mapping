#include <ros/ros.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// cost = 100 => direct obstacle
// cost = 99 => obstacle within inscribed radius
#define COLLISION_THRESHOLD	99

#define LMS 400
#define FRONT_EXTRA 6	// 0.275/0.05
#define WAYPOINT_RADIUS 2.0

class Node 	// a state (x, y, psi) and its covariance (P)
{
public:
	int x = 0;
	int y = 0;
	double psi = 0.0;

	bool close_to_goal = false;
	double travel_time = 0.0;	// estimated time to reach node from root

	Node* parent = NULL;		// parent node pointer within the tree
};

// RRT utility functions
inline bool angleCheck(const Node& parent, const Node& sample, double max_angle);
inline bool checkConstraint(const Node& current, const nav_msgs::OccupancyGrid& cfg_space);
bool localPlanner(const Node& parent, Node& sample, const nav_msgs::OccupancyGrid& cfg_space);
void randSample(Node& sample, const nav_msgs::OccupancyGrid& cfg_space, const Node& centre);

// Callback functions
void cfgSpaceCallback(const nav_msgs::OccupancyGridConstPtr& occupancy_msg,
	nav_msgs::OccupancyGrid& cfg_space, bool& cfg_space_received);
void odomCallback(const nav_msgs::OdometryConstPtr& odom_msg,
	geometry_msgs::PoseStamped& initial_pose, bool& odom_received);

// Common math functions
double angWrap(double x);
inline double distance(double x1, double y1, double x2, double y2);
inline int sgn(double x);

int main(int argc, char **argv)
{
	// Initializing node
	ros::init(argc, argv, "rrt_planner");
	ros::NodeHandle nh;
	ros::Time t;


	// Getting parameters
	int max_nodes = 10000, max_iterations = 1000000, num_paths = 10;
	double delta_step = 0.5, max_angle = 30.0, vehicle_speed = 0.5, vehicle_turn_rate = 1.0;;
	std::vector<double> goal_position (2, 0.0);
	double checking_freq = 10.0, publishing_freq = 0.5;
	nh.param("max_nodes", max_nodes, 10000);						// maximum required nodes in tree
	nh.param("max_iterations", max_iterations, 1000000);			// maximum allowable iterations for CC-RRT
	nh.param("num_paths", num_paths, 50);							// no. of paths to consider for selecting best one
	nh.param("delta_step", delta_step, (double)0.5);				// RRT step size, in metres
	nh.param("max_angle", max_angle, (double)30.0);					// maximum desired turning angle, in degrees
	max_angle = max_angle*M_PI/180.0;
	nh.param("vehicle_speed", vehicle_speed, (double)0.5);			// expected speed
	nh.param("vehicle_turn_rate", vehicle_turn_rate, (double)1.0);	// expected turn rate
	nh.getParam("goal_position", goal_position);					// goal co-ordinates (x, y) in m (in base_link)
	nh.param("checking_freq", checking_freq, (double)10.0);			// checking frequency, in hz
	nh.param("publishing_freq", publishing_freq, (double)0.5);		// checking frequency, in hz

	// Creating objects for storing initial position and configuration space
	geometry_msgs::PoseStamped initial_pose, initial_pose_map, goal_pose, goal_pose_map;
	goal_pose.header.frame_id = "map";
	goal_pose.pose.position.x = goal_position[0];
	goal_pose.pose.position.y = goal_position[1];
	goal_pose.pose.orientation.w = 1.0;								// for normalizing quaternion
	nav_msgs::OccupancyGrid cfg_space;
	bool odom_received = false, cfg_space_received = false;


	// Creating subscriber and publisher objects
	ros::Subscriber sub_odom = nh.subscribe<nav_msgs::Odometry>("/odom", 1,
		boost::bind(odomCallback, _1, boost::ref(initial_pose), boost::ref(odom_received)));
	ros::Subscriber sub_cfg = nh.subscribe<nav_msgs::OccupancyGrid>("/cfg_space", 1,
		boost::bind(cfgSpaceCallback, _1, boost::ref(cfg_space), boost::ref(cfg_space_received)));

	ros::Publisher pub_nodes = nh.advertise<nav_msgs::OccupancyGrid>("/nodes", 1);	// for vizualization
	ros::Publisher pub_path = nh.advertise<nav_msgs::Path>("/path/smooth", 1);

	tf2_ros::Buffer tfBuffer;
	tf2_ros::TransformListener tfListener(tfBuffer);


	ros::Rate publishing_rate(publishing_freq);
	while (ros::ok())
	{
		// Getting configuration space and initial map
		ros::Rate checking_rate(checking_freq);
		while (ros::ok() && (!odom_received || !cfg_space_received))
		{
			ros::spinOnce();
			checking_rate.sleep();
		}
		ROS_INFO_STREAM("Initial pose and configuration space received.");

		// Transforming initial pose and goal pose to the map frame
		geometry_msgs::TransformStamped odom_to_map, map_to_odom;
		while(ros::ok())
		{	
			try
			{
				// For transforming the coordinates of a fixed point from frame_a to frame_b, we need
				// to use lookupTransform("frame_b", "frame_a",ros::Time(0))
				odom_to_map =
					tfBuffer.lookupTransform(cfg_space.header.frame_id, initial_pose.header.frame_id, ros::Time(0));
				map_to_odom =
					tfBuffer.lookupTransform(initial_pose.header.frame_id, cfg_space.header.frame_id, ros::Time(0));
				break;
			}
			catch (tf2::TransformException &ex)
			{
				ROS_WARN("%s",ex.what());
				ros::Duration(1.0).sleep();
				continue;
			}
		}
		tf2::doTransform(initial_pose, initial_pose_map, odom_to_map);

		geometry_msgs::TransformStamped utm_to_map;
		while(ros::ok())
		{	
			try
			{
				// For transforming the coordinates of a fixed point from frame_a to frame_b, we need
				// to use lookupTransform("frame_b", "frame_a",ros::Time(0))
				utm_to_map =
					tfBuffer.lookupTransform(cfg_space.header.frame_id, goal_pose.header.frame_id, ros::Time(0));
				break;
			}
			catch (tf2::TransformException &ex)
			{
				ROS_WARN("%s",ex.what());
				ros::Duration(1.0).sleep();
				continue;
			}
		}
		tf2::doTransform(goal_pose, goal_pose_map, utm_to_map);

		if (distance(goal_pose_map.pose.position.x, goal_pose_map.pose.position.y,
			initial_pose_map.pose.position.x, initial_pose_map.pose.position.y) <
			WAYPOINT_RADIUS)
			break;

		// Converting everything to map units (m->cells, 3D->2D, quaternion->yaw)
		Node* root = new Node;
		root->x = (initial_pose_map.pose.position.x - cfg_space.info.origin.position.x)/cfg_space.info.resolution;
		root->y = (initial_pose_map.pose.position.y - cfg_space.info.origin.position.y)/cfg_space.info.resolution;

		tf2::Quaternion q_temp;					// temp var for quaternion->yaw
		tf2::Matrix3x3 m_temp;					// temp var for quaternion->yaw
		double roll_temp, pitch_temp, yaw_temp;	// temp var for quaternion->yaw
			q_temp = tf2::Quaternion(
			initial_pose_map.pose.orientation.x,
			initial_pose_map.pose.orientation.y,
			initial_pose_map.pose.orientation.z,
			initial_pose_map.pose.orientation.w);
		m_temp = tf2::Matrix3x3(q_temp);
		m_temp.getRPY(roll_temp, pitch_temp, yaw_temp);
		root->psi = yaw_temp;

		Node* goal = new Node;
		goal->x = (goal_pose_map.pose.position.x - cfg_space.info.origin.position.x)/cfg_space.info.resolution;
		goal->y = (goal_pose_map.pose.position.y - cfg_space.info.origin.position.y)/cfg_space.info.resolution;

		delta_step = delta_step/cfg_space.info.resolution;
		vehicle_speed = vehicle_speed/cfg_space.info.resolution;
		vehicle_turn_rate = vehicle_turn_rate;	// no angular scaling

		//////////////////////
		// Main CC-RRT Loop //
		//////////////////////

		ROS_INFO_STREAM("Initializing RRT...");
		srand(time(NULL));
		std::vector<Node*> tree;
		tree.push_back(root);

		int num_iterations = 0;
		double current_distance, min_distance, min_index;
		Node* sample;
		while (tree.size() < max_nodes && num_iterations < max_iterations)
		{
			// Generating a random sample in freespace (only x & y are set)
			sample = new Node;
			randSample(*(sample), cfg_space, *(root));

			// Finding nearest neighbour and setting it as the parent
			min_distance = std::numeric_limits<double>::infinity();
			for (int i = 0; i < tree.size(); i++)
			{
				current_distance = distance(tree[i]->x, tree[i]->y, sample->x, sample->y);
				if (min_distance > current_distance)
				{
					min_distance = current_distance;
					min_index = i;
				}
			}
			sample->parent = tree[min_index];

			// Running some checks in increasing order of computational complexity
			bool all_checks_passed = false;
			// Check 1: Does the sample co-incide with its parent?
			if (min_distance > 0)
			{
				// Updating psi and travel_time based on distance and heading from parent node
				sample->psi = atan2(sample->y - sample->parent->y, sample->x - sample->parent->x);
				sample->travel_time = sample->parent->travel_time +
					std::abs(angWrap(sample->psi - sample->parent->psi))/vehicle_turn_rate +
					min_distance/vehicle_speed;

				// Check 2: Is the steer angle within the acceptable range?
				if (angleCheck(*(sample->parent), *(sample), max_angle))
				{
					// Check 3: Is the path from parent to sample feasible as per chance constraints?
					// (also updates sample->P)
					if (localPlanner(*(sample->parent), *(sample), cfg_space))
					{
						all_checks_passed = true;
					}
				}

			}

			if (all_checks_passed)
			{
				if (min_distance > delta_step)
				{
					// Truncating edge length to delta_step
					sample->x = sample->parent->x +
						round((double)(sample->x - sample->parent->x)*delta_step/min_distance);
					sample->y = sample->parent->y +
						round((double)(sample->y - sample->parent->y)*delta_step/min_distance);
					// Re-updating the values affected by this change
					sample->travel_time = sample->parent->travel_time +
						std::abs(angWrap(sample->psi - sample->parent->psi))/vehicle_turn_rate +
						delta_step/vehicle_speed;
				}
				tree.push_back(sample);
			}
			else
			{
				delete sample;
			}

			num_iterations++;
			ROS_INFO_STREAM_THROTTLE(0.5, num_iterations<<" iterations completed.");
		}
		ROS_INFO_STREAM("Tree with "<<tree.size()<<" nodes generated.");

		////////////////////////
		// End of CC-RRT Loop //
		////////////////////////


		// Makeshift method for visualizing all nodes in the tree
		nav_msgs::OccupancyGrid cc_rrt_nodes = cfg_space;
		for (int i = 0; i < cc_rrt_nodes.data.size(); i++)
			cc_rrt_nodes.data[i] = -1;
		// costmap gives colours from (1->98)
		for (int i = 0; i < tree.size(); i++)
		{
			cc_rrt_nodes.data[tree[i]->y*cc_rrt_nodes.info.width + tree[i]->x] = 50;
		}
		pub_nodes.publish(cc_rrt_nodes);


		ROS_INFO_STREAM("Selecting best path...");
		// Choosing best path
		std::vector<Node*> closest_nodes;
		for (int i = 0; i < std::min(num_paths, (int)tree.size()); i++)
		{
			min_distance = std::numeric_limits<double>::infinity();
			for (int j = 0; j < tree.size(); j++)
			{
				current_distance = distance(tree[j]->x, tree[j]->y, goal->x, goal->y);
				if (!tree[j]->close_to_goal && min_distance > current_distance)
				{
					min_distance = current_distance;
					min_index = j;
				}
			}
			tree[min_index]->close_to_goal = true;
			closest_nodes.push_back(tree[min_index]);
		}
		double min_time = std::numeric_limits<double>::infinity();
		for (int i = 0; i < closest_nodes.size(); i++)
		{
			if (min_time > closest_nodes[i]->travel_time)
			{
				min_time = closest_nodes[i]->travel_time;
				min_index = i;
			}
		}


		// Connecting best path to goal if possible
		goal->parent = closest_nodes[min_index];
		// Running some checks in increasing order of computational complexity
		bool all_checks_passed = false;
		// Check 1: Does the node co-incide with the goal?
		min_distance = distance(closest_nodes[min_index]->x, closest_nodes[min_index]->y, goal->x, goal->y);
		if (min_distance > 0)
		{
			// Updating psi and travel_time based on distance and heading from parent node
			goal->psi = atan2(goal->y - goal->parent->y, goal->x - goal->parent->x);
			goal->travel_time = goal->parent->travel_time +
				std::abs(angWrap(goal->psi - goal->parent->psi))/vehicle_turn_rate +
				min_distance/vehicle_speed;

			// Check 2: Is the path from parent to goal feasible as per chance constraints?
			// (also updates goal->P)
			if (localPlanner(*(goal->parent), *(goal), cfg_space))
			{
				all_checks_passed = true;
			}
		}
		Node* path_end = NULL;
		if (all_checks_passed)
		{
			path_end = goal;
			ROS_INFO_STREAM("Path connected to goal.");
		}
		else
			path_end = closest_nodes[min_index];


		// Tracing the path back to the initial position
		std::vector<Node*> path_rev;
		Node* current = path_end;
		while (current != NULL)
		{
			path_rev.push_back(current);
			current = current->parent;
		}


		// Reversing the path, transforming it back into real-world units & into the map frame before publishing
		geometry_msgs::PoseStamped current_pose, current_pose_odom;
		nav_msgs::Path path_msg;
		t = cfg_space.header.stamp;
		current_pose.header = cfg_space.header;
		path_msg.header = initial_pose.header;
		for (int i = path_rev.size()-1; i >= 0; i--)
		{
			current_pose.pose.position.x = path_rev[i]->x*cfg_space.info.resolution + cfg_space.info.origin.position.x;
			current_pose.pose.position.y = path_rev[i]->y*cfg_space.info.resolution + cfg_space.info.origin.position.y;
			current_pose.pose.orientation.z = std::sin(path_rev[i]->psi/2);
			current_pose.pose.orientation.w = std::cos(path_rev[i]->psi/2);
			tf2::doTransform(current_pose, current_pose_odom, map_to_odom);
			path_msg.poses.push_back(current_pose_odom);
			if (i > 0)
			{
				// Intermediate pose for turning first (followed by straight-line-motion)
				current_pose.pose.orientation.z = std::sin(path_rev[i-1]->psi/2);
				current_pose.pose.orientation.w = std::cos(path_rev[i-1]->psi/2);
				tf2::doTransform(current_pose, current_pose_odom, map_to_odom);
				path_msg.poses.push_back(current_pose_odom);
			}
		}


		// Smoothing Path
		double length_prev = 0.0, length_next;
		for (int i=0; i+2<path_msg.poses.size(); i=i+2)
		{
			length_next = distance(path_msg.poses[i+2].pose.position.x, path_msg.poses[i+2].pose.position.y, path_msg.poses[i+1].pose.position.x, path_msg.poses[i+1].pose.position.y);
			if (length_prev > 0.2 && length_next > 0.2)
			{
				path_msg.poses[i].pose.position.x -= 0.1*(path_msg.poses[i].pose.position.x - path_msg.poses[i-1].pose.position.x)/length_prev;
				path_msg.poses[i].pose.position.y -= 0.1*(path_msg.poses[i].pose.position.y - path_msg.poses[i-1].pose.position.y)/length_prev;
				path_msg.poses[i+1].pose.position.x += 0.1*(path_msg.poses[i+2].pose.position.x - path_msg.poses[i+1].pose.position.x)/length_next;
				path_msg.poses[i+1].pose.position.y += 0.1*(path_msg.poses[i+2].pose.position.y - path_msg.poses[i+1].pose.position.y)/length_next;
			}
			length_prev = length_next;
		}
		pub_path.publish(path_msg);

		odom_received = false;
		cfg_space_received = false;
		for (int i = 0; i < tree.size(); i++)
			delete tree[i];
		delete goal;	// root, current, path_end already included in tree

		publishing_rate.sleep();
	}
	// Exiting Node
	ROS_INFO_STREAM("Shutting down node...");
	ros::shutdown();
	return 0;
}

///////////////////////////
// RRT utility functions //
///////////////////////////

inline bool angleCheck(const Node& parent, const Node& sample, double max_angle)
{
	return std::abs(angWrap(sample.psi - parent.psi)) <= max_angle;
}

inline bool checkConstraint(const Node& current, const nav_msgs::OccupancyGrid& cfg_space)
{
	return (COLLISION_THRESHOLD - cfg_space.data[current.y*cfg_space.info.width + current.x]) > 0;
}

bool localPlanner(const Node& parent, Node& sample,	const nav_msgs::OccupancyGrid& cfg_space)
{
	// Initial State
	Node current = parent;
	current.psi = sample.psi;
	if (!checkConstraint(current, cfg_space))
		return false;

	int num_steps = distance(parent.x, parent.y, sample.x, sample.y) + FRONT_EXTRA;
	double curr_x = current.x, curr_y = current.y;
	double x_incr = std::cos(current.psi), y_incr = std::sin(current.psi);
	for (int i = 0; i < num_steps; i++)
	{
		curr_x += x_incr;
		curr_y += y_incr;
		current.x = round(curr_x);
		current.y = round(curr_y);
		if (!checkConstraint(current, cfg_space))
			return false;
	}

	// Updating sample covariance
	return true;
}

void randSample(Node& sample, const nav_msgs::OccupancyGrid& cfg_space, const Node& centre)
{
	// Gives random node in freespace=
	int map_width = cfg_space.info.width;
	int map_height = cfg_space.info.height;
	while(true)
	{
		sample.x = centre.x - LMS/2 + rand()%LMS;
		sample.y = centre.y - LMS/2 + rand()%LMS;
		if (cfg_space.data[sample.y*map_width + sample.x] < COLLISION_THRESHOLD)
			break;
	}
	
	return;
}

////////////////////////
// Callback functions //
////////////////////////

void odomCallback(const nav_msgs::OdometryConstPtr& odom_msg,
	geometry_msgs::PoseStamped& initial_pose, bool& odom_received)
{
	initial_pose.header = odom_msg->header;
	initial_pose.pose = odom_msg->pose.pose;

	odom_received = true;
}

void cfgSpaceCallback(const nav_msgs::OccupancyGridConstPtr& occupancy_msg,
	nav_msgs::OccupancyGrid& cfg_space, bool& cfg_space_received)
{
	cfg_space.header = occupancy_msg->header;
	cfg_space.info = occupancy_msg->info;
	cfg_space.data = occupancy_msg->data;

	cfg_space_received = true;
}

///////////////////////////
// Common math functions //
///////////////////////////

double angWrap(double x)
{
	while (x <= -M_PI)
		x = x + 2*M_PI;
	while (x > M_PI)
		x = x - 2*M_PI;
	return x;
}

inline double distance(double x1, double y1, double x2, double y2)
{
	return std::sqrt((x2-x1)*(x2-x1) + (y2-y1)*(y2-y1));
}

inline int sgn(double x)
{
	return (0.0 < x) - (x < 0.0);
}