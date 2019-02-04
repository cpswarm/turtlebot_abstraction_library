#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <swarmros/String.h>
#include <cpswarm_msgs/TargetAssignmentEvent.h>
#include <cpswarm_msgs/TargetAssignedEvent.h>
#include <turtlebot_compute_cost/ComputeMapCostAction.h>
#include <move_base_msgs/MoveBaseAction.h>

using namespace std;

typedef actionlib::SimpleActionServer<turtlebot_compute_cost::ComputeMapCostAction> ServerMap;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_base_client;

//Variables
ros::Publisher cost_pub;
ros::Publisher goal_pub;

ros::Subscriber global_pos_sub;
ros::Subscriber selection_sub;
ros::Subscriber UUID_sub;

geometry_msgs::PoseWithCovarianceStamped map_position;
cpswarm_msgs::TargetAssignedEvent target_assigned;
string UUID;			//UUID of the CPS
bool position_received; 	//TRUE when gps or map position has been received
bool cps_selected;		//TRUE when cps_selected answer has been received
int target_id;
string move_base_action;

//**********************************************************************************************************
/**
 * Return euclidean distance in meters
 */
double euclidean_distance(double x0, double y0, double x1, double y1)
{
	return sqrt(pow(x1 - x0, 2) + pow(y1 - y0, 2));
}

//**************** CALLBACKS *******************************************************************************
void mapPosition_cb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{
	map_position = *msg;
	position_received = true;
}

void cpsSelected_cb(const cpswarm_msgs::TargetAssignedEvent::ConstPtr &msg)
{
	if (msg->target_id == target_id)
	{
		target_assigned = *msg;
		cps_selected = true;
		ROS_INFO("CMP_COST - Target ASSIGNED event callback");
	}
}

void UUID_cb(const swarmros::String::ConstPtr &msg)
{
	UUID = msg->value;
	ROS_INFO("CMP_COST - Received node UUID: %s", UUID.c_str());
}

void execute_map_cb(const turtlebot_compute_cost::ComputeMapCostGoal::ConstPtr &goal, ServerMap *as)
{
	ROS_INFO("CMP_COST - Executing ComputeCost action..");
	//Compute distance from target
	double distance = euclidean_distance(goal->local_pose.pose.position.x, goal->local_pose.pose.position.y, map_position.pose.pose.position.x, map_position.pose.pose.position.y);
	target_id = goal->target_id;
	cps_selected = false;

	ROS_INFO("CMP_COST - Computed cost for target %d, value: %.5f", target_id, distance);

	cpswarm_msgs::TargetAssignmentEvent target_assignement;
	target_assignement.header.stamp = ros::Time::now();
	target_assignement.header.frame_id = "";
	target_assignement.swarmio.name = "cps_selection";
	target_assignement.swarmio.node = goal->sender_UUID;
	target_assignement.id = target_id;
	target_assignement.cost = distance;

	ros::Rate rate(1.0);
	//Wait for answer
	while (ros::ok() && !as->isPreemptRequested() && !cps_selected)
	{
		ROS_INFO("CMP_COST - Waiting for cps_selected event");
		cost_pub.publish(target_assignement);
		ros::spinOnce();
		rate.sleep();
	}

	bool selected = (target_assigned.cps_id.compare(UUID) == 0);
	ROS_INFO("CMP_COST - TargetAssignedEvent received: %d", selected);

	//Reset variables
	cps_selected = false;
	target_id = -1;

	if (as->isPreemptRequested())
	{
		as->setPreempted();
	}
	else if (selected)
	{
		//return succeed
		turtlebot_compute_cost::ComputeMapCostResult result;
		result.target_id = target_assigned.target_id;
		result.local_pose = goal->local_pose;
		as->setSucceeded(result);
	}
	else
	{
		as->setAborted();
	}
}
//*******************************************************************************************************************

int main(int argc, char **argv)
{
	ros::init(argc, argv, "compute_cost");
	ros::NodeHandle nh;
	string target_cost_topic;
	nh.getParam(ros::this_node::getName() + "/target_cost_topic", target_cost_topic);
	string cps_selected_topic;
	nh.getParam(ros::this_node::getName() + "/cps_selected_topic", cps_selected_topic);
	string UUID_topic;
	nh.getParam(ros::this_node::getName() + "/UUID_topic", UUID_topic);
	string map_pose_topic;
	nh.getParam(ros::this_node::getName() + "/map_pose_topic", map_pose_topic);

	nh.getParam(ros::this_node::getName() + "/move_base_action", move_base_action);

	//Init variables
	UUID = "";
	position_received = false;
	cps_selected = false;
	target_id = -1;

	//Initialize subscribers

	ROS_INFO("Using map frame coordinates");
	global_pos_sub = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>(map_pose_topic, 10, mapPosition_cb);

	selection_sub = nh.subscribe<cpswarm_msgs::TargetAssignedEvent>(cps_selected_topic, 10, cpsSelected_cb);
	UUID_sub = nh.subscribe<swarmros::String>(UUID_topic, 1, UUID_cb);
	//Initialize publishers
	cost_pub = nh.advertise<cpswarm_msgs::TargetAssignmentEvent>(target_cost_topic, 10);

	ros::Rate rate(10.0);
	// wait for gps position and UUID
	while (ros::ok() && (!position_received || UUID.compare("") == 0))
	{
		ROS_DEBUG_ONCE("CMP_COST - Waiting for position OR UUID..");
		ros::spinOnce();
		rate.sleep();
	}

	ServerMap server(nh, "cmd/compute_cost", boost::bind(&execute_map_cb, _1, &server), false);
	server.start();
	ROS_INFO("CMP_COST - Starting action server");
	ROS_INFO("CMP_COST - ComputeCost action available");
	ros::spin();
	return 0;
}
