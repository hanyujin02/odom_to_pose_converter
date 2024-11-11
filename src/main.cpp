#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Core>
#include <Eigen/Dense>

ros::Publisher publisher;

void subscriberCallback(const nav_msgs::Odometry& odometryMsg)
{
	geometry_msgs::PoseStamped poseStamped;
	// poseStamped.header.frame_id = odometryMsg.header.frame_id;
	poseStamped.header.frame_id = "map";
	poseStamped.header.stamp = odometryMsg.header.stamp;
	Eigen::Matrix3d rm;
	rm << 1, 0, 0,
		  0,  1, 0,
		  0,  0, 1;
	//position
	Eigen::Vector3d pv(odometryMsg.pose.pose.position.x, odometryMsg.pose.pose.position.y, odometryMsg.pose.pose.position.z);
	Eigen::Vector3d rv = rm * pv;
	poseStamped.pose.position.x = rv.x();
	poseStamped.pose.position.y = rv.y();
	poseStamped.pose.position.z = rv.z();

	//orientation
	Eigen::Quaterniond q(odometryMsg.pose.pose.orientation.w, odometryMsg.pose.pose.orientation.x, odometryMsg.pose.pose.orientation.y, odometryMsg.pose.pose.orientation.z);
	Eigen::Matrix3d prm = q.toRotationMatrix();

	// Eigen::Vector3d I(1, 0, 0);
	// Eigen::Vector3d ori = prm * I;
	// Eigen::Vector3d front_pv = pv + ori;
	// front_pv = rm * front_pv;
	// Eigen::Vector3d new_ori = front_pv - rv;
	Eigen::Quaterniond q_base(Eigen::AngleAxisd(0*M_PI/180,Eigen::Vector3d::UnitX()));
	Eigen::Quaterniond q_camera_init(Eigen::AngleAxisd(0.0*M_PI,Eigen::Vector3d::UnitZ()));
	Eigen::Matrix3d m_q_base = q_base.toRotationMatrix();
	Eigen::Matrix3d m_q_camera_init = q_camera_init.toRotationMatrix();

	Eigen::Matrix3d m_q_map = m_q_base * m_q_camera_init * prm;
	Eigen::Matrix3d m_rotated = rm * m_q_map * rm.inverse();
	Eigen::Quaterniond new_q(m_rotated);
	

	// Normalization might not be necessary if you're already working with unit vectors, but it doesn't hurt.
	new_q.normalized();
	poseStamped.pose.orientation.x = new_q.x();
	poseStamped.pose.orientation.y = new_q.y();
	poseStamped.pose.orientation.z = new_q.z();
	poseStamped.pose.orientation.w = new_q.w();

	// poseStamped.pose = odometryMsg.pose.pose;
	// poseStamped.pose.orientation = odometryMsg.pose.pose.orientation;
	publisher.publish(poseStamped);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "odom_to_pose_converter_node");
	ros::NodeHandle nodeHandle;
	
	ros::Subscriber subscriber = nodeHandle.subscribe("/Odometry", 1000, subscriberCallback);
	publisher = nodeHandle.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 1000);
	
	ros::spin();
	
	return 0;
}
