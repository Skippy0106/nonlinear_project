
#include <ros/ros.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Imu.h"
#include "forceest.h"
#include "geometry_msgs/PoseStamped.h"
#include <tf/transform_datatypes.h>
#include "geometry_msgs/Point.h"
#include "math.h"
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Dense"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/Point.h"
#include "mavros_msgs/RCOut.h"
#include "sensor_msgs/MagneticField.h"
#include "sensor_msgs/BatteryState.h"
#include <string>
#include <gazebo_msgs/ModelStates.h>
#include "geometry_msgs/WrenchStamped.h"
#include <random>
#include <nav_msgs/Odometry.h>



#define l 0.25
#define k 0.02
std::string model_name;
int drone_flag;
forceest forceest1(statesize,measurementsize);
geometry_msgs::Point force1,force2,pose,vel;
sensor_msgs::Imu drone2_imu;

nav_msgs::Odometry odometry;
geometry_msgs::PoseStamped payload_pose;
geometry_msgs::TwistStamped payload_vel;


void pose_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
	odometry = *msg;
	payload_pose.pose = odometry.pose.pose;
	payload_vel.twist = odometry.twist.twist;

}





int main(int argc, char **argv)
{
	ros::init(argc, argv, "cooperate_ukf_estimate");
	ros::NodeHandle nh;

	ros::Subscriber pos_sub = nh.subscribe<nav_msgs::Odometry>("/payload/position", 4, pose_cb);

	ros::Publisher pose_pub = nh.advertise<geometry_msgs::Point>("pose_estimate", 2);
    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Point>("vel_estimate", 2);
    ros::Publisher force1_pub = nh.advertise<geometry_msgs::Point>("force1_estimate", 2);
    ros::Publisher force2_pub = nh.advertise<geometry_msgs::Point>("force2_estimate", 2);
	ros::Rate loop_rate(50);

	double measure_ex, measure_ey, measure_ez;
	double sum_pwm;
	int count = 1;
	Eigen::MatrixXd mnoise;
	mnoise.setZero(measurementsize,measurementsize);
	mnoise   = 3e-3*Eigen::MatrixXd::Identity(measurementsize, measurementsize);

	mnoise(mp_x,mp_x) = 1e-4;
	mnoise(mp_y,mp_y) = 1e-4;
	mnoise(mp_z,mp_z) = 1e-4;
	mnoise(mv_x,mv_x) = 1e-2;
	mnoise(mv_y,mv_y) = 1e-2;
	mnoise(mv_z,mv_z) = 1e-2;

	mnoise(me_x,me_x) = 1;//1
	mnoise(me_y,me_y) = 1;
	mnoise(me_z,me_z) = 1;

	forceest1.set_measurement_noise(mnoise);

	Eigen::MatrixXd pnoise;
	pnoise.setZero(statesize,statesize);
	pnoise(p_x,p_x) = 1e-2;
	pnoise(p_y,p_y) = 1e-2;
	pnoise(p_z,p_z) = 1e-2;
	pnoise(v_x,v_x) = 1e-2;
	pnoise(v_y,v_y) = 1e-2;
	pnoise(v_z,v_z) = 1e-2;

	pnoise(e_x,e_x) = 0.005;//0.5,調小beta收斂較快
	pnoise(e_y,e_y) = 0.005;
	pnoise(e_z,e_z) = 0.005;

	pnoise(F1_x,F1_x) = 0.15;
	pnoise(F1_y,F1_y) = 0.15;
	pnoise(F1_z,F1_z) = 0.15;

    pnoise(F2_x,F2_x) = 0.15;
	pnoise(F2_y,F2_y) = 0.15;
	pnoise(F2_z,F2_z) = 0.15;


	forceest1.set_process_noise(pnoise);



	Eigen::MatrixXd measurement_matrix;
	measurement_matrix.setZero(measurementsize,statesize);

	measurement_matrix(mp_x,p_x) = 1;
	measurement_matrix(mp_y,p_y) = 1;
	measurement_matrix(mp_z,p_z) = 1;


	measurement_matrix(mv_x,v_x) = 1;
	measurement_matrix(mv_y,v_y) = 1;
	measurement_matrix(mv_z,v_z) = 1;

	measurement_matrix(me_x,e_x) = 1;//1,調小，beta會劇烈震盪
	measurement_matrix(me_y,e_y) = 1;
	measurement_matrix(me_z,e_z) = 1;

	forceest1.set_measurement_matrix(measurement_matrix);
	double start_time;

	while(ros::ok()) {

		const double mean = 0.0;
		const double stddev = 0.1;
		std::default_random_engine generatorx,generatory,generatorz;
		std::normal_distribution<double> distx(mean,stddev);
		std::normal_distribution<double> disty(mean,stddev);
		std::normal_distribution<double> distz(mean,stddev);
		forceest1.gausian_noise << distx(generatorx), disty(generatory), distz(generatorz);

		pose.x = payload_pose.pose.position.x;
      
       forceest1.predict();
		Eigen::VectorXd measure;
		measure.setZero(measurementsize);

		measure << payload_pose.pose.position.x, payload_pose.pose.position.y, payload_pose.pose.position.z,payload_vel.twist.linear.x ,payload_vel.twist.linear.y ,payload_vel.twist.linear.z ;


			    

		forceest1.qk11 = forceest1.qk1;

		forceest1.correct(measure);
		forceest1.x[e_x] = 0;
		forceest1.x[e_y] = 0;
		forceest1.x[e_z] = 0;



		force1.x = forceest1.x[F1_x];
		force1.y = forceest1.x[F1_y];
		force1.z = forceest1.x[F1_z];
		force1_pub.publish(force1);
        
	    force2.x = forceest1.x[F2_x];
		force2.y = forceest1.x[F2_y];
		force2.z = forceest1.x[F2_z];
		force2_pub.publish(force2);

        pose.x = forceest1.x[p_x];
		pose.y = forceest1.x[p_y];
		pose.z = forceest1.x[p_z];
		pose_pub.publish(pose);

        vel.x = forceest1.x[v_x];
		vel.y = forceest1.x[v_y];
		vel.z = forceest1.x[v_z];
		vel_pub.publish(vel);

		loop_rate.sleep();
		ros::spinOnce();

	}
}