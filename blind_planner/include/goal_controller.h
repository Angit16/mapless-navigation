#include <iostream>
#include <math.h>
#include "pose.h"
#include <ros/ros.h>
#include <bits/stdc++.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include "blind_planner/GoToPoseAction.h"
#include <tf/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf/LinearMath/Transform.h>


namespace goal_control{

typedef actionlib::SimpleActionServer<blind_planner::GoToPoseAction> GoActionServer;

class GoalController{

    public:
        GoalController(std::string name);

        ~GoalController(void)
        {
        }

            double normalize_half_pi(double &alpha){
                    alpha = normalize_pi(alpha);
                    if (alpha > M_PI/2)
                        return alpha - M_PI;
                    else if (alpha < -M_PI/2)
                        return alpha + M_PI;
                    else
                        return alpha;
            }

            double normalize_pi (double &alpha){
                    if (alpha > M_PI)
                        alpha -= 2*M_PI;
                    else if (alpha < -M_PI)
                        alpha += 2*M_PI;
                    return alpha;                
            }

            double sign(double &x){
                    if (x >= 0)
                        return 1;
                    else
                        return -1;
            }
            // calculates euclidean distance between two points in 2D place.
            double get_goal_distance(Pose &cur, Pose& goal){
                    double diffX = cur.x - goal.x;
                    double diffY = cur.y - goal.y;
                    return sqrt(diffX*diffX + diffY*diffY);
            }
            // implments the controller
            void publishVelocity();
            // Odometry callback
            void poseMessageReceived(const nav_msgs::Odometry::ConstPtr& msg);
            // computes pose from quaternion
            Pose get_angle_pose(const geometry_msgs::PoseStamped& quaternion_pose);
            // action server callback
            void executeCB(const blind_planner::GoToPoseGoalConstPtr &goal);
            bool goal_stat = false;
            ros::Publisher goal_achieved_pub;
            blind_planner::GoToPoseGoal goal_;
            double kP;
            double kA;
            double kB;
            double max_linear_speed;
            double min_linear_speed;
            double max_angular_speed;
            double min_angular_speed;
            double linear_tolerance;     
            double angular_tolerance;  

protected:
  ros::NodeHandle nh_;
  GoActionServer as_;
  std::string action_name_;
  blind_planner::GoToPoseResult result_;
};
}
