#include "goal_controller.h"
#include <geometry_msgs/Twist.h>
#include <math.h>

using namespace goal_control;
using namespace std;

Pose pose, goal_pose;
ros::Publisher twist_pub; //  Declare publisher object globally
ros::Subscriber odom_sub; //    Declare subscriber object globally

geometry_msgs::Twist msgv;
nav_msgs::Odometry msgp;
float goal_x, goal_y, goal_yaw, currentx, currenty, currentyaw, linvel, angvel;
float m, anglem, xdiff, ydiff, del_theta, del_lin;
double roll, pitch, yaw;
bool goal_received = false, goal_reached = false;

// constructor
  GoalController::GoalController(std::string name) :
    as_(this->nh_, "my_server", boost::bind(&GoalController::executeCB, this, _1), false),
    action_name_("my_server")
  {
    ros::NodeHandle private_nh("~");
// =========================================
    // controller gain parameters
    private_nh.param("kP", kP, 0.5);
    private_nh.param("kB", kB, 1.0);
    private_nh.param("kA", kA, -0.8);
 // ========================================
    private_nh.param("max_linear_speed", max_linear_speed, 0.4);
    private_nh.param("min_linear_speed", min_linear_speed, 0.0);
    private_nh.param("max_angular_speed", max_angular_speed, 0.4);
    private_nh.param("min_angular_speed", min_angular_speed, 0.0);
    private_nh.param("linear_tolerance", linear_tolerance, 0.01);
    private_nh.param("angular_tolerance", angular_tolerance, (1*3.141/180));

    this->as_.start();
    
    odom_sub = this->nh_.subscribe<nav_msgs::Odometry>("odom", 1000, &GoalController::poseMessageReceived, this);
    goal_achieved_pub = this->nh_.advertise<std_msgs::Bool>("goal_achieved", 1);
    ros::spin();
}

// server plus controller 
void GoalController::publishVelocity()
{
    xdiff = goal_x - currentx;
    ydiff = goal_y - currenty;       

    if ( (xdiff == 0) && (ydiff == 0))  // atan2 is undefined
        anglem = 0;
    else
        anglem = atan2 (ydiff, xdiff);  // Desired heading
    
    // Display received message
    ROS_INFO_STREAM(std::setprecision(2) << std::fixed << "RECEIVED pos = (" << currentx << "," << currenty 
        << " ang = " << currentyaw <<")\n");

    del_lin = fabs( sqrt( xdiff * xdiff + ydiff * ydiff) ); // Error in Euclidean distance
    
                Pose pose; double ratio, direction;
                double goal_heading = atan2(goal_y - currenty, goal_x - currentx);
                double a = -currentyaw + goal_heading;
                double Th1 = currentyaw;
                double Th2 = goal_yaw;
                double dTh = (Th1 - Th2);
                dTh = abs(this->normalize_pi(dTh));

                double b = -dTh - a;

                    direction = cos(a);
                    direction = sign(direction);
                    a = this->normalize_half_pi(a);
                    b = this->normalize_half_pi(b);                  
                // }
                if (abs(del_lin) < linear_tolerance){
                    pose.xVel = 0;
                    pose.thetaVel = kB * dTh;
                }
                else{
                    pose.xVel = kP * del_lin * direction;
                    pose.thetaVel = kA*a + kB*b;
                }

                 // Adjust velocities if X velocity is too high.
                if (abs(pose.xVel) > max_linear_speed){
                    ratio = max_linear_speed / abs(pose.xVel);
                    pose.xVel *= ratio;
                    pose.thetaVel *= ratio;
                }

                // # Adjust velocities if turning velocity too high.
                if (abs(pose.thetaVel) > max_angular_speed){
                    ratio = max_angular_speed / abs(pose.thetaVel);
                    pose.xVel *= ratio;
                    pose.thetaVel *= ratio;
                }
                if (abs(pose.xVel) > 0 && abs(pose.xVel) < min_linear_speed){
                    ratio = min_linear_speed / abs(pose.xVel);
                    pose.xVel *= ratio;
                    pose.thetaVel *= ratio;
                }
                else if (pose.xVel==0 && abs(pose.thetaVel) < min_angular_speed){
                    ratio = min_angular_speed / abs(pose.thetaVel);
                    pose.xVel *= ratio;
                    pose.thetaVel *= ratio;
                }
    
    if ( del_lin < linear_tolerance && dTh < angular_tolerance)
 
    {
        std::cout << "\n\n\n ***************** Goal reached. **************** \n\n\n";
        pose.xVel = 0;
        pose.thetaVel = 0;
        goal_reached = true;
    }
        
    msgv.linear.x = pose.xVel;
    msgv.angular.z = pose.thetaVel;
    
    cout << " linear velocity   "<< pose.xVel<< "   angular velocity  "<< pose.thetaVel<< endl;
    // Publish the message
    twist_pub.publish (msgv);

    // Send a message to rosout with the details
    ROS_INFO_STREAM ("SENT: " << "linear Error= " << del_lin << "; " << "angular Error=" << dTh );
}




void GoalController::poseMessageReceived (const nav_msgs::Odometry::ConstPtr& msg)
{
    currentx = msg->pose.pose.position.x;
    currenty = msg->pose.pose.position.y;   
    
    // Convert quaternion to yaw Euler angle
    tf::Quaternion q( msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w );
    tf::Matrix3x3 mat(q);
    mat.getRPY (roll, pitch, yaw);
    currentyaw = float( yaw);
    
    // Call publisher function
    if(goal_received)
        this->publishVelocity();
}

    void GoalController::executeCB(const blind_planner::GoToPoseGoalConstPtr &msg)
    {
        goal_x = msg->pose.pose.position.x;
        goal_y = msg->pose.pose.position.y;  
        tf::Quaternion q( msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w );
        tf::Matrix3x3 mat(q);
        mat.getRPY (roll, pitch, yaw);
        goal_yaw = float( yaw);       

        std::cout<<"entered in the server callback"<< std::endl;
        goal_received = true;
        goal_pose = this->get_angle_pose(msg->pose);   
        bool success = true; 
        while(ros::ok()){
            // checks for if new goal is requested while another goal is active
            if (this->as_.isPreemptRequested() || !ros::ok()){
                ROS_INFO("%s: Preempted", action_name_.c_str());
                linvel = 0, angvel = 0;
                msgv.linear.x = linvel;
                msgv.angular.z = angvel;   
                twist_pub.publish (msgv);             
                this->as_.setPreempted();
                success = false;
                break;
            }
        }
        if(success)
        {
            result_.success = success;
            this->as_.setSucceeded(result_);
        }
    }

    Pose GoalController::get_angle_pose(const geometry_msgs::PoseStamped &quaternion_pose){
        Pose angle_pose;
        tf::Quaternion q;
        quaternionMsgToTF(quaternion_pose.pose.orientation, q);
        tf::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        angle_pose.x = quaternion_pose.pose.position.x;
        angle_pose.y = quaternion_pose.pose.position.y;
        angle_pose.theta = yaw;
        return angle_pose;      
    }


int main(int argc, char** argv)
{
  ros::init(argc, argv, "my server");

    ros::NodeHandle nh;
    twist_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
    GoalController controller("my_server");
  
    ros::spin();

  return 0;
}
