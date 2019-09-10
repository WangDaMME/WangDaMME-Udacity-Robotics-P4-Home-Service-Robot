#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

//Define a client to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;




int main(int argc, char** argv)
{
  //Initialize the simpe_navigation_goals node
  ros::init(argc,argv, "pick_objects"); // node name: simpe_navigation_goals

  //Tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base",true);

  //Wait 5 secs for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0)))
  {
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  //  ----------- Set Goals ----------//
  double pick_goal_coordinate[3]={-6.5, -1.0, 1.5707}; //x y w
  double dropoff_goal_coordinate[3]={0.0, 0.0, 1.0472};


  // ----------- Pick Up----------//

  move_base_msgs::MoveBaseGoal pick_goal;
 
  //set up the frame parameters
  pick_goal.target_pose.header.frame_id = "map";  //"base_link"
  pick_goal.target_pose.header.stamp = ros::Time::now();


  //Define the 1st GOAL {--Pick Up--} position and orientation for the robot to reach 
  pick_goal.target_pose.pose.position.x = pick_goal_coordinate[0];
  pick_goal.target_pose.pose.position.y = pick_goal_coordinate[1];
  pick_goal.target_pose.pose.orientation.w = pick_goal_coordinate[2];


  //Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending Pick-Up Goal");
  ac.sendGoal(pick_goal);

  //Wait an infinite time for the results
  ac.waitForResult();

  //Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    
    //Display the message to Track if robot successfully reached
    ROS_INFO("Hooray, the robot is in the PICK ZONE ! ");

    //Pause 5 seconds after reaching the pickup zone.
    ros::Duration(5).sleep(); // pause 5 seconds

   


    // ----------- Drop off ----------//

    move_base_msgs::MoveBaseGoal dropoff_goal;
 
    //set up the frame parameters
    dropoff_goal.target_pose.header.frame_id = "map";  
    dropoff_goal.target_pose.header.stamp = ros::Time::now();


    //Define the 2nd GOAL {--Drop Off--} position and orientation for the robot to reach 
    dropoff_goal.target_pose.pose.position.x = dropoff_goal_coordinate[0];
    dropoff_goal.target_pose.pose.position.y = dropoff_goal_coordinate[1];
    dropoff_goal.target_pose.pose.orientation.w = dropoff_goal_coordinate[2];
 

    //Send the goal position and orientation for the robot to reach
    ROS_INFO("Sending Drop-Off Goal");
    ac.sendGoal(dropoff_goal);

    //Wait an infinite time for the results
    ac.waitForResult();

    //Check if the robot reached its goal
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
    
      // Display the message to Track if robot successfully reached
      ROS_INFO("Hooray, the robot is in the DROP OFF ZONE ! ");

    }
    else
    {
      ROS_INFO("The base FAILED to Drop off ...");  
    }

    //ros::Duration(5).sleep(); // pause 5 seconds

  }
  else
  {
    ROS_INFO("The base FAILED to Pick up...");  
  }




  return 0;
}
