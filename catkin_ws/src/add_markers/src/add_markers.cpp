#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>

//Tasks
// void Odometry_CallBack ()
double x_odom = 0.0;
double y_odom=0.0;

void Odometry_CallBack(const nav_msgs::Odometry::ConstPtr& msg)
{
   x_odom=msg->pose.pose.position.x;
   y_odom=msg->pose.pose.position.y;
}


int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  
  // Subscribe odometry to keep track of the robot pose.
  ros::Subscriber odom_subscriber = n.subscribe("/odom",1000,Odometry_CallBack);


  //  ----------- Set Goals ----------(Same with Pick Up.cpp Node)//
  double pick_goal_coordinate[3]={-6.5, -1.0, 1.5707}; //x y w
  double dropoff_goal_coordinate[3]={0.0, 0.0, 1.0472};


  // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::CUBE;

  // Work as switch to control the display of markers
  uint32_t phase_flag = 0;




  ///************** 1. Intialize Marker ************///
    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "basic_shapes";
    marker.id = 0;

    // Set the marker type.  Initially this is CUBE. (and cycles between that and SPHERE, ARROW, and CYLINDER)
    marker.type = shape; 

    // Set the scale of the marker -- 0.5x0.5x0.5 here means 0.5m on a side
    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.5;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;
    //How long this marker should stick around before automatically deleted.
    marker.lifetime = ros::Duration();

  ///************** 1. Intialize Marker ************///



  // Variables to check position
  double distance_pick=0.0;
  double distance_drop=0.0;
  double pos_tolerance = 1.0;


  while (ros::ok())
  {
    
    while (marker_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return 0;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }






  ///************** 2. Switch - Display Each Phase *********///
       // 2.0 Initially show the marker @ the PickUp Zone
       // 2.1 Check if the robot is in the Pick Up Zone  
       // 2.2 Hide Marker && Wait 5 secs to simulate a pickup
       // 2.3 Show the marker at the drop off zone once robot reaches
       // 2.4 Check if the robot is in the Drop Off Zone
   switch (phase_flag)
    {
    case 0:
            ROS_INFO("Show the marker at the PickUp Zone!");
            //Action: Add
            marker.action = visualization_msgs::Marker::ADD;
            //Position: PickUp
            marker.pose.position.x = pick_goal_coordinate[0];
   	    marker.pose.position.y = pick_goal_coordinate[1];
  	    marker.pose.position.z = 0;
    	    marker.pose.orientation.x = 0.0;
   	    marker.pose.orientation.y = 0.0;
     	    marker.pose.orientation.z = 0.0;
    	    marker.pose.orientation.w = pick_goal_coordinate[2];
   	   
            marker_pub.publish(marker);
	    phase_flag=1;
    	break;

    case 1:
	    ROS_INFO("The robot is Moving to the PickUp Zone!");
	    distance_pick=sqrt(pow((marker.pose.position.x-x_odom),2)+pow((marker.pose.position.y -y_odom),2));
	    ROS_INFO("The X-axis is : %f and The Y-axis is : %f",x_odom,y_odom);	
	    ROS_INFO("Pick Disance is : %f ",distance_pick);
            if (distance_pick<pos_tolerance)
		{
		   ROS_INFO("The robot Reaches the PickUp Zone!");
		   ROS_INFO("The X-axis is : %f and The Y-axis is : %f",x_odom,y_odom);	
		   phase_flag=2;  
				
		}else 
                {
		   //ROS_ERROR_STREAM("FAILED To reach the PickUp Goal");
 		}

      break;

    case 2:
             // Hide the Marker
	     ROS_INFO("The Marker is Fading Away!");	
             //Action: Delete
             marker.action = visualization_msgs::Marker::DELETE;
	     marker_pub.publish(marker);
	     ROS_INFO("Simulate the Robot to Pick Up, Using 5 secs!");
             ros::Duration(5).sleep(); // pause 5 seconds
	     phase_flag=3;

      break;

    case 3:
            ROS_INFO("Show the marker at the DropOff Zone!");
            //Action: Add
            marker.action = visualization_msgs::Marker::ADD;
            //Position: PickUp
            marker.pose.position.x = dropoff_goal_coordinate[0];
   	    marker.pose.position.y = dropoff_goal_coordinate[1];
  	    marker.pose.position.z = 0;
    	    marker.pose.orientation.x = 0.0;
   	    marker.pose.orientation.y = 0.0;
     	    marker.pose.orientation.z = 0.0;
    	    marker.pose.orientation.w = pick_goal_coordinate[2];
   	   
            marker_pub.publish(marker);
	    phase_flag=4;
    	break;

    case 4:
	    ROS_INFO("The robot is Moving to the DropOff Zone!");
	    distance_drop=sqrt(pow((marker.pose.position.x-x_odom),2)+pow((marker.pose.position.y -y_odom),2));
	    ROS_INFO("The X-axis is : %f and The Y-axis is : %f",x_odom,y_odom);	
	    ROS_INFO("Pick Disance is : %f ",distance_drop);


            if (distance_drop<pos_tolerance)
		{
		   ROS_INFO("The robot Reaches the DropOff Zone!");
		   ROS_INFO("The X-axis is : %f and The Y-axis is : %f",x_odom,y_odom);	
		   phase_flag=5;  
				
		}else 
                {
		   //ROS_ERROR_STREAM("FAILED To reach the PickUp Goal");
 		}


      break;

    case 5:
             // Hide the Marker
	     ROS_INFO("The Marker is Fading Away, Again!");	
             //Action: Delete
             marker.action = visualization_msgs::Marker::DELETE;
	     marker_pub.publish(marker);

             ros::Duration(5).sleep(); // pause 5 seconds
	     ROS_INFO("--Well Done!-- The robot has droped off the object");
      break;
    }


/*

    // Maker {Action && Position }
    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Cycle between different shapes
    switch (shape)
    {
    case visualization_msgs::Marker::CUBE:
      shape = visualization_msgs::Marker::SPHERE;
      break;
    case visualization_msgs::Marker::SPHERE:
      shape = visualization_msgs::Marker::ARROW;
      break;
    case visualization_msgs::Marker::ARROW:
      shape = visualization_msgs::Marker::CYLINDER;
      break;
    case visualization_msgs::Marker::CYLINDER:
      shape = visualization_msgs::Marker::CUBE;
      break;
    }
*/

    ros::spinOnce();
    r.sleep();
  } //---> while ros::ok
  return 0;
}


