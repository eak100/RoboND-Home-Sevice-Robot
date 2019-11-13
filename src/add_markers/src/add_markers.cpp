#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <math.h>



double startX= -3.0;
double startY= 1.0;
double endX= 2.0;
double endY= 1.0;

bool isItemPicked=false;
bool isItemDropped=false;

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
  double RobotX = msg->pose.pose.position.x;
  double RobotY = msg->pose.pose.position.y;

  double distance2Start=sqrt(pow(RobotX-startX, 2) + pow(RobotY-startY, 2));
  double distance2End=sqrt(pow(RobotX-endX, 2) + pow(RobotY-endY, 2));

  if (distance2Start < 0.4) {
      isItemPicked = true;
    }
else if (distance2End < 0.8) {
      isItemDropped = true;
    }
}



int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  ros::Subscriber odom_sub = n.subscribe("odom", 10, odomCallback);


  // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::CUBE;

  visualization_msgs::Marker marker;
  // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();

  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  marker.ns = "cube";
  marker.id = 0;

  // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
  marker.type = shape;



  // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
  marker.pose.position.x = startX;
  marker.pose.position.y = startY;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker.scale.x = 0.2;
  marker.scale.y = 0.2;
  marker.scale.z = 0.2;

  // Set the color -- be sure to set alpha to something non-zero!
  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0;

  marker.lifetime = ros::Duration();


  while (ros::ok())
  {

    // Publish the marker
    while (marker_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return 0;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }

    if (isItemPicked==false){
      // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
      marker.action = visualization_msgs::Marker::ADD;
      marker_pub.publish(marker);
}
  else if (isItemDropped==false)
{
  marker.action = visualization_msgs::Marker::DELETE;
  marker_pub.publish(marker);
}
else if (isItemDropped)
{
  // Go to Sleep for 1 seconds
    ros::Duration(2.0).sleep();
  marker.pose.position.x = endX;
  marker.pose.position.y = endY;
  marker.action = visualization_msgs::Marker::ADD;
  marker_pub.publish(marker);
}



ros::spinOnce();



  }
}
