#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>

// set up positions
float pick_up[3] = {3.0, 2.0, 1.0};
float drop_off[3] = {-1.0, 0.0, 1.0};
float threshold[2] = {0.3, 0.01};

bool atPickup = false;
bool atDropoff = false;
bool pickUpThing = false;
bool putDownThing = false;

void commCallback(const nav_msgs::Odometry::ConstPtr& msg){
    // pick up something
    if(std::abs(pick_up[0] -msg->pose.pose.position.x) < threshold[0] && std::abs(pick_up[1] -msg->pose.pose.position.y) < threshold[0] && std::abs(pick_up[2] -msg->pose.pose.orientation.w) < threshold[1]){
        if(!atPickup){
            atPickup = true;
        } 
    } else {
        atPickup = false;
    }

    if(std::abs(drop_off[0] -msg->pose.pose.position.x) < threshold[0] && std::abs(drop_off[1] -msg->pose.pose.position.y) < threshold[0] && std::abs(drop_off[2] -msg->pose.pose.orientation.w) < threshold[1]){
         if(!atDropoff){
            atDropoff = true;
         } 
    } else {
        atDropoff = false;
    }
}

int main(int argc, char** argv){
    // initialization of ros, nodeHandle, Publisher
    ros::init(argc, argv, "add_markers");
    ros::NodeHandle n;
    ros::Rate r(1);
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    ros::Subscriber odom_sub = n.subscribe("odom", 1000, commCallback);
    
    // we are going to make a cube
    uint32_t shape = visualization_msgs::Marker::CUBE;

    while (ros::ok())
    {
        // create the marker and set the frame and timestamp
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();

        // set namespace and id for marker, needs a unique ID
        // if you make a marker with the same namespace and ID, it will overwrite the old one
        marker.ns = "add_markers";
        marker.id = 0;

        // set the marker type. it starts as a cube, and cycles between that and sphere
        // then goes to arrow and cylinder
        marker.type = shape;

        // set marker action, you can ADD, DELETE and in Indigo (DELETEALL)
        marker.action = visualization_msgs::Marker::ADD;

        // set marker poses, full 6DOF pose relative to frame id/timestamp
        marker.pose.position.x = pick_up[0];
        marker.pose.position.y = pick_up[1];
        marker.pose.position.z = 0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = pick_up[2];

        // set the scale of the marker, 1 x 1 x 1 is a 1m a side
        marker.scale.x = 0.5;
        marker.scale.y = 0.5;
        marker.scale.z = 0.5;

        // set a color
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0;

        // how long will the cube live?
        marker.lifetime = ros::Duration();

        // let's publish the marker
        while(marker_pub.getNumSubscribers() < 1){
            if(!ros::ok()) {
                return 0;
            }
            ROS_WARN_ONCE("Please add a subscriber to the marker.");
            sleep(1);
        }
        
        marker_pub.publish(marker);
        ROS_INFO("Pick up marker is now displayed.");

        // robot is travelling
        while(!atPickup)
            ros::spinOnce();
        
        // robot arrived at the object
        if(atPickup && !pickUpThing){
            marker.action = visualization_msgs::Marker::DELETE;
            marker_pub.publish(marker);
            ROS_INFO("Pick up completed.");
        }
        
        // robot is moving to the dropoff
        while(!atDropoff){
            ros::spinOnce();
        }

        // robot has arrived at the drop off 
        if(atDropoff && !putDownThing){
            marker.pose.position.x = drop_off[0];
            marker.pose.position.y = drop_off[1];
            marker.pose.orientation.w = drop_off[2];
            
            marker.action = visualization_msgs::Marker::ADD;
            marker_pub.publish(marker);
            ROS_INFO("Drop off marker displayed.");
            putDownThing = true;
            
            ros::Duration(10.0).sleep();
        }

        return 0;
    }
}
