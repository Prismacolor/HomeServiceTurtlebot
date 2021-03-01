#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

// start and goal positions
float pick_up[3] = {3.0, 2.0, 1.0};
float drop_off[3]= {-1.0, 0.0, 1.0};

// need Simple Action client to send goal requests to the move_base server
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
    // initialize our simple nav goals node
    ros::init(argc, argv, "pick_objects");

    // tell action client we want to spin up a thread by default
    MoveBaseClient ac("move_base", true);
    
    // Wait five seconds for move_base action server to come up 
    while(!ac.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    move_base_msgs::MoveBaseGoal goal;

    //set up frame parameters
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    // establish new goals
    // float goals[2][3] = { {3.0, 6.0, 1.0}, {-2.0, 1.0, 1.0} };
    // int numofPoints = 2;

       
    // define a goal: pick up location
    goal.target_pose.pose.position.x = pick_up[0];
    goal.target_pose.pose.position.y = pick_up[1];
    goal.target_pose.pose.orientation.w = pick_up[2];

    //send goal position and orientation 
    ROS_INFO("sending pick-up goal");
    ac.sendGoal(goal);
    
    // wait for results
    ac.waitForResult();

    // check to see if robot reached goals
    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
        ROS_INFO("Robot has reached the pick-up location.");
        ros::Duration(5.0).sleep();

        // define new goal: drop off location
        goal.target_pose.pose.position.x = drop_off[0];
        goal.target_pose.pose.position.y = drop_off[1];
        goal.target_pose.pose.orientation.w = drop_off[2]; 

        ROS_INFO("sending drop-off goal");
        ac.sendGoal(goal);
    
        // wait for results
        ac.waitForResult();

        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
            ROS_INFO("Robot has reached the drop-off location.");
            ros::Duration(5.0).sleep();
        } else {
            ROS_INFO("Robot has not reached the drop-off point.");
        }
    } else {
        ROS_INFO("Error! Robot has not reached the pick up location.");
    }
    
    return 0;
}
