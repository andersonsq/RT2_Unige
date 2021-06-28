#include "ros/ros.h"
#include "rt2_assignment1/Command.h"
#include "rt2_assignment1/Position.h"
#include "rt2_assignment1/RandomPosition.h"

#include "rt2_assignment1/Assignment1Action.h"
#include "actionlib/client/simple_action_client.h"
#include "actionlib/client/terminal_state.h"

/*global variables*/
bool start = false;
int options = 0;

/*function to start my user_interface*/
bool user_interface(rt2_assignment1::Command::Request &req, rt2_assignment1::Command::Response &res)
{
    if (req.command == "start")
    	{
    	options = 1;
    	}
    else 
    	{
    	options = -1;
    	}
    return true;
}

/*void function to check my robot status*/
void simple_done_callback(const actionlib::SimpleClientGoalState &get_state, const rt2_assignment1::Assignment1ResultConstPtr &result)
{    
  options = 2;
}

/*main function*/
int main(int argc, char **argv)
{
   ros::init(argc, argv, "state_machine");
   ros::NodeHandle n;
   
   ros::ServiceServer service= n.advertiseService("/user_interface", user_interface);
   ros::ServiceClient client_rp = n.serviceClient<rt2_assignment1::RandomPosition>("/position_server");
   actionlib::SimpleActionClient<rt2_assignment1::Assignment1Action> ac("go_to_point", true);
    
   /*local variables*/
   rt2_assignment1::Assignment1Goal goal; 
   //actionlib::SimpleClientGoalState get_state = ac.getState();
   
   //Limit where the robot can go
   rt2_assignment1::RandomPosition rp;
   rp.request.x_max = 5.0;
   rp.request.x_min = -5.0;
   rp.request.y_max = 5.0;
   rp.request.y_min = -5.0;   
 
   while(ros::ok()){
   	ros::spinOnce();

   	switch (options){
   		case -1:
   			//ac.getState();
			ac.cancelGoal();
   			std::cout << "\nGoal was canceled" << std::endl;
   			options = 0;
   			break;
   		/*case 0:			//Useless case
   			ac.getState();
   			break;*/
   		case 1:
   			//ac.getState();
   			client_rp.call(rp);
   			goal.x = rp.response.x;
   			goal.y = rp.response.y;
   			goal.theta = rp.response.theta;
   			std::cout << "\nGoing to the position: x= " << goal.x << " y= " <<goal.y << " theta = \n " << goal.theta << std::endl;
   			ac.sendGoal(goal, &simple_done_callback);
   			options = 0;	//false = 0
   			break; 
   		case 2:
   			actionlib::SimpleClientGoalState get_state = ac.getState();
   			if(get_state == actionlib::SimpleClientGoalState::SUCCEEDED)
			{
			std::cout << "\nOn Goal" << std::endl;
			options = 1;	//false = 0
			}
			else if(get_state == actionlib::SimpleClientGoalState::PREEMPTED)
			{
			std::cout << "\nCanceled" << std::endl;
			options = 0;	//false = 0
			}
			else
			{
			std::cout << "\nFailed" << std::endl;
			options = 0;	//false = 0
			}
   	}
   }
   return 0;
}
