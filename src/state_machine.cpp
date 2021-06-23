#include "ros/ros.h"
#include "rt2_assignment1/Command.h"
#include "rt2_assignment1/Position.h"
#include "rt2_assignment1/RandomPosition.h"

#include "rt2_assignment1/Assignment1Action.h"
#include "actionlib/client/simple_action_client.h"
#include "actionlib/client/terminal_state.h"

bool start = false;
int options = 0;

bool user_interface(rt2_assignment1::Command::Request &req, rt2_assignment1::Command::Response &res){
    if (req.command == "start"){
    	options = true;
    }
    else {
    	options = -1;
    }
    return true;
}

void simple_done_callback(const actionlib::SimpleClientGoalState & get_state,
                const rt2_assignment1::Assignment1ResultConstPtr & result)
                {    
  		options = 2;
  		}

int main(int argc, char **argv)
{
   ros::init(argc, argv, "state_machine");
   ros::NodeHandle n;
   
   ros::ServiceServer service= n.advertiseService("/user_interface", user_interface);
   ros::ServiceClient client_rp = n.serviceClient<rt2_assignment1::RandomPosition>("/position_server");
   //ros::ServiceClient client_p = n.serviceClient<rt2_assignment1::Position>("/go_to_point");
     actionlib::SimpleActionClient<rt2_assignment1::Assignment1Action> ac("go_to_point", true);
   
   //Limit where the robot can go
   rt2_assignment1::RandomPosition rp;
   rp.request.x_max = 5.0;
   rp.request.x_min = -5.0;
   rp.request.y_max = 5.0;
   rp.request.y_min = -5.0;
   //rt2_assignment1::Position p;
   rt2_assignment1::Assignment1Goal goal;
      
   actionlib::SimpleClientGoalState get_state = ac.getState();
   
   while(ros::ok()){
   	ros::spinOnce();

   	switch (options){
   		/*case 0:
			std::cout << "\nPress 1 to start the robot" << std::endl;
			std::cout << "\nPress -1 to cancel the goal" << std::endl;
			std::cin >> options;
			break;*/
   		case -1:
   			//ac.getState();
			ac.cancelGoal();
   			std::cout << "\nGoal was canceled" << std::endl;
   			options = 0;
   			break;
   		case true:
   			//ac.getState();
   			client_rp.call(rp);
   			goal.x = rp.response.x;
   			goal.y = rp.response.y;
   			goal.theta = rp.response.theta;
   			std::cout << "\nGoing to the position: x= " << goal.x << " y= " <<goal.y << " theta = \n " << goal.theta << std::endl;
   			ac.sendGoal(goal, &simple_done_callback);
   			options = 0;
   			break;
   		case 2:
   			//ac.getState();
   			if(get_state == actionlib::SimpleClientGoalState::SUCCEEDED)
			{
			std::cout << "\nOn Goal direction" << std::endl;
			options = 1;
			}
			else if(get_state == actionlib::SimpleClientGoalState::PREEMPTED)
			{
			std::cout << "\nCanceled" << std::endl;
			options = 0;
			}
			else
			{
			std::cout << "\nFailed" << std::endl;
			options = 0;
			}
   		//default: 
   			//std::cout << "\nInvalid option" << std::endl;
   	}
   }
   return 0;
}
