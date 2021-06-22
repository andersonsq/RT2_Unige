#include "ros/ros.h"
#include "rt2_assignment1/Command.h"
#include "rt2_assignment1/Position.h"
#include "rt2_assignment1/RandomPosition.h"

#include "rt2_assignment1/Assignment1Action.h"
#include "actionlib/client/simple_action_client.h"

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
   
   while(ros::ok()){
   	ros::spinOnce();
   	/*if (start){
   		client_rp.call(rp);
   		p.request.x = rp.response.x;
   		p.request.y = rp.response.y;
   		p.request.theta = rp.response.theta;
   		std::cout << "\nGoing to the position: x= " << p.request.x << " y= " <<p.request.y << " theta = " <<p.request.theta << std::endl;
   		client_p.call(p);
   		std::cout << "Position reached" << std::endl;
   	}*/
   	switch (options){
   		case 0:
			std::cout << "\nPress 1 to start the robot" << std::endl;
			std::cout << "\nPress -1 to cancel the goal" << std::endl;
			std::cin >> options;
			break;
   		case -1:
			ac.cancelGoal();
   			std::cout << "\nGoal was canceled" << std::endl;
   			options = 0;
   			break;
   		case true:
   			client_rp.call(rp);
   			goal.x = rp.response.x;
   			goal.y = rp.response.y;
   			goal.theta = rp.response.theta;
   			std::cout << "\nGoing to the position: x=%f " << goal.x << " y=%f " <<goal.y << " theta =%f \n " << goal.theta << std::endl;
   			actionlib::SimpleClientGoalState goal_state = ac.getState();
   			ac.sendGoal(goal);
   			options = 0;
   			break;
   		//default: 
   			//std::cout << "\nInvalid option" << std::endl;
   	}
   }
   return 0;
}
