#include <chrono>
#include <memory>
#include <inttypes.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "rt2_assignment1/srv/RandomPosition.hpp"
#include "rt2_assignment1/srv/Command.hpp"
#include "rt2_assignment1/srv/Position.hpp"

using namespace std::chrono_literals;

using RandomPosition = rt2_assignment1::srv::RandomPosition;	
using Command = rt2_assignment1::srv::Command;	
using Position = rt2_assignment1::srv::Position;	

namespace rt2_assignment1{

class State_Machine : public rclcpp::Node
{
public:
	State_Machine(const rclcpp::NodeOptions & options)
	: Node("state_machine", options)
	{
	start = false;
	position_reached = true;
	
  void user_interface(  	
  	const std::shared_ptr<Command::Request> req,
  	const std::shared_ptr<Command::Response> res)
  	const std::shared_ptr<rmw_request_id_t> request_header,
  	{
  	(void) request_header;
  		if (req->command == "start")
  		{
  		start = true;
  		}
  		else
  		{
  		start = false;
  		}
  	res->ok = start;
  	RCLCPP_INFO(this->get_logger(), "Received request %s", req->command.c_str());
  	}  	

//Initializqtion client and service    
      client_rp = this->create_client<RandomPosition>("/position_server");
      
      service_ = this->create_service<Command>(
      "/user_interface", std::bind(&State_Machine::user_interface, this));  
      
    while (!client_->wait_for_service(std::chrono::seconds(1))){
     if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "client interrupted while waiting for service to appear.");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "waiting for service to appear...");
  }
  
  client_p = this->create_client<Position>("/go_to_point");
  
  while (!client_->wait_for_service(std::chrono::seconds(1))){
     if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "client interrupted while waiting for service to appear.");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "waiting for service to appear...");
  }
  
  rp_req = std::make_shared<RandomPosition::Request>();
  p_req = std::make_shared<Position::Request>();
  rp_res = std::make_shared<RandomPosition::Response>();
  
  rp_req->x_max = 5.0;
  rp_req->x_min = -5.0;
  rp_req->y_max = 5.0;
  rp_req->y_min = -5.0;
}
void myrandom_call(){  
	auto rp_res_callback = [this](rclcpp::Client<rt2_assignment1::srv::RandomPosition>::SharedFuture future){rp_res = future.get();};
	auto future_result = client_rp->async_send_request(rp_req, rp_res_callback);
}

//int main() part

bool start, position_reached;

const std::shared_ptr<RandomPosition::Request> rp_req, 
const std::shared_ptr<RandomPosition::Response> rp_res, 
const std::shared_ptr<Position::Request> p_req)

rclcpp::Client<RandomPosition>::SharedPtr client_rp;
rclcpp::Service<Command>::SharedPtr service_;
rclcpp::Client<Position>::SharedPtr client_p;
};
}

RCLCPP_COMPONENTS_REGISTER_NODE(rt2_assignment1::State_Machine)

/*
#include "ros/ros.h"
#include "rt2_assignment1/Command.h"
#include "rt2_assignment1/Position.h"
#include "rt2_assignment1/RandomPosition.h"

bool start = false;

bool user_interface(rt2_assignment1::Command::Request &req, rt2_assignment1::Command::Response &res){
    if (req.command == "start"){
    	start = true;
    }
    else {
    	start = false;
    }
    return true;
}


int main(int argc, char **argv)
{
   ros::init(argc, argv, "state_machine");
   ros::NodeHandle n;
   ros::ServiceServer service= n.advertiseService("/user_interface", user_interface);
   ros::ServiceClient client_rp = n.serviceClient<rt2_assignment1::RandomPosition>("/position_server");
   ros::ServiceClient client_p = n.serviceClient<rt2_assignment1::Position>("/go_to_point");
   
   rt2_assignment1::RandomPosition rp;
   rp.request.x_max = 5.0;
   rp.request.x_min = -5.0;
   rp.request.y_max = 5.0;
   rp.request.y_min = -5.0;
   rt2_assignment1::Position p;
   
   while(ros::ok()){
   	ros::spinOnce();
   	if (start){
   		client_rp.call(rp);
   		p.request.x = rp.response.x;
   		p.request.y = rp.response.y;
   		p.request.theta = rp.response.theta;
   		std::cout << "\nGoing to the position: x= " << p.request.x << " y= " <<p.request.y << " theta = " <<p.request.theta << std::endl;
   		client_p.call(p);
   		std::cout << "Position reached" << std::endl;
   	}
   }
   return 0;
}
*/

