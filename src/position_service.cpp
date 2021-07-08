#include <chrono>
#include <memory>
#include <inttypes.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "rt2_assignment1/srv/random_position.hpp"

//Register a component that can be dynamically loaded at runtime
#include "rclcpp_components/register_node_macro.hpp"

using namespace std::chrono_literals;

using RandomPosition = rt2_assignment1::srv::RandomPosition;	//name of my .srv file, I'll get the informations from there

namespace rt2_assignment1{

class Position_srv : public rclcpp::Node
{
public:
	Position_srv(const rclcpp::NodeOptions & options)
	: Node("random_position_server", options)
	{
		service_ = this->create_service<RandomPosition>(
      "/position_server", std::bind(&Position_srv::myrandom, this));
      }

private:
	double randMToN(double M, double N)
{     return M + (rand() / ( RAND_MAX / (N-M) ) ) ; }


void myrandom (
	const std::shared_ptr<RandomPosition::Request> req, 
	const std::shared_ptr<RandomPosition::Response> res, 
	const std::shared_ptr<rmw_request_id_t> request_header)
	{
	(void)request_header;
	res->x = randMToN(req->x_min, req->x_max);
	res->y = randMToN(req->y_min, req->y_max);
	res->theta = randMToN(-3.14, 3.14);
	}
rclcpp::Service<RandomPosition>::SharedPtr service_;  //my server for position
};
			}	

RCLCPP_COMPONENTS_REGISTER_NODE(rt2_assignment1::Position_srv)
	
