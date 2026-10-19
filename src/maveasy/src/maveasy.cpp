#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <mavros_msgs/msg/status_text.hpp>

using namespace std;

class MavEasy : public rclcpp::Node {
	public: 
		MavEasy() : Node("maveasy_node") {
			statustext_pub = this->create_publisher<mavros_msgs::msg::StatusText>("/mavros/statustext/send", 10);
			auto message_callback = 
				[this] (std_msgs::msg::String::UniquePtr msg) -> void {
					//string data = msg.data;
					//int seq = 0;
					//while (data != ""){
						mavros_msgs::msg::StatusText send_status;
						send_status.header.stamp = this->get_clock()->now();
						send_status.severity = send_status.INFO;
						send_status.text = msg->data;
						//send_status.text = ""
						//while (send_status.text.length() < 50 && data != "") {
						//	send_status.text += data.at(0);
						//	data.erease(0,1);
						//}
						//send_status.id = message_counter;
						//send_status.chunk_seq = seq++;
						statustext_pub->publish(send_status);
					//}
					message_counter++;
				};
			message_sub = this->create_subscription<std_msgs::msg::String>("gs_message_send", 10, message_callback);
			message_counter = 1;
		}
	private:
		rclcpp::Subscription<std_msgs::msg::String>::SharedPtr message_sub;	
		rclcpp::Publisher<mavros_msgs::msg::StatusText>::SharedPtr statustext_pub; 
		int message_counter;
};

int main(int argc, char * argv[]) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<MavEasy>());
	rclcpp::shutdown();
	return 0;
}
