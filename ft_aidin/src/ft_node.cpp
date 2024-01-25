#include "rclcpp/rclcpp.hpp"
#include "can_msgs/msg/frame.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"  

class FTNode : public rclcpp::Node {
public:
  FTNode() : Node("ft_node"), cut_off_frequency_(10.0), dt_(0.001) {    

    // 멤버 변수 초기화
    force_.x = 0.0;
    force_.y = 0.0;
    force_.z = 0.0;
    torque_.x = 0.0;
    torque_.y = 0.0;
    torque_.z = 0.0;

    force_prev_.x = 0.0;
    force_prev_.y = 0.0;
    force_prev_.z = 0.0;
    torque_prev_.x = 0.0;
    torque_prev_.y = 0.0;
    torque_prev_.z = 0.0;

    force_now_.x = 0.0;
    force_now_.y = 0.0;
    force_now_.z = 0.0;
    torque_now_.x = 0.0;
    torque_now_.y = 0.0;
    torque_now_.z = 0.0;

    // Subscriber 및 Publisher 초기화
    subscriber_ = this->create_subscription<can_msgs::msg::Frame>(
      "/can_tx", 10, std::bind(&FTNode::frame_callback, this, std::placeholders::_1));
    publisher_ = this->create_publisher<geometry_msgs::msg::WrenchStamped>("/ft_data", 10);
    publisher_fit_= this->create_publisher<geometry_msgs::msg::WrenchStamped>("/ft_fit_data", 10);
  }

private:
  void frame_callback(const can_msgs::msg::Frame::SharedPtr msg) {
    const auto& data = msg->data;

    force_prev_ = force_now_;
    torque_prev_ = torque_now_;

    if (data.size() < 8) {
      RCLCPP_ERROR(this->get_logger(), "Data field too short");
      return;
    }
    
    if(msg->id == 1) {
      force_.x = 0.01 * (256 * data[0] + data[1]) - 300.0; 
      force_.y = 0.01 * (256 * data[2] + data[3]) - 300.0; 
      force_.z = 0.01 * (256 * data[4] + data[5]) - 300.0;
    }
    else if(msg->id == 2) {
      torque_.x = 0.002 * (256 * data[0] + data[1]) - 50.0; 
      torque_.y = 0.002 * (256 * data[2] + data[3]) - 50.0; 
      torque_.z = 0.002 * (256 * data[4] + data[5]) - 50.0;
    }
    else {
      RCLCPP_WARN(this->get_logger(), "Unknown ID");
      return;
    }

    force_now_.x = lowPassFilter(force_.x, force_prev_.x, dt_, cut_off_frequency_);
    force_now_.y = lowPassFilter(force_.y, force_prev_.y, dt_, cut_off_frequency_);
    force_now_.z = lowPassFilter(force_.z, force_prev_.z, dt_, cut_off_frequency_);
    torque_now_.x = lowPassFilter(torque_.x, torque_prev_.x, dt_, cut_off_frequency_);
    torque_now_.y = lowPassFilter(torque_.y, torque_prev_.y, dt_, cut_off_frequency_);
    torque_now_.z = lowPassFilter(torque_.z, torque_prev_.z, dt_, cut_off_frequency_);
    
    geometry_msgs::msg::WrenchStamped ft_data_stamped;
    ft_data_stamped.header.stamp = this->get_clock()->now();  // 현재 시간으로 스탬프 설정
    ft_data_stamped.header.frame_id = "some_frame_id";  // 필요한 경우 frame_id 설정

    geometry_msgs::msg::WrenchStamped ft_data_stamped_fit;
    ft_data_stamped_fit.header.stamp = this->get_clock()->now();  // 현재 시간으로 스탬프 설정
    ft_data_stamped_fit.header.frame_id = "some_frame_id";  // 필요한 경우 frame_id 설정

    ft_data_stamped_fit.wrench.force = force_now_;
    ft_data_stamped_fit.wrench.torque = torque_now_;

    ft_data_stamped.wrench.force = force_;
    ft_data_stamped.wrench.torque = torque_;

    publisher_->publish(ft_data_stamped);
    publisher_fit_->publish(ft_data_stamped_fit);
  }

  inline double lowPassFilter(double input, double prev, double dt, double cut_off_frequency) 
  {
    double alpha = (2*3.1415*dt*cut_off_frequency) / (2*3.1415*dt*cut_off_frequency + 1);
    return alpha*input + (1-alpha)*prev;
  }

  geometry_msgs::msg::Vector3 force_;
  geometry_msgs::msg::Vector3 torque_;
  geometry_msgs::msg::Vector3 force_prev_;
  geometry_msgs::msg::Vector3 torque_prev_;
  geometry_msgs::msg::Vector3 force_now_;
  geometry_msgs::msg::Vector3 torque_now_;
  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr subscriber_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr publisher_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr publisher_fit_;
  double cut_off_frequency_;
  double dt_;
};


int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FTNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

