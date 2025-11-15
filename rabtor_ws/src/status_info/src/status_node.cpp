#include "rclcpp/rclcpp.hpp"
#include "rex_interfaces/msg/rover_status.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::placeholders;
using namespace std::chrono_literals;



class sub_rover : public rclcpp::Node
{
    public:
    sub_rover():Node("rover"),last_msg_time_(this->now())
    {
        sub_=this->create_subscription
        <rex_interfaces::msg::RoverStatus>
        ("/MQTT/RoverStatus",10,std::bind(&sub_rover::callback,this,_1));

        pub_=this->create_publisher<std_msgs::msg::String>("/Raptors/Status", 10);

        timer_=this->create_wall_timer(5s,std::bind(&sub_rover::check_back,this));

    }
    private:
    int lastmode=0;
    bool is_pad=true;
    int last_communication=1; 
    bool first_message_received_ = false;

    rclcpp::Subscription<rex_interfaces::msg::RoverStatus>::SharedPtr sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time last_msg_time_;

    void callback(const rex_interfaces::msg::RoverStatus & last_status_)
    {
        
        last_msg_time_=this->now();
        first_message_received_=true;

        if(lastmode!=last_status_.control_mode)
        {
            if(last_status_.control_mode<=0)
            {
                 RCLCPP_WARN(this->get_logger(),"the rover published invalid value mode has changed %d",last_status_.control_mode);
            }
            else
            RCLCPP_INFO(this->get_logger(),"the control mode has changed %d",last_status_.control_mode);
        }
       
       if(is_pad!=last_status_.pad_connected)
        {
           RCLCPP_INFO(this->get_logger(),"the pad connected has changed %s", 
            last_status_.pad_connected ? "true" : "false");
        }
        if(last_communication!=last_status_.communication_state)
        {
            if(last_status_.communication_state==0)
            {
                RCLCPP_INFO(this->get_logger(),"communication_state CREATED");

            }
            else if(last_status_.communication_state==1)
            {
                 RCLCPP_INFO(this->get_logger(),"communication_state OPENING");
            }
             else if(last_status_.communication_state==2)
            {
                 RCLCPP_INFO(this->get_logger(),"communication_state OPENED");
            }
             else if(last_status_.communication_state==3)
            {
                 RCLCPP_INFO(this->get_logger(),"communication_state CLOSING");
            }
             else if(last_status_.communication_state==4)
            {
                 RCLCPP_INFO(this->get_logger(),"communication_state CLOSED");
            }
             else if(last_status_.communication_state==5)
            {
                 RCLCPP_INFO(this->get_logger(),"communication_state FAULTED");
            }
            else
            {
             RCLCPP_WARN(this->get_logger(),"the communication_state has got Wrong value ");
            }
        }
 

        lastmode = last_status_.control_mode;
        is_pad  = last_status_.pad_connected;
        last_communication = last_status_.communication_state;


       
    }
    void check_back()
    {
        auto msg = std_msgs::msg::String();
        auto now=this->now();
        auto time_diff=now-last_msg_time_;

        if(!first_message_received_)
        {
            return;
        }

       if (time_diff.seconds() > 3.0) {
            msg.data = "Communication state: " + 
                             std::to_string(last_communication) + " (outdated)";
        } else {
            msg.data = "Communication state: " + 
                             std::to_string(last_communication);
        }
        
        pub_->publish(msg);

    }
};


int main(int argc,char**argv)
{
    rclcpp::init(argc,argv);
    auto node=std::make_shared<sub_rover>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;

}