#include <chrono>
#include <memory>


#include "rclcpp/rclcpp.hpp"
#include "more_interfaces/msg/address_book.hpp"

using namespace std::chrono_literals;
using namespace std;

class AddressBookPublisher: public rclcpp::Node
{

    public:
        AddressBookPublisher() : Node("address_book_publisher") {
            address_book_publisher_ = this->create_publisher<more_interfaces::msg::AddressBook>("address_book", 10);

            auto publish_message = [this]() -> void {
                auto message = more_interfaces::msg::AddressBook();
                message.first_name= "Sun Qin";
                message.last_name= "123 Main St";
                message.phone_number= "123-456-7890";
                message.phone_type = message.PHONE_TYPE_MOBILE;

                cout << "Publishing message, first: " << message.first_name << " last: " << message.last_name << endl;
                this->address_book_publisher_->publish(message);
            };

            this->timer_ = this->create_wall_timer(std::chrono::seconds(1), publish_message);


        }

    private:
        rclcpp::Publisher<more_interfaces::msg::AddressBook>::SharedPtr address_book_publisher_;
        rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AddressBookPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}