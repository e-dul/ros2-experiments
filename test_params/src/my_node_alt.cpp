#include "rclcpp/rclcpp.hpp"
#include "rclcpp/parameter_map.hpp"
#include "rclcpp/parameter_value.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <string>

class MyNodeAlt : public rclcpp::Node {
public:
    MyNodeAlt(const rclcpp::NodeOptions& opt) : Node("my_node_alt", opt) {
        std::map<std::string, rclcpp::Parameter> validators;
        if (get_parameters("validators", validators)) {
            PrintParams(validators);
        } else {
            RCLCPP_ERROR(get_logger(), "Can't read params");
        }
    }

private:
    void PrintParams(std::map<std::string, rclcpp::Parameter> parameters) {
        for (const auto& parameter : parameters) {
            RCLCPP_INFO(get_logger(), "name: %s value: %s", parameter.first.c_str(), parameter.second.value_to_string().c_str());
            // RCLCPP_INFO(get_logger(), "node: %s", parameter.first.c_str());
            // PrintParams(parameter.second);
        }
    }

    void PrintParams(std::vector<rclcpp::Parameter> parameters) {
        for (const auto& parameter : parameters) {
            RCLCPP_INFO(get_logger(), "name: %s value: %s", parameter.get_name().c_str(), parameter.value_to_string().c_str());
        }
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions opt;
    opt.allow_undeclared_parameters(true).automatically_declare_parameters_from_overrides(true);
    auto node = std::make_shared<MyNodeAlt>(opt);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
