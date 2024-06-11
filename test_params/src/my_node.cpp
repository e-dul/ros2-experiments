#include "rclcpp/rclcpp.hpp"
#include "rclcpp/parameter_map.hpp"
#include "rclcpp/parameter_value.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <string>

class MyNode : public rclcpp::Node {
public:
    MyNode() : Node("my_node") {
        declare_parameter(std::string("validators"), "test");

        //get path to yaml
        std::string pkg_dir = ament_index_cpp::get_package_share_directory("test_params");

        // Read parameters from the YAML file
        readParametersFromFile(pkg_dir + "/config/test.yaml");

        // Read the value of the validated parameter
        readVector4dParameter();
    }

private:
    void readParametersFromFile(const std::string& file_path) {
        // Load parameters from YAML file
        RCLCPP_INFO(get_logger(), "Read YAML from: %s", file_path.c_str());
        const rclcpp::ParameterMap parameters = rclcpp::parameter_map_from_yaml_file(file_path);
        // to make get_parameter work declare what you need from parameters manually
        PrintParams(parameters);
    }

    void PrintParams(rclcpp::ParameterMap parameters) {
        for (const auto& parameter : parameters) {
            RCLCPP_INFO(get_logger(), "node: %s", parameter.first.c_str());
            PrintParams(parameter.second);
        }
    }

    void PrintParams(std::vector<rclcpp::Parameter> parameters) {
        for (const auto& parameter : parameters) {
            RCLCPP_INFO(get_logger(), "name: %s value: %s", parameter.get_name().c_str(), parameter.value_to_string().c_str());
        }
    }

    void readVector4dParameter() {
        RCLCPP_INFO(get_logger(), "Test Param get");
        rclcpp::Parameter param;
        auto result = get_parameter("validators", param);
        if (result) {
            RCLCPP_INFO(get_logger(), "param: %s val: %s", param.get_name().c_str(), param.value_to_string().c_str());
        } else {
            RCLCPP_ERROR(get_logger(), "Failed to get params");
            return;
        }

        // Get the value of the validated parameter
        auto param_value = get_parameter("validators.name");

        if (param_value.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY) {
            const auto& values = param_value.as_double_array();
            if (values.size() == 4) {
                // Print the values of the vector4d parameter
                RCLCPP_INFO(get_logger(), "Vector4d Parameter: [%f, %f, %f, %f]", 
                            values[0], values[1], values[2], values[3]);
            } else {
                RCLCPP_ERROR(get_logger(), "Invalid size for vector4d parameter");
            }
        } else {
            RCLCPP_ERROR(get_logger(), "Invalid type for vector4d parameter %d", param_value.get_type());
        }
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MyNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
