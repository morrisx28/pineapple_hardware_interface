
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <memory>
#include <mutex>
#include <new>
#include <string>
#include <thread>
#include <csignal>
#include <unordered_map>

#include "pineapple_hardware/pineapple_sdk2_bridge.h"
#include <pthread.h>
#include "yaml-cpp/yaml.h"

struct RealRobotConfig
  {
    int domain_id = 1;
    std::string interface = "lo";

  } config;

static const std::unordered_map<std::string, int> kMotorTypeByName = {
    {"DM3507", damiao::DM3507},
    {"DM4310", damiao::DM4310},
    {"DM4310_48V", damiao::DM4310_48V},
    {"DM4340", damiao::DM4340},
    {"DM4340_48V", damiao::DM4340_48V},
    {"DM6006", damiao::DM6006},
    {"DM6248", damiao::DM6248},
    {"DM8006", damiao::DM8006},
    {"DM8009", damiao::DM8009},
    {"DM10010L", damiao::DM10010L},
    {"DM10010", damiao::DM10010},
    {"DMH3510", damiao::DMH3510},
    {"DMH6215", damiao::DMH6215},
    {"DMS3519", damiao::DMS3519},
    {"DMG6220", damiao::DMG6220},
};

std::atomic<bool> running(true);

void signalHandler(int signum) {
    running = false;
    std::cerr << "\nInterrupt signal (" << signum << ") received.\n";
}

Journaller* gJournal = 0; // Xsens IMU related

int main(int argc, char **argv)
{
    std::signal(SIGINT, signalHandler);
    // *** Get USB2CANFD device ID *** //
    libusb_context* context = nullptr;
    int result = libusb_init(&context);
    if (result < 0) {
        std::cerr << "Failed to initialize libusb: " << libusb_error_name(result) << std::endl;
        return 1;
    }

    // get device list
    libusb_device** devices;
    ssize_t count = libusb_get_device_list(context, &devices);
    if (count < 0) {
        std::cerr << "Failed to obtain device list: " << libusb_error_name(count) << std::endl;
        libusb_exit(context);
        return 1;
    }

    // get serial number
    char serial_number[256] = {0};
    // search for all
    for (int i = 0; devices[i]; i++) {
        libusb_device* device = devices[i];
        
        libusb_device_descriptor desc;
        result = libusb_get_device_descriptor(device, &desc);
        if (result < 0) {
            std::cerr << "Failed to obtain device descriptor: " << libusb_error_name(result) << std::endl;
            continue;
        }
        
        if (desc.idVendor != 0x34B7 || desc.idProduct != 0x6877) {
            continue;
        }
        
        // open device
        libusb_device_handle* handle = nullptr;
        result = libusb_open(device, &handle);
        if (result != LIBUSB_SUCCESS) {
            std::cerr << "Failed to open device: " << libusb_error_name(result) << std::endl;
            return 0;
        }
        
        if (desc.iSerialNumber > 0) {
            result = libusb_get_string_descriptor_ascii(
                handle, 
                desc.iSerialNumber,
                reinterpret_cast<unsigned char*>(serial_number),
                sizeof(serial_number)
            );
            
            if (result < 0) {
                std::cerr << "Failed to obtain serial number: " << libusb_error_name(result) << std::endl;
                serial_number[0] = '\0';
                return 0;
            }
        }
        
        std::cout << "U2CANFD_DEV " << i << ":" << std::endl;
        std::cout << "  VID: 0x" << std::hex << desc.idVendor << std::endl;
        std::cout << "  PID: 0x" << std::hex << desc.idProduct << std::endl;
        std::cout << "  SN: " << (serial_number[0] ? serial_number : "[No serial number]") << std::endl;
        std::cout << std::endl;
        
        libusb_close(handle);
    }
    
    // clear resource 
    libusb_free_device_list(devices, 1);
    libusb_exit(context);

    // Load parameter (config file selects the robot variant, default is the wheeled robot)
    std::string config_path = (argc > 1) ? argv[1] : "../config/config.yaml";
    std::cout << "Loading config: " << config_path << std::endl;
    YAML::Node yaml_node = YAML::LoadFile(config_path);
    config.domain_id = yaml_node["domain_id"].as<int>();
    config.interface = yaml_node["interface"].as<std::string>();

    MotorConfig motor_config;
    motor_config.set_zero = yaml_node["set_zero"].as<bool>();
    motor_config.can_id_list = yaml_node["can_id"].as<std::vector<uint16_t>>();
    motor_config.mst_id_list = yaml_node["mst_id"].as<std::vector<uint16_t>>();
    for (const auto &type_node : yaml_node["motor_type"])
    {
        auto type_name = type_node.as<std::string>();
        auto it = kMotorTypeByName.find(type_name);
        if (it == kMotorTypeByName.end())
        {
            std::cerr << "Unknown motor_type \"" << type_name << "\" in " << config_path << std::endl;
            return 1;
        }
        motor_config.motor_type.push_back(it->second);
    }
    motor_config.motor_offset = yaml_node["motor_offset"].as<std::vector<double>>();
    motor_config.direction = yaml_node["direction"].as<std::vector<double>>();
    for (const auto &limit : yaml_node["pos_limit"])
    {
        auto limit_pair = limit.as<std::vector<double>>();
        motor_config.pos_limit.push_back({limit_pair[0], limit_pair[1]});
    }

    size_t num_motor = motor_config.can_id_list.size();
    if (motor_config.mst_id_list.size() != num_motor ||
        motor_config.motor_type.size() != num_motor ||
        motor_config.motor_offset.size() != num_motor ||
        motor_config.direction.size() != num_motor ||
        motor_config.pos_limit.size() != num_motor)
    {
        std::cerr << "Config error in " << config_path << ": can_id, mst_id, motor_type, "
                  << "motor_offset, direction and pos_limit must all have the same length" << std::endl;
        return 1;
    }

    // Main function
    ChannelFactory::Instance()->Init(config.domain_id, config.interface);
    PineappleSdk2Bridge pineapple_interface(serial_number, motor_config);
    
    while (running) 
    {
        sleep(2);
    }
    
    return 0;
}
