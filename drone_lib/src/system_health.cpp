#include "drone/SystemHealth.hpp"
#include <sstream>
#include <cstdio>

SystemHealth::SystemHealth() : Node("system_health") {
    // Create publisher for system health telemetry
    system_health_pub_ = this->create_publisher<custom_msgs::msg::SystemHealth>("/telemetry/system_health", 10);
    
    // Create timer for 1Hz health monitoring
    health_timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&SystemHealth::publishSystemHealth, this));
    
    RCLCPP_INFO(this->get_logger(), "System health monitoring node initialized");
}

void SystemHealth::publishSystemHealth() {
    auto msg = custom_msgs::msg::SystemHealth();
    msg.header.stamp = this->get_clock()->now();
    
    // Get system metrics
    msg.cpu_percent = getCpuUsage();
    msg.memory_percent = getMemoryUsage();
    msg.temperature = getCpuTemperature();
    msg.disk_usage_percent = getDiskUsage();
    
    system_health_pub_->publish(msg);
}

float SystemHealth::getCpuUsage() {
    // Simple CPU usage approximation using load average
    std::ifstream file("/proc/loadavg");
    if (!file.is_open()) return 0.0f;
    
    float load;
    file >> load;
    file.close();
    
    // Convert load average to approximate percentage (assuming single core baseline)
    return std::min(load * 100.0f, 100.0f);
}

float SystemHealth::getMemoryUsage() {
    std::ifstream file("/proc/meminfo");
    if (!file.is_open()) return 0.0f;
    
    std::string line;
    float total = 0, available = 0;
    
    while (std::getline(file, line)) {
        if (line.find("MemTotal:") == 0) {
            sscanf(line.c_str(), "MemTotal: %f kB", &total);
        } else if (line.find("MemAvailable:") == 0) {
            sscanf(line.c_str(), "MemAvailable: %f kB", &available);
            break; // We have both values
        }
    }
    file.close();
    
    if (total > 0) {
        return ((total - available) / total) * 100.0f;
    }
    return 0.0f;
}

float SystemHealth::getCpuTemperature() {
    std::ifstream file("/sys/class/thermal/thermal_zone0/temp");
    if (!file.is_open()) return 0.0f;
    
    float temp;
    file >> temp;
    file.close();
    
    // Convert from millidegrees to degrees Celsius
    return temp / 1000.0f;
}

float SystemHealth::getDiskUsage() {
    struct statvfs stat;
    if (statvfs("/", &stat) != 0) return 0.0f;
    
    unsigned long total = stat.f_blocks * stat.f_frsize;
    unsigned long free = stat.f_bavail * stat.f_frsize;
    
    if (total > 0) {
        return ((float)(total - free) / total) * 100.0f;
    }
    return 0.0f;
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SystemHealth>());
    rclcpp::shutdown();
    return 0;
}
