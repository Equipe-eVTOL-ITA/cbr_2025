#ifndef ARENA_POINTS_CPP
#define ARENA_POINTS_CPP

#include <vector>
#include <Eigen/Eigen>

struct ArenaPoint{
    Eigen::Vector3d coordinates;
    bool is_visited = false;

    // Default constructor
    ArenaPoint() = default;
    
    // Constructor that takes three coordinates
    ArenaPoint(float x, float y, float z) 
        : coordinates(x, y, z), is_visited(false) {}
    
    // Constructor that takes Eigen::Vector3d
    ArenaPoint(const Eigen::Vector3d& coords) 
        : coordinates(coords), is_visited(false) {}
};

ArenaPoint* getNextPoint(std::vector<ArenaPoint>* waypoints) {
    if (!waypoints) return nullptr;
    for (auto& point : *waypoints) {
        if (!point.is_visited) {
            return &point;
        }
    }
    // Return nullptr if no unvisited points are found
    return nullptr;
}

#endif