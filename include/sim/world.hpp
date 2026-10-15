// The simulated environment: walls and furniture with surface materials,
// loaded from a plain-text map file (see maps/apartment.map).
// Provides the two physics queries the simulator needs:
//   raycast()          -> what a LiDAR beam hits
//   collidesPolygon()  -> does the robot's footprint touch anything
#pragma once
#include <optional>
#include <string>
#include <vector>

#include "core/geometry.hpp"

namespace sim {

struct Material {
    std::string name;
    double reflectivity{0.8};  // 0..1, how much laser light comes back from a matte surface
    double mirror{0.0};        // 0..1, chance a beam is reflected specularly (glass, gloss)
    std::string color{"cccccc"};
};

struct WallSegment {
    geo::Segment seg;
    int material{0};
};

struct Box {
    std::string name;
    double x0, y0, x1, y1;
    int material{0};
};

struct Circle {
    std::string name;
    geo::Point2D c;
    double r;
    int material{0};
};

struct RayHit {
    double distance;
    int material;
    geo::Point2D point;
    geo::Point2D normal;  // unit normal of the surface at the hit point
};

class World {
public:
    bool load(const std::string& path, std::string& error);

    std::optional<RayHit> raycast(const geo::Point2D& origin, double angle, double max_range) const;

    // True if the closed polygon (robot footprint in world coordinates) touches an obstacle.
    // `contact` receives an approximate contact point.
    bool collidesPolygon(const std::vector<geo::Point2D>& poly, geo::Point2D* contact) const;

    const Material& material(int i) const { return materials_.at(static_cast<size_t>(i)); }
    const std::vector<Material>& materials() const { return materials_; }
    const std::vector<WallSegment>& walls() const { return walls_; }
    const std::vector<Box>& boxes() const { return boxes_; }
    const std::vector<Circle>& circles() const { return circles_; }
    const std::vector<geo::Point2D>& waypoints() const { return waypoints_; }
    std::optional<geo::Pose2D> start() const { return start_; }
    double width() const { return width_; }
    double height() const { return height_; }

private:
    int materialIndex(const std::string& name) const;

    double width_{0.0}, height_{0.0};
    std::vector<Material> materials_;
    std::vector<WallSegment> walls_;
    std::vector<Box> boxes_;
    std::vector<Circle> circles_;
    std::vector<WallSegment> segments_;  // walls + box edges, used for ray casting
    std::vector<geo::Point2D> waypoints_;
    std::optional<geo::Pose2D> start_;
};

}  // namespace sim
