// Basic 2D geometry used everywhere in the project.
// Units: metres and radians. Angles are counter-clockwise, 0 = +x axis.
#pragma once
#include <cmath>
#include <optional>

namespace geo {

constexpr double kPi = 3.14159265358979323846;

inline double deg2rad(double d) { return d * kPi / 180.0; }
inline double rad2deg(double r) { return r * 180.0 / kPi; }

// Keep an angle in the range (-pi, pi]
inline double wrapAngle(double a) {
    while (a > kPi) a -= 2.0 * kPi;
    while (a <= -kPi) a += 2.0 * kPi;
    return a;
}

struct Point2D {
    double x{0.0};
    double y{0.0};
};

// Position and heading of the robot in the world frame
struct Pose2D {
    double x{0.0};
    double y{0.0};
    double theta{0.0};
};

// A wall, piece of furniture edge, etc.
struct Segment {
    Point2D a;
    Point2D b;
};

// Distance from point o along direction `angle` to segment s, if the ray hits it.
// Derivation: solve o + t*d = a + u*e for t (along the ray) and u (along the segment)
// using 2D cross products. A hit needs t >= 0 and 0 <= u <= 1.
inline std::optional<double> raySegment(const Point2D& o, double angle, const Segment& s) {
    const double dx = std::cos(angle), dy = std::sin(angle);
    const double ex = s.b.x - s.a.x, ey = s.b.y - s.a.y;
    const double den = dx * ey - dy * ex;          // d x e
    if (std::fabs(den) < 1e-12) return std::nullopt;  // parallel
    const double wx = s.a.x - o.x, wy = s.a.y - o.y;
    const double t = (wx * ey - wy * ex) / den;    // (w x e) / (d x e)
    const double u = (wx * dy - wy * dx) / den;    // (w x d) / (d x e)
    if (t >= 0.0 && u >= 0.0 && u <= 1.0) return t;
    return std::nullopt;
}

// Transform a point measured in the robot frame into the world frame
inline Point2D robotToWorld(const Pose2D& p, double range, double bearing) {
    return {p.x + range * std::cos(p.theta + bearing),
            p.y + range * std::sin(p.theta + bearing)};
}

}  // namespace geo
