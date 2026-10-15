// Wheel odometry for a differential-drive robot.
// From the change in left/right wheel travel we estimate how the robot moved.
//   ds     = (dl + dr) / 2          distance travelled by the centre
//   dtheta = (dr - dl) / wheelbase  change in heading
// The position update uses the mid-point heading (theta + dtheta/2),
// which is more accurate than using the old heading for curved motion.
#pragma once
#include <optional>

#include "core/geometry.hpp"
#include "neato/robot_interface.hpp"

namespace core {

class Odometry {
public:
    explicit Odometry(double wheelbase_m = 0.248) : wheelbase_(wheelbase_m) {}

    void reset(const geo::Pose2D& start) { pose_ = start; last_.reset(); }

    // Feed the latest cumulative encoder values; returns the updated pose
    const geo::Pose2D& update(const neato::WheelPositions& w);

    const geo::Pose2D& pose() const { return pose_; }
    double distanceTravelled() const { return travelled_; }

private:
    double wheelbase_;
    geo::Pose2D pose_;
    std::optional<neato::WheelPositions> last_;
    double travelled_{0.0};
};

}  // namespace core
