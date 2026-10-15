#include "core/odometry.hpp"

#include <cmath>

namespace core {

const geo::Pose2D& Odometry::update(const neato::WheelPositions& w) {
    if (!last_) {  // first reading only sets the reference
        last_ = w;
        return pose_;
    }
    const double dl = (w.left_mm - last_->left_mm) / 1000.0;
    const double dr = (w.right_mm - last_->right_mm) / 1000.0;
    last_ = w;

    const double ds = 0.5 * (dl + dr);
    const double dtheta = (dr - dl) / wheelbase_;
    const double mid = pose_.theta + 0.5 * dtheta;
    pose_.x += ds * std::cos(mid);
    pose_.y += ds * std::sin(mid);
    pose_.theta = geo::wrapAngle(pose_.theta + dtheta);
    travelled_ += std::fabs(ds);
    return pose_;
}

}  // namespace core
