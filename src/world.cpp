#include "sim/world.hpp"

#include <cmath>
#include <fstream>
#include <sstream>

namespace sim {

namespace {

geo::Point2D closestOnSegment(const geo::Point2D& p, const geo::Segment& s) {
    const double ex = s.b.x - s.a.x, ey = s.b.y - s.a.y;
    const double len2 = ex * ex + ey * ey;
    double t = len2 > 0 ? ((p.x - s.a.x) * ex + (p.y - s.a.y) * ey) / len2 : 0.0;
    t = std::fmax(0.0, std::fmin(1.0, t));
    return {s.a.x + t * ex, s.a.y + t * ey};
}

// Proper segment-segment intersection (including touching), returns the crossing point
bool segmentsIntersect(const geo::Segment& p, const geo::Segment& q, geo::Point2D* where) {
    const double rx = p.b.x - p.a.x, ry = p.b.y - p.a.y;
    const double sx = q.b.x - q.a.x, sy = q.b.y - q.a.y;
    const double den = rx * sy - ry * sx;
    if (std::fabs(den) < 1e-12) return false;  // parallel: ignored, other edges will catch it
    const double qpx = q.a.x - p.a.x, qpy = q.a.y - p.a.y;
    const double t = (qpx * sy - qpy * sx) / den;
    const double u = (qpx * ry - qpy * rx) / den;
    if (t < 0.0 || t > 1.0 || u < 0.0 || u > 1.0) return false;
    if (where) *where = {p.a.x + t * rx, p.a.y + t * ry};
    return true;
}

}  // namespace

int World::materialIndex(const std::string& name) const {
    for (size_t i = 0; i < materials_.size(); ++i)
        if (materials_[i].name == name) return static_cast<int>(i);
    return -1;
}

bool World::load(const std::string& path, std::string& error) {
    std::ifstream f(path);
    if (!f) {
        error = "Cannot open map file " + path;
        return false;
    }
    std::string line;
    int line_no = 0;
    while (std::getline(f, line)) {
        ++line_no;
        const auto hash = line.find('#');
        if (hash != std::string::npos) line.resize(hash);
        std::istringstream in(line);
        std::string cmd;
        if (!(in >> cmd)) continue;

        auto fail = [&](const std::string& what) {
            error = path + ":" + std::to_string(line_no) + ": " + what;
            return false;
        };
        auto mat = [&](const std::string& name) {
            const int m = materialIndex(name);
            if (m < 0) error = path + ":" + std::to_string(line_no) + ": unknown material '" + name + "'";
            return m;
        };

        if (cmd == "room") {
            if (!(in >> width_ >> height_)) return fail("room needs W H");
        } else if (cmd == "material") {
            Material m;
            if (!(in >> m.name >> m.reflectivity >> m.mirror >> m.color)) return fail("bad material line");
            materials_.push_back(m);
        } else if (cmd == "wall") {
            WallSegment w;
            std::string mname;
            if (!(in >> w.seg.a.x >> w.seg.a.y >> w.seg.b.x >> w.seg.b.y >> mname)) return fail("bad wall line");
            if ((w.material = mat(mname)) < 0) return false;
            walls_.push_back(w);
            segments_.push_back(w);
        } else if (cmd == "box") {
            Box b;
            std::string mname;
            if (!(in >> b.name >> b.x0 >> b.y0 >> b.x1 >> b.y1 >> mname)) return fail("bad box line");
            if ((b.material = mat(mname)) < 0) return false;
            boxes_.push_back(b);
            const geo::Point2D c00{b.x0, b.y0}, c10{b.x1, b.y0}, c11{b.x1, b.y1}, c01{b.x0, b.y1};
            for (const auto& s : {geo::Segment{c00, c10}, geo::Segment{c10, c11}, geo::Segment{c11, c01},
                                  geo::Segment{c01, c00}})
                segments_.push_back({s, b.material});
        } else if (cmd == "circle") {
            Circle c;
            std::string mname;
            if (!(in >> c.name >> c.c.x >> c.c.y >> c.r >> mname)) return fail("bad circle line");
            if ((c.material = mat(mname)) < 0) return false;
            circles_.push_back(c);
        } else if (cmd == "start") {
            geo::Pose2D p;
            double deg;
            if (!(in >> p.x >> p.y >> deg)) return fail("start needs x y heading");
            p.theta = geo::deg2rad(deg);
            start_ = p;
        } else if (cmd == "waypoint") {
            geo::Point2D w;
            if (!(in >> w.x >> w.y)) return fail("waypoint needs x y");
            waypoints_.push_back(w);
        } else {
            return fail("unknown command '" + cmd + "'");
        }
    }
    return true;
}

std::optional<RayHit> World::raycast(const geo::Point2D& o, double angle, double max_range) const {
    std::optional<RayHit> best;
    const double dx = std::cos(angle), dy = std::sin(angle);

    for (const auto& w : segments_) {
        const auto t = geo::raySegment(o, angle, w.seg);
        if (!t || *t > max_range || (best && *t >= best->distance)) continue;
        double nx = -(w.seg.b.y - w.seg.a.y), ny = w.seg.b.x - w.seg.a.x;
        const double len = std::hypot(nx, ny);
        nx /= len;
        ny /= len;
        if (nx * dx + ny * dy > 0) { nx = -nx; ny = -ny; }  // normal faces the sensor
        best = RayHit{*t, w.material, {o.x + *t * dx, o.y + *t * dy}, {nx, ny}};
    }

    // Ray-circle: |o + t*d - c|^2 = r^2  ->  t^2 + 2bt + c' = 0
    for (const auto& c : circles_) {
        const double ox = o.x - c.c.x, oy = o.y - c.c.y;
        const double b = ox * dx + oy * dy;
        const double cc = ox * ox + oy * oy - c.r * c.r;
        const double disc = b * b - cc;
        if (disc < 0) continue;
        const double t = -b - std::sqrt(disc);
        if (t < 0 || t > max_range || (best && t >= best->distance)) continue;
        const geo::Point2D p{o.x + t * dx, o.y + t * dy};
        best = RayHit{t, c.material, p, {(p.x - c.c.x) / c.r, (p.y - c.c.y) / c.r}};
    }
    return best;
}

bool World::collidesPolygon(const std::vector<geo::Point2D>& poly, geo::Point2D* contact) const {
    const size_t n = poly.size();
    for (size_t i = 0; i < n; ++i) {
        const geo::Segment edge{poly[i], poly[(i + 1) % n]};
        for (const auto& w : segments_)
            if (segmentsIntersect(edge, w.seg, contact)) return true;
        for (const auto& c : circles_) {
            const auto q = closestOnSegment(c.c, edge);
            if (std::hypot(q.x - c.c.x, q.y - c.c.y) < c.r) {
                if (contact) *contact = q;
                return true;
            }
        }
    }
    return false;
}

}  // namespace sim
