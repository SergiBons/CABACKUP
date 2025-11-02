#include "colliders.h"
#include <cmath>
#include <algorithm> // for std::clamp

/*
 * Generic function for collision response from contact plane
 */
void Collider::resolveCollision(Particle* p, const Collision& col, double kElastic, double kFriction) const
{
    Vec3 n = col.normal.normalized();

    Vec3 v = p->pos - p->prevPos;
    double vLen = v.norm();
    if (vLen < 1e-12) return; // avoid degenerate cases


    double t = (col.position - p->prevPos).norm() / vLen;
    t = std::clamp(t, 0.0, 1.0);

    // Position at collision
    Vec3 preCollisionPos = p->prevPos + v * t;
    Vec3 remaining = v * (1.0 - t);

    // Decompose remaining motion
    double remDotN = remaining.dot(n);
    Vec3 remN = remDotN * n;
    Vec3 remT = remaining - remN;

    // Apply restitution and friction
    Vec3 newRemN = -kElastic * remN;
    Vec3 newRemT = (1.0 - kFriction) * remT;
    Vec3 newRemaining = newRemN + newRemT;

    p->pos = col.position + newRemaining ;
    p->prevPos = p->pos - newRemaining;
}


/*
 * Plane
 */
bool ColliderPlane::isInside(const Particle* p) const
{
    double dist = planeN.dot(p->pos) + planeD;
    return dist <= 0.0;
}


bool ColliderPlane::testCollision(const Particle* p, Collision& colInfo) const
{
    double R = p->radius;

    // dist0/dist1 represent distance from the particle SURFACE to the plane
    double dist0 = planeN.dot(p->prevPos) + planeD - R;
    double dist1 = planeN.dot(p->pos) + planeD - R;

    // Collision occurs if the surface is now past the plane (dist1 <= 0.0)
    if (dist1 <= 0.0)
    {
        colInfo.normal = planeN.normalized();
        // Calculate t for when SURFACE crossed the plane
        double t = dist0 / (dist0 - dist1);
        colInfo.position = p->prevPos + t * (p->pos - p->prevPos);
        colInfo.penetration = -dist1;
        return true;
    }
    return false;
}


/*
 * Sphere
 */
bool ColliderSphere::isInside(const Particle* p) const
{
    double distSq = (p->pos - center).squaredNorm();
    return distSq <= radius * radius;
}


bool ColliderSphere::testCollision(const Particle* p, Collision& colInfo) const
{
    Vec3 dir = p->pos - p->prevPos;
    Vec3 m = p->prevPos - center;

    double a = dir.dot(dir);
    double b = 2.0 * m.dot(dir);
    double c = m.dot(m) - radius * radius;

    double discriminant = b * b - 4.0 * a * c;
    if (discriminant < 0.0)
        return false; // no intersection

    double sqrtDisc = std::sqrt(discriminant);

    // find smallest positive root (entering point)
    double t0 = (-b - sqrtDisc) / (2.0 * a);
    double t1 = (-b + sqrtDisc) / (2.0 * a);

    double t = std::min(t0, t1);
    if (t < 0.0 || t > 1.0)
        return false; // intersection outside the motion segment

    Vec3 tempPos = p->prevPos + t * dir;
    colInfo.normal = (tempPos - center).normalized();
    colInfo.position = tempPos-(p->pos -p->prevPos).normalized()*p->radius;
    return true;
}


/*
 * Axis-Aligned Bounding Box (AABB)
 */
bool ColliderAABB::isInside(const Particle* p) const
{
    const Vec3& pos = p->pos;
    return (pos.x() >= bmin.x() && pos.x() <= bmax.x() &&
            pos.y() >= bmin.y() && pos.y() <= bmax.y() &&
            pos.z() >= bmin.z() && pos.z() <= bmax.z());
}


bool ColliderAABB::testCollision(const Particle* p, Collision& colInfo) const
{
    Vec3 dir = p->pos - p->prevPos;

    // Avoid divide by zero by clamping near-zero directions
    Vec3 invDir;
    invDir.x() = (std::abs(dir.x()) > 1e-12) ? 1.0 / dir.x() : 1e12;
    invDir.y() = (std::abs(dir.y()) > 1e-12) ? 1.0 / dir.y() : 1e12;
    invDir.z() = (std::abs(dir.z()) > 1e-12) ? 1.0 / dir.z() : 1e12;

    Vec3 t0s = (bmin - p->prevPos).cwiseProduct(invDir);
    Vec3 t1s = (bmax - p->prevPos).cwiseProduct(invDir);

    Vec3 tMinVec = t0s.cwiseMin(t1s);
    Vec3 tMaxVec = t0s.cwiseMax(t1s);

    double tMin = tMinVec.maxCoeff();
    double tMax = tMaxVec.minCoeff();

    if (tMax < 0.0 || tMin > tMax || tMin > 1.0)
        return false;

    double t = std::clamp(tMin, 0.0, 1.0);

    colInfo.position = p->prevPos + t * dir;

    // Determine collision normal based on which face was hit
    Vec3 hitPoint = colInfo.position;
    Vec3 n = Vec3::Zero();
    const double epsilon = 1e-6;

    if (std::abs(hitPoint.x() - bmin.x()) < epsilon) n = Vec3(-1, 0, 0);
    else if (std::abs(hitPoint.x() - bmax.x()) < epsilon) n = Vec3(1, 0, 0);
    else if (std::abs(hitPoint.y() - bmin.y()) < epsilon) n = Vec3(0, -1, 0);
    else if (std::abs(hitPoint.y() - bmax.y()) < epsilon) n = Vec3(0, 1, 0);
    else if (std::abs(hitPoint.z() - bmin.z()) < epsilon) n = Vec3(0, 0, -1);
    else if (std::abs(hitPoint.z() - bmax.z()) < epsilon) n = Vec3(0, 0, 1);

    colInfo.normal = n;
    return true;
}

// In a new class definition or in colliders.cpp/h

bool ColliderHourglass::isInside(const Particle* p) const {
    const Vec3& pos = p->pos;
    float x = pos.x();
    float y = pos.y();
    float z = pos.z();
    float r2 = x*x + z*z;

    // Check against y-axis bounds
    if (y > height_half || y < -height_half) {
        return false; // Particle is outside the main height boundaries
    }

    // Calculate local cone radius R_y at current height y
    float y_abs = std::abs(y);
    // R_y linearly interpolates between radius_neck at y=0 and radius_max at y=height_half
    float R_y = radius_neck + (radius_max - radius_neck) * (y_abs / height_half);

    // Check against the conical surface
    if (r2 > R_y * R_y) {
        return false; // Particle position is outside the conical boundary
    }

    return true; // Position is inside the hourglass walls
}

// 2. ColliderHourglass::testCollision (Main logic)
bool ColliderHourglass::testCollision(const Particle* p, Collision& colInfo) const {
    const Vec3& pos = p->pos;
    float R = p->radius; // Particle radius
    float x = pos.x();
    float y = pos.y();
    float z = pos.z();
    float r_xz = std::sqrt(x*x + z*z); // Radial distance from y-axis

    // --- 1. Vertical Cap Collision (Top/Bottom) ---
    if (y > height_half - R) { // Collision with Top Cap (Plane: y = height_half)
            if (r_xz < radius_max - 1e-6f) {
            colInfo.normal = Vec3(0, 1, 0); // Upward normal
            // Penetration depth: Distance from sphere center (p->pos.y) to boundary (height_half) + radius
            colInfo.penetration = (y - height_half) + R;
            colInfo.position = pos - colInfo.normal * (colInfo.penetration - R); // Collision point on surface
            return true;
            }
    }
    if (y < -height_half + R) { // Collision with Bottom Cap (Plane: y = -height_half)
        if (r_xz < radius_max - 1e-6f) {
            colInfo.normal = Vec3(0, -1, 0); // Downward normal
            colInfo.penetration = (-height_half - y) + R;
            colInfo.position = pos - colInfo.normal * (colInfo.penetration - R);
            return true;
        }
    }

    // --- 2. Conical Surface Collision ---
    // Calculate local cone radius R_y at current height y
    float y_abs = std::abs(y);
    float R_y = radius_neck + (radius_max - radius_neck) * (y_abs / height_half);

    // Check if the particle center is outside the cone wall + particle radius
    if (r_xz > R_y + R) {
        // Collision with the cone wall

        //Calculate the slope (m) of the cone wall
        float m = (radius_max - radius_neck) / height_half;

        // Calculate the collision normal (N) which is perpendicular to the cone wall.
        // The cone wall slope is (dy/dr) = (height_half / (radius_max - radius_neck)).

        float N_r = height_half;
        float N_y_mag = radius_max - radius_neck;
        float N_y = (y > 0) ? -N_y_mag : N_y_mag; // Ensures N_y points upward for y>0 and downward for y<0

        // 3. Normalize the (N_r, N_y) vector in the (r, y) plane:
        float N_mag_sq = N_r*N_r + N_y_mag*N_y_mag; // Note: Use N_y_mag in magnitude calculation
        float N_mag = std::sqrt(N_mag_sq);

        // 4. Calculate the 3D Normal:
        if (r_xz < 1e-6f) {
            return false;
        } else {
            // Full 3D Normal: (N_r * x/r_xz, N_y, N_r * z/r_xz)
            colInfo.normal = Vec3(N_r * x / r_xz / N_mag, N_y / N_mag, N_r * z / r_xz / N_mag);
        }

        float excess_radius = r_xz - R_y;

        // The true distance is the excess distance projected onto the normal:
        // Since N is normalized, the distance to the plane is (N dot (P - P_on_plane))
        // We can approximate penetration as the radial excess + particle radius, or a more exact geometric distance.

        // Using the radial excess distance (conservative approximation):
        colInfo.penetration = excess_radius + R;

        // Collision point is along the normal
        colInfo.position = pos - colInfo.normal * (colInfo.penetration - R);

        return true;
    }

    // No collision found
    return false;
};
