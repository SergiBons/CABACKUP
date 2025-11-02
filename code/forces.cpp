#include "forces.h"

void ForceConstAcceleration::apply() {
    for (Particle* p : particles) {
        p->force += acceleration * p->mass;

    }
}

void ForceDrag::apply() {
    for (Particle* p : particles) {
        if (!p) continue;

        double speed = p->vel.norm();
        if (speed > 1e-12) {
            p->force += - (klinear * p->vel + kquadratic * speed * p->vel);
        }
    }

}

void ForceSpring::apply() {
    if (particles.size() < 2) return;

    Particle* p1 = particles[0];
    Particle* p2 = particles[1];
    if (!p1 || !p2) return;

    Vec3 deltaP = p2->pos - p1->pos;
    double dist = deltaP.norm();
    if (dist == 0) return; // avoid division by zero

    Vec3 dir = deltaP / dist;  // normalized direction
    Vec3 deltaV = p2->vel - p1->vel;

    double fspring = ks * (dist - L);
    double fdamping = kd * (deltaV.dot(dir));

    Vec3 force = (fspring + fdamping) * dir;

    p1->force += force;
    p2->force -= force;
}


void ForceGravitational::apply() {
//TODO
}
