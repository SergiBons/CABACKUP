#include "integrators.h"
#include <iostream>

void IntegratorEuler::step(ParticleSystem &system, double dt) {
    double t0 = system.getTime();
    Vecd x0 = system.getState();
    Vecd dx = system.getDerivative();
    Vecd x1 = x0 + dt*dx;
    system.setState(x1);
    system.setTime(t0+dt);
    system.updateForces();
}


void IntegratorSymplecticEuler::step(ParticleSystem &system, double dt) {
    double t0 = system.getTime();
    //Get velocity for Symplectic Euler
    Vecd x0 = system.getPositions();
    Vecd v0 = system.getVelocities();
    Vecd a0 = system.getAccelerations();

    //Compute next step velocity
    Vecd v1 = v0 + dt*a0;

    Vecd x1 = x0 + dt*v1;
    system.setPositions(x1);
    system.setPreviousPositions(x0);
    system.setVelocities(v1);
    system.setTime(t0+dt);
    system.updateForces();

}


void IntegratorMidpoint::step(ParticleSystem &system, double dt) {
    system.updateForces();
    double t0 = system.getTime();
    Vecd x0 = system.getPositions();
    Vecd v0 = system.getVelocities();
    Vecd dx0 = v0;                     // dx/dt = velocity
    Vecd dv0 = system.getAccelerations(); // dv/dt = acceleration

    // Midpoint estimates
    Vecd x_mid = x0 + 0.5 * dt * dx0;
    Vecd v_mid = v0 + 0.5 * dt * dv0;

    // Evaluate derivative at midpoint
    system.setPositions(x_mid);
    system.setVelocities(v_mid);
    system.setTime(t0 + 0.5 * dt);
    system.updateForces(); // refresh accelerations

    Vecd dv_mid = system.getAccelerations();
    Vecd dx_mid = v_mid;

    // Step to full time
    Vecd x1 = x0 + dt * dx_mid;
    Vecd v1 = v0 + dt * dv_mid;

    // Update system
    system.setPositions(x1);
    system.setVelocities(v1);
    system.setTime(t0 + dt);
    system.updateForces();
}


void IntegratorRK2::step(ParticleSystem &system, double dt) {
    system.updateForces();
    double t0 = system.getTime();
    Vecd x0 = system.getPositions();
    Vecd v0 = system.getVelocities();

    Vecd dx0 = v0;                      // dx/dt = v
    Vecd dv0 = system.getAccelerations(); // dv/dt = a

    Vecd x_temp = x0 + dt * dx0;
    Vecd v_temp = v0 + dt * dv0;

    system.setPositions(x_temp);
    system.setVelocities(v_temp);
    system.setTime(t0 + dt);
    system.updateForces();

    Vecd dx1 = v_temp;
    Vecd dv1 = system.getAccelerations();

    Vecd x1 = x0 + 0.5 * dt * (dx0 + dx1);
    Vecd v1 = v0 + 0.5 * dt * (dv0 + dv1);

    system.setPositions(x1);
    system.setVelocities(v1);
    system.setTime(t0 + dt);
    system.updateForces();
}


void IntegratorRK4::step(ParticleSystem &system, double dt) {
    system.updateForces();
    double t0 = system.getTime();

    // Initial state
    Vecd x0 = system.getPositions();
    Vecd v0 = system.getVelocities();
    //k1
    Vecd k1x = v0;
    Vecd k1v = system.getAccelerations();
    //k2
    Vecd x_temp = x0 + 0.5 * dt * k1x;
    Vecd v_temp = v0 + 0.5 * dt * k1v;
    system.setPositions(x_temp);
    system.setVelocities(v_temp);
    system.setTime(t0 + 0.5 * dt);
    system.updateForces();
    Vecd k2x = v_temp;
    Vecd k2v = system.getAccelerations();
    //k3
    x_temp = x0 + 0.5 * dt * k2x;
    v_temp = v0 + 0.5 * dt * k2v;
    system.setPositions(x_temp);
    system.setVelocities(v_temp);
    system.setTime(t0 + 0.5 * dt);
    system.updateForces();
    Vecd k3x = v_temp;
    Vecd k3v = system.getAccelerations();
    //k4
    x_temp = x0 + dt * k3x;
    v_temp = v0 + dt * k3v;
    system.setPositions(x_temp);
    system.setVelocities(v_temp);
    system.setTime(t0 + dt);
    system.updateForces();
    Vecd k4x = v_temp;
    Vecd k4v = system.getAccelerations();
    //Combine
    Vecd x1 = x0 + (dt / 6.0) * (k1x + 2.0 * k2x + 2.0 * k3x + k4x);
    Vecd v1 = v0 + (dt / 6.0) * (k1v + 2.0 * k2v + 2.0 * k3v + k4v);
    //Final update
    system.setPositions(x1);
    system.setVelocities(v1);
    system.setTime(t0 + dt);
    system.updateForces();
}


void IntegratorVerlet::step(ParticleSystem &system, double dt) {
    system.updateForces();
    double t0 = system.getTime();

    Vecd x  = system.getPositions();
    Vecd px = system.getPreviousPositions();
    Vecd a  = system.getAccelerations();
    // Damping factor
    double k = 0.99;
    // Verlet position update
    Vecd xf = x + k * (x - px) + (dt * dt) * a;

    system.setPreviousPositions(x);  // current becomes previous
    system.setPositions(xf);
    system.setTime(t0 + dt);
    system.updateForces();

    //reconstruct velocity
    Vecd vf = (xf - px) / (2.0 * dt);
    system.setVelocities(vf);
}
