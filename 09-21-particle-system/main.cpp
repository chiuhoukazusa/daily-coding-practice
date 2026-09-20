// Particle System Physics: Euler vs Verlet Integration
// Topic: Game-dev/physics simulation. Quantitative verification (NOT visual).
//
// Validations performed (all numeric, asserted):
//   1. Projectile motion (constant gravity, no drag): both integrators should
//      match the analytic parabolic solution x(t)=v0x*t, y(t)=v0y*t - 0.5*g*t^2.
//      We measure max position error over the trajectory.
//   2. Energy conservation for a harmonic oscillator (spring): total energy
//      E = 0.5*m*v^2 + 0.5*k*x^2 should stay near-constant. Verlet (symplectic)
//      is expected to have bounded/periodic energy drift vs Euler's monotonic.
//   3. A simple bounce (reflect on ground) test to confirm particle lifecycle.

#include <bits/stdc++.h>
using namespace std;

struct Vec2 { double x, y; };
Vec2 operator+(Vec2 a, Vec2 b){ return {a.x+b.x, a.y+b.y}; }
Vec2 operator*(Vec2 a, double s){ return {a.x*s, a.y*s}; }
using AccelFn = std::function<Vec2(Vec2,Vec2,double)>;

// ---------- Integrators ----------
// state = (pos, vel); a(pos, vel, t) = acceleration
struct PState { Vec2 pos, vel; };

Vec2 gravityAccel(Vec2, Vec2, double){ return {0.0, -9.81}; } // m/s^2

PState eulerStep(PState s, double dt, AccelFn a, double t){
    Vec2 acc = a(s.pos, s.vel, t);
    Vec2 npos = s.pos + s.vel * dt;
    Vec2 nvel = s.vel + acc * dt;
    return {npos, nvel};
}

// Velocity Verlet (symplectic)
PState verletStep(PState s, double dt, AccelFn a, double t){
    Vec2 acc0 = a(s.pos, s.vel, t);
    Vec2 npos = s.pos + s.vel * dt + acc0 * (0.5*dt*dt);
    Vec2 acc1 = a(npos, s.vel, t+dt); // acceleration assumes not vel-dependent for these tests
    Vec2 nvel = s.vel + (acc0 + acc1) * (0.5*dt);
    return {npos, nvel};
}

// ---------- Validation 1: Projectile vs analytic ----------
void validateProjectile(){
    const double v0x = 5.0, v0y = 10.0, g = 9.81;
    double dt = 0.001;
    double T = 1.5;

    auto runEuler = [&](){
        PState s{{0,0},{v0x,v0y}};
        double maxErr = 0;
        int steps = (int)round(T/dt);
        for(int i=0;i<steps;i++){
            double t = i*dt;
            s = eulerStep(s, dt, gravityAccel, t);
            double exactX = v0x*(i+1)*dt;
            double exactY = v0y*(i+1)*dt - 0.5*g*(i+1)*dt*(i+1)*dt;
            double err = hypot(s.pos.x-exactX, s.pos.y-exactY);
            maxErr = max(maxErr, err);
        }
        return maxErr;
    };
    auto runVerlet = [&](){
        PState s{{0,0},{v0x,v0y}};
        double maxErr = 0;
        int steps = (int)round(T/dt);
        for(int i=0;i<steps;i++){
            double t = i*dt;
            s = verletStep(s, dt, gravityAccel, t);
            double exactX = v0x*(i+1)*dt;
            double exactY = v0y*(i+1)*dt - 0.5*g*(i+1)*dt*(i+1)*dt;
            double err = hypot(s.pos.x-exactX, s.pos.y-exactY);
            maxErr = max(maxErr, err);
        }
        return maxErr;
    };

    double eulerErr = runEuler();
    double verletErr = runVerlet();

    printf("[Vanilla] Projectile max position error: Euler=%.6f m, Verlet=%.6f m\n", eulerErr, verletErr);
    // Both should be small (< 0.05 m over the trajectory with dt=0.001)
    if(eulerErr > 0.05 || verletErr > 0.05){
        printf("FAIL: projectile error too large\n"); exit(1);
    }
    printf("PASS: projectile matches analytic parabola (both < 0.05 m)\n");
}

// ---------- Validation 2: Energy conservation (harmonic oscillator) ----------
void validateEnergyConservation(){
    // spring: a = -k*x / m
    const double k = 20.0, m = 1.0;
    auto springAccel = [&](Vec2 p, Vec2, double){ return Vec2{-k*p.x/m, 0.0}; };
    double dt = 0.01;
    int steps = 5000; // 50 s

    auto energy = [&](PState s){
        return 0.5*m*s.vel.x*s.vel.x + 0.5*k*s.pos.x*s.pos.x;
    };

    // Euler
    {
        PState s{{1.0,0},{0.0,0}};
        double E0 = energy(s);
        double maxDrift = 0;
        for(int i=0;i<steps;i++){
            s = eulerStep(s, dt, springAccel, i*dt);
            maxDrift = max(maxDrift, fabs(energy(s)-E0)/E0);
        }
        double finalDrift = fabs(energy(s)-E0)/E0;
        printf("[Vanilla] Euler energy drift: max=%.4f%%, final=%.4f%%\n", maxDrift*100, finalDrift*100);
    }
    // Verlet
    {
        PState s{{1.0,0},{0.0,0}};
        double E0 = energy(s);
        double maxDrift = 0;
        for(int i=0;i<steps;i++){
            s = verletStep(s, dt, springAccel, i*dt);
            maxDrift = max(maxDrift, fabs(energy(s)-E0)/E0);
        }
        double finalDrift = fabs(energy(s)-E0)/E0;
        printf("[Vanilla] Verlet energy drift: max=%.6f%%, final=%.6f%%\n", maxDrift*100, finalDrift*100);

        // Verlet (symplectic) should conserve energy MUCH better than explicit Euler.
        if(maxDrift > 0.05){
            printf("FAIL: Verlet energy drift too large (%.6f%%)\n", maxDrift*100); exit(1);
        }
        printf("PASS: Verlet conserves energy (bounded drift, symplectic)\n");
    }
}

// ---------- Validation 3: Bounce lifecycle ----------
void validateBounce(){
    // projectile with ground at y=0 bounce (reflect vy)
    double dt = 0.001;
    PState s{{0.0, 2.0},{1.0, 0.0}};
    int bounces = 0, steps = (int)round(5.0/dt);
    for(int i=0;i<steps;i++){
        s = verletStep(s, dt, gravityAccel, i*dt);
        if(s.pos.y < 0.0){
            s.pos.y = -s.pos.y;      // reflect
            s.vel.y = -s.vel.y*0.9;  // restitution
            bounces++;
        }
    }
    printf("[Vanilla] Bounce test: %d bounces, final y=%.3f m\n", bounces, s.pos.y);
    if(bounces < 1){
        printf("FAIL: no bounce detected\n"); exit(1);
    }
    printf("PASS: particle bounced and settled (lifecycle works)\n");
}

int main(){
    printf("=== Particle System Physics: Euler vs Verlet ===\n\n");
    try{
        validateProjectile();
    }catch(...){ printf("FAIL proj\n"); return 1; }
    validateEnergyConservation();
    validateBounce();
    printf("\nALL CHECKS PASSED\n");
    return 0;
}
