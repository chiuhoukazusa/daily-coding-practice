#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <iomanip>
#include <algorithm>
#include <string>

// ============================================================
// RK4 Runge-Kutta ODE Solver - Spring-Mass-Damper System
// ============================================================
// Solves: m*x'' + c*x' + k*x = 0  (damped harmonic oscillator)
// State vector: [x, v] where v = x'
// ODE system: dx/dt = v,  dv/dt = -(c/m)*v - (k/m)*x

struct State {
    double x;  // position
    double v;  // velocity
};

// Physical parameters
const double MASS   = 1.0;    // kg
const double SPRING = 10.0;   // N/m (spring constant)
const double DAMPING = 0.5;   // N·s/m (small damping so energy loss is measurable but not instant)

// Initial conditions
const double X0 = 1.0;   // initial position
const double V0 = 0.0;   // initial velocity

// Compute derivatives for state
State derivatives(const State& s) {
    double a = -(DAMPING / MASS) * s.v - (SPRING / MASS) * s.x;
    return {s.v, a};
}

// Compute total mechanical energy (kinetic + potential)
// For damped system, energy = 0.5*m*v^2 + 0.5*k*x^2
double energy(const State& s) {
    return 0.5 * MASS * s.v * s.v + 0.5 * SPRING * s.x * s.x;
}

// Euler integration (forward Euler)
std::vector<State> euler_integrate(double dt, double t_end) {
    std::vector<State> states;
    State s = {X0, V0};
    states.push_back(s);
    
    for (double t = 0; t < t_end; t += dt) {
        State d = derivatives(s);
        s.x += d.x * dt;
        s.v += d.v * dt;
        states.push_back(s);
    }
    return states;
}

// RK4 integration (4th order Runge-Kutta)
std::vector<State> rk4_integrate(double dt, double t_end) {
    std::vector<State> states;
    State s = {X0, V0};
    states.push_back(s);
    
    for (double t = 0; t < t_end; t += dt) {
        // k1
        State k1 = derivatives(s);
        
        // k2
        State s2 = {s.x + 0.5 * dt * k1.x, s.v + 0.5 * dt * k1.v};
        State k2 = derivatives(s2);
        
        // k3
        State s3 = {s.x + 0.5 * dt * k2.x, s.v + 0.5 * dt * k2.v};
        State k3 = derivatives(s3);
        
        // k4
        State s4 = {s.x + dt * k3.x, s.v + dt * k3.v};
        State k4 = derivatives(s4);
        
        // Combine
        s.x += (dt / 6.0) * (k1.x + 2*k2.x + 2*k3.x + k4.x);
        s.v += (dt / 6.0) * (k1.v + 2*k2.v + 2*k3.v + k4.v);
        states.push_back(s);
    }
    return states;
}

// Analytical solution for underdamped case: x(t) = A * e^(-ζω₀t) * cos(ω_d*t + φ)
// ω₀ = sqrt(k/m), ζ = c/(2*sqrt(km)), ω_d = ω₀*sqrt(1-ζ²)
struct AnalyticalResult {
    std::vector<State> states;
    double final_energy;
};

AnalyticalResult analytical_solution(double dt, double t_end) {
    double omega0 = std::sqrt(SPRING / MASS);
    double zeta   = DAMPING / (2.0 * std::sqrt(SPRING * MASS));
    double omega_d = omega0 * std::sqrt(1.0 - zeta * zeta);
    
    // Initial conditions: x(0)=X0, v(0)=V0
    // x(t) = e^{-ζω₀t} * [X0*cos(ω_d*t) + ((V0+ζω₀*X0)/ω_d)*sin(ω_d*t)]
    
    std::vector<State> states;
    double A = X0;
    double B = (V0 + zeta * omega0 * X0) / omega_d;
    
    for (double t = 0; t <= t_end + 0.5*dt; t += dt) {
        double exp_decay = std::exp(-zeta * omega0 * t);
        double x = exp_decay * (A * std::cos(omega_d * t) + B * std::sin(omega_d * t));
        double v = exp_decay * (
            -zeta * omega0 * (A * std::cos(omega_d * t) + B * std::sin(omega_d * t))
            + (-A * omega_d * std::sin(omega_d * t) + B * omega_d * std::cos(omega_d * t))
        );
        states.push_back({x, v});
    }
    
    double final_e = 0.5 * MASS * states.back().v * states.back().v 
                   + 0.5 * SPRING * states.back().x * states.back().x;
    return {states, final_e};
}

// Write PPM image
void write_ppm(const std::string& filename, 
               const std::vector<State>& euler, 
               const std::vector<State>& rk4,
               const std::vector<State>& analytical,
               double dt_euler, double dt_rk4,
               double t_end) {
    int width = 1000;
    int height = 700;
    int margin = 60;
    int plot_width = width - 2 * margin;
    int plot_height = height - 2 * margin;
    
    std::vector<std::vector<int>> r(width, std::vector<int>(height, 255));
    std::vector<std::vector<int>> g(width, std::vector<int>(height, 255));
    std::vector<std::vector<int>> b(width, std::vector<int>(height, 255));
    
    // White background
    for (int i = 0; i < width; i++)
        for (int j = 0; j < height; j++)
            r[i][j] = g[i][j] = b[i][j] = 255;
    
    // Draw axes
    int origin_x = margin;
    int origin_y = height / 2;
    int zero_line_y = origin_y;
    
    // x-axis (time)
    for (int i = margin; i < width - margin; i++) {
        r[i][zero_line_y] = g[i][zero_line_y] = b[i][zero_line_y] = 128;
    }
    // y-axis
    for (int j = margin; j < height - margin; j++) {
        r[origin_x][j] = g[origin_x][j] = b[origin_x][j] = 128;
    }
    
    // Grid lines
    for (int i = margin; i < width - margin; i += 50) {
        for (int j = margin; j < height - margin; j++) {
            r[i][j] = std::min(255, r[i][j] - 30);
            g[i][j] = std::min(255, g[i][j] - 30);
            b[i][j] = std::min(255, b[i][j] - 30);
        }
    }
    for (int j = margin; j < height - margin; j += 50) {
        for (int i = margin; i < width - margin; i++) {
            r[i][j] = std::min(255, r[i][j] - 30);
            g[i][j] = std::min(255, g[i][j] - 30);
            b[i][j] = std::min(255, b[i][j] - 30);
        }
    }
    
    // Plot function: map (t, x) to pixel
    double y_scale = plot_height / 2.2;  // amplitude range ~ ±1.1
    double x_scale = (double)plot_width / t_end;
    
    auto plot_curve = [&](const std::vector<State>& states, double dt, int cr, int cg, int cb) {
        for (size_t i = 0; i < states.size(); i++) {
            double t = i * dt;
            int px = margin + (int)(t * x_scale);
            int py = zero_line_y - (int)(states[i].x * y_scale);
            if (px >= margin && px < width - margin && py >= margin && py < height - margin) {
                r[px][py] = cr;
                g[px][py] = cg;
                b[px][py] = cb;
                // Draw 2x2 for visibility
                for (int dx = 0; dx < 2 && px+dx < width-margin; dx++)
                    for (int dy = 0; dy < 2 && py+dy < height-margin; dy++) {
                        r[px+dx][py+dy] = cr;
                        g[px+dx][py+dy] = cg;
                        b[px+dx][py+dy] = cb;
                    }
            }
        }
    };
    
    // Euler = Red, RK4 = Blue, Analytical = Black dots
    plot_curve(analytical, dt_rk4, 0, 0, 0);
    plot_curve(euler, dt_euler, 255, 50, 50);
    plot_curve(rk4, dt_rk4, 50, 50, 255);
    
    // Energy subplot (bottom half or separate section)
    // We'll draw energy error on the bottom portion
    
    // Write PPM
    std::ofstream out(filename);
    out << "P3\n" << width << " " << height << "\n255\n";
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            out << r[x][y] << " " << g[x][y] << " " << b[x][y] << " ";
        }
    }
    out.close();
}

// Write side-by-side comparison PPM
void write_comparison_ppm(const std::string& filename,
                          const std::vector<double>& euler_energy,
                          const std::vector<double>& rk4_energy,
                          double dt, double t_end) {
    int img_w = 1000, img_h = 500;
    int margin = 60;
    int plot_w = img_w - 2 * margin;
    int plot_h = img_h - 2 * margin;
    
    std::vector<std::vector<int>> r(img_w, std::vector<int>(img_h, 255));
    std::vector<std::vector<int>> g(img_w, std::vector<int>(img_h, 255));
    std::vector<std::vector<int>> b(img_w, std::vector<int>(img_h, 255));
    
    // Grid
    for (int i = margin; i < img_w - margin; i += 50)
        for (int j = margin; j < img_h - margin; j++)
            r[i][j] = g[i][j] = b[i][j] = 235;
    for (int j = margin; j < img_h - margin; j += 50)
        for (int i = margin; i < img_w - margin; i++)
            r[i][j] = g[i][j] = b[i][j] = 235;
    
    double max_energy = energy({X0, V0});
    double y_max = max_energy * 1.3;
    double x_scale = (double)plot_w / t_end;
    
    // Plot Euler energy as red line, RK4 energy as blue line
    auto plot_line = [&](const std::vector<double>& energy_vec, int cr, int cg, int cb) {
        for (size_t i = 0; i < energy_vec.size(); i++) {
            double t = i * dt;
            int px = margin + (int)(t * x_scale);
            int py = margin + plot_h - (int)(energy_vec[i] / y_max * plot_h);
            if (px >= margin && px < img_w - margin && py >= margin && py < img_h - margin) {
                for (int dx = 0; dx < 2; dx++)
                    for (int dy = -2; dy < 2; dy++)
                        if (px+dx < img_w-margin && py+dy >= margin && py+dy < img_h-margin) {
                            r[px+dx][py+dy] = cr;
                            g[px+dx][py+dy] = cg;
                            b[px+dx][py+dy] = cb;
                        }
            }
        }
    };
    
    plot_line(euler_energy, 255, 60, 60);
    plot_line(rk4_energy, 60, 60, 255);
    
    // Axis
    for (int i = margin; i < img_w - margin; i++)
        r[i][margin] = g[i][margin] = b[i][margin] = 128;
    for (int j = margin; j < img_h - margin; j++)
        r[margin][j] = g[margin][j] = b[margin][j] = 128;
    
    std::ofstream out(filename);
    out << "P3\n" << img_w << " " << img_h << "\n255\n";
    for (int y = 0; y < img_h; y++)
        for (int x = 0; x < img_w; x++)
            out << r[x][y] << " " << g[x][y] << " " << b[x][y] << " ";
    out.close();
}

int main() {
    // Use a coarse timestep to clearly show Euler vs RK4 difference
    double dt = 0.05;
    double t_end = 10.0;
    
    std::cout << "==============================================" << std::endl;
    std::cout << " RK4 vs Euler ODE Solver - Quantitative Analysis" << std::endl;
    std::cout << "==============================================" << std::endl;
    std::cout << "System: Spring-mass-damper" << std::endl;
    std::cout << "  m=" << MASS << " kg, k=" << SPRING << " N/m, c=" << DAMPING << " N·s/m" << std::endl;
    std::cout << "  Initial: x0=" << X0 << " m, v0=" << V0 << " m/s" << std::endl;
    std::cout << "  Initial energy: " << energy({X0, V0}) << " J" << std::endl;
    std::cout << "  dt=" << dt << " s, t_end=" << t_end << " s" << std::endl;
    
    double omega0 = std::sqrt(SPRING / MASS);
    double zeta = DAMPING / (2.0 * std::sqrt(SPRING * MASS));
    std::cout << "  ω₀=" << std::fixed << std::setprecision(3) << omega0 
              << " rad/s, ζ=" << zeta << " (underdamped)" << std::endl;
    std::cout << std::endl;
    
    // Run integrations
    auto euler_states = euler_integrate(dt, t_end);
    auto rk4_states   = rk4_integrate(dt, t_end);
    auto analytical   = analytical_solution(dt, t_end);
    
    // ============================================
    // Quantitative Verification #1: Compare final state with analytical
    // ============================================
    std::cout << "=== Verification #1: Final State Comparison ===" << std::endl;
    std::cout << std::setw(20) << "Method" 
              << std::setw(16) << "Final x (m)"
              << std::setw(16) << "Final v (m/s)"
              << std::setw(16) << "|x error|"
              << std::setw(16) << "|v error|" << std::endl;
    std::cout << std::string(84, '-') << std::endl;
    
    double ana_x = analytical.states.back().x;
    double ana_v = analytical.states.back().v;
    
    double euler_x = euler_states.back().x;
    double euler_v = euler_states.back().v;
    double euler_x_err = std::abs(euler_x - ana_x);
    double euler_v_err = std::abs(euler_v - ana_v);
    
    double rk4_x = rk4_states.back().x;
    double rk4_v = rk4_states.back().v;
    double rk4_x_err = std::abs(rk4_x - ana_x);
    double rk4_v_err = std::abs(rk4_v - ana_v);
    
    std::cout << std::scientific << std::setprecision(6);
    std::cout << std::setw(20) << "Analytical" 
              << std::setw(16) << ana_x
              << std::setw(16) << ana_v
              << std::setw(16) << "0"
              << std::setw(16) << "0" << std::endl;
    std::cout << std::setw(20) << "Euler" 
              << std::setw(16) << euler_x
              << std::setw(16) << euler_v
              << std::setw(16) << euler_x_err
              << std::setw(16) << euler_v_err << std::endl;
    std::cout << std::setw(20) << "RK4" 
              << std::setw(16) << rk4_x
              << std::setw(16) << rk4_v
              << std::setw(16) << rk4_x_err
              << std::setw(16) << rk4_v_err << std::endl;
    std::cout << std::endl;
    
    // ============================================
    // Quantitative Verification #2: Max position errors
    // ============================================
    double max_euler_x_err = 0, max_rk4_x_err = 0;
    for (size_t i = 0; i < analytical.states.size(); i++) {
        double ex = std::abs(euler_states[i].x - analytical.states[i].x);
        double rx = std::abs(rk4_states[i].x - analytical.states[i].x);
        max_euler_x_err = std::max(max_euler_x_err, ex);
        max_rk4_x_err = std::max(max_rk4_x_err, rx);
    }
    
    std::cout << "=== Verification #2: Maximum Position Errors ===" << std::endl;
    std::cout << std::scientific << std::setprecision(6);
    std::cout << "  Euler max |x error|: " << max_euler_x_err << " m" << std::endl;
    std::cout << "  RK4   max |x error|: " << max_rk4_x_err << " m" << std::endl;
    std::cout << "  RK4 improvement: " << std::fixed << std::setprecision(1) 
              << (max_euler_x_err / max_rk4_x_err) << "x smaller error" << std::endl;
    std::cout << std::endl;
    
    // ============================================
    // Quantitative Verification #3: Energy conservation / error
    // ============================================
    std::vector<double> euler_energies, rk4_energies;
    for (const auto& s : euler_states) euler_energies.push_back(energy(s));
    for (const auto& s : rk4_states) rk4_energies.push_back(energy(s));
    
    // For a damped system, energy should monotonically decrease
    // Euler may show energy increase due to numerical instability
    double euler_energy_end = euler_energies.back();
    double rk4_energy_end = rk4_energies.back();
    
    // Count how many steps Euler energy increases (should be 0 for correct damping)
    int euler_energy_increases = 0;
    for (size_t i = 1; i < euler_energies.size(); i++) {
        if (euler_energies[i] > euler_energies[i-1]) euler_energy_increases++;
    }
    
    int rk4_energy_increases = 0;
    for (size_t i = 1; i < rk4_energies.size(); i++) {
        if (rk4_energies[i] > rk4_energies[i-1]) rk4_energy_increases++;
    }
    
    // Analytical expected energy at end
    double ana_final_e = analytical.final_energy;
    
    std::cout << "=== Verification #3: Energy Conservation ===" << std::endl;
    std::cout << "  Expected final energy (analytical): " << std::fixed << std::setprecision(6) << ana_final_e << " J" << std::endl;
    std::cout << std::setw(25) << "Euler final energy: " << euler_energy_end << " J" << std::endl;
    std::cout << std::setw(25) << "RK4 final energy:   " << rk4_energy_end << " J" << std::endl;
    std::cout << std::setw(25) << "Euler energy error: " << std::abs(euler_energy_end - ana_final_e) << " J" << std::endl;
    std::cout << std::setw(25) << "RK4 energy error:   " << std::abs(rk4_energy_end - ana_final_e) << " J" << std::endl;
    std::cout << std::endl;
    std::cout << "  Energy increase violations:" << std::endl;
    std::cout << "    Euler: " << euler_energy_increases << " / " << (euler_energies.size()-1) 
              << " steps (should be 0 for stable integration)" << std::endl;
    std::cout << "    RK4:   " << rk4_energy_increases << " / " << (rk4_energies.size()-1) 
              << " steps (should be 0 for stable integration)" << std::endl;
    std::cout << std::endl;
    
    // ============================================
    // Quantitative Verification #4: Convergence rates with different dt
    // ============================================
    std::cout << "=== Verification #4: Convergence Study ===" << std::endl;
    std::cout << std::setw(10) << "dt" 
              << std::setw(20) << "Euler |x err|"
              << std::setw(20) << "RK4 |x err|"
              << std::setw(10) << "Ratio" << std::endl;
    std::cout << std::string(70, '-') << std::endl;
    
    std::vector<double> dts = {0.2, 0.1, 0.05, 0.025, 0.0125};
    std::vector<double> euler_errs, rk4_errs;
    for (double test_dt : dts) {
        auto test_euler = euler_integrate(test_dt, t_end);
        auto test_rk4 = rk4_integrate(test_dt, t_end);
        auto test_ana = analytical_solution(test_dt, t_end);
        
        double e_err = std::abs(test_euler.back().x - test_ana.states.back().x);
        double r_err = std::abs(test_rk4.back().x - test_ana.states.back().x);
        euler_errs.push_back(e_err);
        rk4_errs.push_back(r_err);
        
        std::cout << std::scientific << std::setprecision(4);
        std::cout << std::setw(10) << test_dt
                  << std::setw(20) << e_err
                  << std::setw(20) << r_err
                  << std::setw(10) << std::fixed << std::setprecision(0) 
                  << (e_err / r_err) << "x" << std::endl;
    }
    
    // Check convergence order: Euler should be O(dt), RK4 O(dt^4)
    // Halving dt: Euler error ≈ halves, RK4 error ≈ /16
    // Use the middle range where both methods are in the asymptotic regime
    double euler_order_stable = std::log2(euler_errs[2] / euler_errs[3]);
    double rk4_order_stable   = std::log2(rk4_errs[2] / rk4_errs[3]);
    std::cout << std::endl;
    std::cout << "Convergence order check (asymptotic region):" << std::endl;
    std::cout << "  Euler convergence order: " << std::fixed << std::setprecision(2) 
              << euler_order_stable << " (expected ~1.0, O(dt))" << std::endl;
    std::cout << "  RK4   convergence order: " << std::fixed << std::setprecision(2) 
              << rk4_order_stable << " (expected ~4.0, O(dt^4))" << std::endl;
    std::cout << std::endl;
    
    // ============================================
    // Verification #5: PASS/FAIL criteria
    // ============================================
    int tests_passed = 0, tests_failed = 0;
    
    std::cout << "=== Verification #5: Pass/Fail Summary ===" << std::endl;
    
    // Test 1: RK4 is more accurate than Euler
    if (rk4_x_err < euler_x_err) {
        std::cout << "  ✅ RK4 more accurate than Euler in position" << std::endl;
        tests_passed++;
    } else {
        std::cout << "  ❌ RK4 NOT more accurate than Euler!" << std::endl;
        tests_failed++;
    }
    
    // Test 2: RK4 max error is smaller
    if (max_rk4_x_err < max_euler_x_err) {
        std::cout << "  ✅ RK4 has smaller max position error" << std::endl;
        tests_passed++;
    } else {
        std::cout << "  ❌ RK4 max error NOT smaller!" << std::endl;
        tests_failed++;
    }
    
    // Test 3: RK4 energy should be closer to analytical
    if (std::abs(rk4_energy_end - ana_final_e) < std::abs(euler_energy_end - ana_final_e)) {
        std::cout << "  ✅ RK4 energy closer to analytical" << std::endl;
        tests_passed++;
    } else {
        std::cout << "  ❌ RK4 energy NOT closer!" << std::endl;
        tests_failed++;
    }
    
    // Test 4: Euler may have spurious energy increases; RK4 should not
    // (Euler at dt=0.05 may or may not; we just verify RK4 is stable)
    if (rk4_energy_increases <= 2) {  // allow 1-2 floating point issues
        std::cout << "  ✅ RK4 energy is monotonic/near-monotonic decreasing" << std::endl;
        tests_passed++;
    } else {
        std::cout << "  ❌ RK4 energy has too many increases!" << std::endl;
        tests_failed++;
    }
    
    // Test 5: RK4 convergence order should be ~4
    if (rk4_order_stable >= 3.0) {  // generous threshold
        std::cout << "  ✅ RK4 convergence order ~" << std::fixed << std::setprecision(1) 
                  << rk4_order_stable << " (expected ~4)" << std::endl;
        tests_passed++;
    } else {
        std::cout << "  ❌ RK4 convergence order too low: " << rk4_order_stable << std::endl;
        tests_failed++;
    }
    
    // Test 6: Position values are in expected range
    if (std::abs(euler_states.back().x) < 1.5 && std::abs(rk4_states.back().x) < 1.5) {
        std::cout << "  ✅ Final positions in expected range (< 1.5 m)" << std::endl;
        tests_passed++;
    } else {
        std::cout << "  ❌ Final positions out of range!" << std::endl;
        tests_failed++;
    }
    
    std::cout << std::endl;
    std::cout << "==============================================" << std::endl;
    std::cout << "  Results: " << tests_passed << " passed, " << tests_failed << " failed" << std::endl;
    std::cout << "==============================================" << std::endl;
    
    // ============================================
    // Generate visualization images
    // ============================================
    write_ppm("trajectory.ppm", euler_states, rk4_states, analytical.states, dt, dt, t_end);
    write_comparison_ppm("energy.ppm", euler_energies, rk4_energies, dt, t_end);
    
    std::cout << "\nVisualization saved: trajectory.ppm, energy.ppm" << std::endl;
    
    return tests_failed > 0 ? 1 : 0;
}
