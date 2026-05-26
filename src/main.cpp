#include <mu/Application.h>
#include <dense/Matrix.h>
#include <iostream>
#include <cmath>
#include "MpcLayout.h"
#include "MpcEngine.h"

int main(int argc, const char* argv[]) {
    mu::Application app(argc, argv);

    std::cout << "MpcCore start" << std::endl;

    mpc::MpcLayout layout(20, 0.1, 0.5);
    mpc::MpcEngine engine(layout);

    dense::DblMatrix coeffs(4, 1, nullptr, true);
    auto c = coeffs.getColumnManipulator();
    c(0) = 0.0;
    c(1) = 0.0;
    c(2) = 0.0;
    c(3) = 0.0;

    mpc::Telemetry tel;
    tel.x = 0.0;
    tel.y = 0.0;
    tel.psi = 0.0;
    tel.v = 1.0;

    mpc::MpcSqp::Settings cfg;
    cfg.maxIter = 5;
    cfg.tol = 1e-3;
    cfg.alpha = 1.0;
    cfg.verbose = false;
    engine.setSettings(cfg);

    const int steps = 40; // 30-60 steps
    for (int step = 0; step < steps; ++step) {
        mpc::Trajectory traj;
        if (!engine.Solve(tel, coeffs, 1.0, layout.dt(), traj)) {
            std::cout << "Engine solve failed at step " << step << std::endl;
            break;
        }

        if (traj.N < 2) {
            std::cout << "Trajectory too short at step " << step << std::endl;
            break;
        }

        auto u = traj.controls.getManipulator();
        const double delta = u(0, 0);
        const double a = u(1, 0);

        const double dt = layout.dt();
        const double Lf = layout.Lf();
        const double x = tel.x;
        const double y = tel.y;
        const double psi = tel.psi;
        const double v = tel.v;

        tel.x = x + v * std::cos(psi) * dt;
        tel.y = y + v * std::sin(psi) * dt;
        tel.psi = psi + (v / Lf) * delta * dt;
        tel.v = v + a * dt;

        auto s = traj.states.getManipulator();
        const double ref_x = s(0, 0);
        const double ref_y = s(1, 0);
        const double dx = tel.x - ref_x;
        const double dy = tel.y - ref_y;
        const double err = std::sqrt(dx * dx + dy * dy);

        std::cout << "step=" << step << " tracking_err=" << err << std::endl;
    }

    std::cout << "MpcCore end" << std::endl;
    return 0;
}
