/*
physics_UAV.cpp

Demo de consola de la fisica real del UAV. Las ecuaciones de movimiento
(Beard & McLain Cap. 3) y la geometria de masa/inercia viven en
physics_UAV.h, para que drone_model.cpp (el motor de visualizacion) consuma
exactamente el mismo modelo.
*/

#include <iostream>
#include <iomanip>
#include "physics_UAV.h"
#include "simulate_step.h"

void printMassProperties(const DroneMassProperties& mp)
{
    std::cout << std::fixed << std::setprecision(3);
    std::cout << "=== Geometria de masa / inercia (dron ala delta) ===\n";
    std::cout << "Masa total : " << mp.mass << " kg\n";
    std::cout << "CG (x,y,z) : (" << mp.cg.x << ", " << mp.cg.y << ", " << mp.cg.z
              << ") m  [respecto al origen del fuselaje]\n";
    std::cout << "Tensor de inercia respecto al CG [kg*m^2]:\n";
    std::cout << "  Ixx=" << mp.I_full.Ixx << "  Ixy=" << mp.I_full.Ixy << "  Ixz=" << mp.I_full.Ixz << "\n";
    std::cout << "  Iyx=" << mp.I_full.Ixy << "  Iyy=" << mp.I_full.Iyy << "  Iyz=" << mp.I_full.Iyz << "\n";
    std::cout << "  Izx=" << mp.I_full.Ixz << "  Izy=" << mp.I_full.Iyz << "  Izz=" << mp.I_full.Izz << "\n\n";
}


// ─────────────────────────────────────────────────────────────────────────────
// Conversiones de actitud: Euler <-> Cuaternion <-> Matriz de rotacion
// (declaradas en physics_UAV.h; formulas en euler_to_quaternions.jpeg y
// quaternion_to_rotationMatrix.jpeg)
// ─────────────────────────────────────────────────────────────────────────────

// Angulos de Euler (phi=roll, theta=pitch, psi=yaw) -> cuaternion e_b^i
Quaternion eulerToQuaternion(double phi, double theta, double psi)
{
    const double cp2 = std::cos(phi   / 2.0), sp2 = std::sin(phi   / 2.0);
    const double ct2 = std::cos(theta / 2.0), st2 = std::sin(theta / 2.0);
    const double cy2 = std::cos(psi   / 2.0), sy2 = std::sin(psi   / 2.0);

    Quaternion e;
    e.e0 = cy2*ct2*cp2 + sy2*st2*sp2;
    e.e1 = cy2*ct2*sp2 - sy2*st2*cp2;
    e.e2 = cy2*st2*cp2 + sy2*ct2*sp2;
    e.e3 = sy2*ct2*cp2 - cy2*st2*sp2;
    return e;
}

// Cuaternion e_b^i -> angulos de Euler, devueltos como Vec3(phi, theta, psi)
Vec3 quaternionToEuler(const Quaternion& e)
{
    const double e0 = e.e0, e1 = e.e1, e2 = e.e2, e3 = e.e3;

    double phi   = std::atan2(2.0*(e0*e1 + e2*e3), e0*e0 + e3*e3 - e1*e1 - e2*e2);
    double theta = std::asin (2.0*(e0*e2 - e1*e3));
    double psi   = std::atan2(2.0*(e0*e3 + e1*e2), e0*e0 + e1*e1 - e2*e2 - e3*e3);

    return Vec3(phi, theta, psi);
}

// Cuaternion e_b^i -> matriz de rotacion R_b^i (cuerpo -> inercial)
Mat3x3 quaternionToRotationMatrix(const Quaternion& e)
{
    const double e0 = e.e0, e1 = e.e1, e2 = e.e2, e3 = e.e3;

    Mat3x3 R;
    R.m[0][0] = e1*e1 + e0*e0 - e2*e2 - e3*e3;
    R.m[0][1] = 2.0*(e1*e2 - e3*e0);
    R.m[0][2] = 2.0*(e1*e3 + e2*e0);

    R.m[1][0] = 2.0*(e1*e2 + e3*e0);
    R.m[1][1] = e2*e2 + e0*e0 - e1*e1 - e3*e3;
    R.m[1][2] = 2.0*(e2*e3 - e1*e0);

    R.m[2][0] = 2.0*(e1*e3 - e2*e0);
    R.m[2][1] = 2.0*(e2*e3 + e1*e0);
    R.m[2][2] = e3*e3 + e0*e0 - e1*e1 - e2*e2;

    return R;
}

void printQuaternion(const Quaternion& e)
{
    std::cout << "Cuaternion (e0,e1,e2,e3) = ("
              << e.e0 << ", " << e.e1 << ", " << e.e2 << ", " << e.e3 << ")\n";
}

void printRotationMatrix(const Mat3x3& R)
{
    std::cout << "Matriz de rotacion R_b^i:\n";
    for (int i = 0; i < 3; ++i) {
        std::cout << "  [" << R.m[i][0] << "  " << R.m[i][1] << "  " << R.m[i][2] << "]\n";
    }
}


// ─────────────────────────────────────────────────────────────────────────────
// Main — simulation example
// ─────────────────────────────────────────────────────────────────────────────
int main()
{
    const double g = 9.81;

    // Masa, CG e inercia derivados de la geometria real del dron ala delta
    // (Bloque 0) — no son valores inventados, son consecuencia de la forma.
    const DroneMassProperties drone = buildDeltaWingDrone();
    printMassProperties(drone);

    const double mass = drone.mass;

    UAVState state{};
    state.pn    =  0.0;
    state.pe    =  0.0;
    state.pd    = -10.0;   // 10 m altitude (NED: negative = up)
    state.u     =  5.0;    // m/s forward
    state.phi   =  0.1;    // ~5.7 deg roll
    state.theta =  0.05;   // ~2.9 deg pitch
    state.p     =  0.01;   // rad/s roll rate

    // Demo de conversion de actitud: Euler -> Cuaternion -> Matriz de rotacion
    std::cout << "=== Conversion de actitud (attitude en t=0) ===\n";
    Quaternion e0 = eulerToQuaternion(state.phi, state.theta, state.psi);
    printQuaternion(e0);

    Vec3 eulerBack = quaternionToEuler(e0);
    std::cout << "Euler recuperado (phi,theta,psi) = ("
              << eulerBack.x * 180.0 / M_PI << ", "
              << eulerBack.y * 180.0 / M_PI << ", "
              << eulerBack.z * 180.0 / M_PI << ") deg\n";

    Mat3x3 R = quaternionToRotationMatrix(e0);
    printRotationMatrix(R);
    std::cout << "\n";

    const double fx_thrust =  0.5;
    const double fy_thrust =  0.0;
    const double fz_thrust = -mass * g;  // full-weight upward thrust

    // Zero torques — pure free-flight / gravity scenario
    const double l_torque = 0.0;
    const double m_torque = 0.0;
    const double n_torque = 0.0;

    const double dt    = 0.01;
    const int    steps = 500;   // 5 seconds

    const double PISO_PD = 0.0;   // suelo solido en el plano X-Y (pd=0, NED)
    bool crashed = false;

    const int W = 12;
    std::cout << std::fixed << std::setprecision(4);
    std::cout << std::setw(W) << "t(s)"
              << std::setw(W) << "pn(m)"
              << std::setw(W) << "pe(m)"
              << std::setw(W) << "pd(m)"
              << std::setw(W) << "u(m/s)"
              << std::setw(W) << "phi(deg)"
              << std::setw(W) << "theta(deg)"
              << std::setw(W) << "p(deg/s)"
              << "\n";
    std::cout << std::string(W * 8, '-') << "\n";

    for (int i = 0; i <= steps; ++i) {
        if (i % 50 == 0) {
            std::cout << std::setw(W) << i * dt
                      << std::setw(W) << state.pn
                      << std::setw(W) << state.pe
                      << std::setw(W) << state.pd
                      << std::setw(W) << state.u
                      << std::setw(W) << state.phi   * 180.0 / M_PI
                      << std::setw(W) << state.theta * 180.0 / M_PI
                      << std::setw(W) << state.p     * 180.0 / M_PI
                      << "\n";
        }

        // Gravity projected into body frame — updates each step as attitude changes
        double fg_x = -mass * g * std::sin(state.theta);
        double fg_y =  mass * g * std::cos(state.theta) * std::sin(state.phi);
        double fg_z =  mass * g * std::cos(state.theta) * std::cos(state.phi);

        // simulateStep() (simulate_step.h): mismo RK4, pero con el tensor de
        // inercia COMPLETO (drone.I_full: momentos + productos de inercia),
        // en vez de la forma reducida {Jx,Jy,Jz,Jxz} que usa integrate_rk4().
        state = simulateStep(state,
                              fx_thrust + fg_x,
                              fy_thrust + fg_y,
                              fz_thrust + fg_z,
                              l_torque, m_torque, n_torque,
                              drone.I_full, mass, dt);

        GroundContact gc = checkGroundCollision(state, PISO_PD);
        if (gc.colision) {
            state = resolveGroundCollision(state, PISO_PD);
            if (!crashed)
                std::cout << "\n[COLISION] impacto contra el piso en t=" << (i+1)*dt
                          << " s  (pn=" << state.pn << ", pe=" << state.pe << ")\n\n";
            crashed = true;
        }
    }

    return 0;
}
