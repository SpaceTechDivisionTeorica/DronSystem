/*
physics_UAV.h

Motor de fisico  del UAV — separado de physics_UAV.cpp (demo de consola)
y de drone_model.cpp (motor de simulacion / visualizacion OpenGL) para que
ambos consuman exactamente las mismas ecuaciones.

main source:
Ecuaciones de movimiento de un UAV — Beard & McLain Cap. 3

  BLOQUE 0 — Geometria de masa / inercia : forma -> masa, CG, tensor de inercia
  BLOQUE 1 — Translational Kinematics    : body velocity -> inertial position rates
  BLOQUE 2 — Translational Dynamics      : forces -> body velocity rates
  BLOQUE 3 — Rotational Kinematics       : body rates -> Euler angle rates
  BLOQUE 4 — Rotational Dynamics         : torques -> body angular rate derivatives

Ejes cuerpo: X adelante (nariz), Y ala derecha, Z abajo (NED-body).
*/

#ifndef PHYSICS_UAV_H
#define PHYSICS_UAV_H

#include <iostream>
#include <cmath>

struct Vec3 {
    double x, y, z;
    Vec3(double x=0, double y=0, double z=0) : x(x), y(y), z(z) {}
};

// Complete state of UAV
struct UAVState {
    double pn, pe, pd;       // inertial position NED (m)
    double u,  v,  w;        // body-frame velocity (m/s)
    double phi, theta, psi;  // Euler angles: roll, pitch, yaw (rad)
    double p,  q,  r;        // body-frame angular rates (rad/s)
};

// Inertia tensor (kg·m²) — Jxz is the cross-product term
struct Inertia {
    double Jx, Jy, Jz, Jxz;
};


// ─────────────────────────────────────────────────────────────────────────────
// BLOQUE 0 — Geometria de masa / inercia
//
// Construye masa total, CG y tensor de inercia de un dron generico ala delta
// a partir de sus componentes geometricos (fuselaje + ala + compartimento +
// motor). Esto alimenta el Bloque 4 (dinamica rotacional): la simulacion no
// usa valores de inercia inventados, los deriva de una geometria real.
// ─────────────────────────────────────────────────────────────────────────────

// Tensor de inercia completo (para construir/verificar la geometria antes de
// reducirlo a la forma {Jx,Jy,Jz,Jxz} que consumen las ecuaciones de movimiento)
struct Inertia3x3 {
    double Ixx, Iyy, Izz;
    double Ixy, Ixz, Iyz;
};

struct MassComponent {
    const char* nombre;
    double masa;         // kg
    Vec3   cg;            // posicion del CG del componente respecto al origen [m]
    Inertia3x3 I_local;   // inercia del componente respecto a SU PROPIO cg [kg*m^2]
};

// --- Inercia de un cilindro solido (fuselaje), eje a lo largo de X ---
inline Inertia3x3 inerciaCilindro(double masa, double radio, double longitud)
{
    double Ixx = 0.5 * masa * radio * radio;
    double Iyy = (1.0/12.0) * masa * (3*radio*radio + longitud*longitud);
    double Izz = Iyy;
    return {Ixx, Iyy, Izz, 0.0, 0.0, 0.0};
}

// --- Inercia de una placa triangular delta plana en el plano XY ---
// Triangulo isosceles: base = envergadura (Y), altura = cuerda de raiz (X)
inline Inertia3x3 inerciaAlaDelta(double masa, double envergadura, double cuerdaRaiz)
{
    double b = envergadura;
    double c = cuerdaRaiz;
    double Ixx = masa * (b*b) / 24.0;   // rotacion sobre eje X (envergadura)
    double Iyy = masa * (c*c) / 18.0;   // rotacion sobre eje Y (cuerda)
    double Izz = Ixx + Iyy;             // ejes perpendiculares (placa plana)
    return {Ixx, Iyy, Izz, 0.0, 0.0, 0.0};
}

// --- Inercia de una caja rectangular (compartimento delantero) ---
inline Inertia3x3 inerciaCaja(double masa, double lx, double ly, double lz)
{
    double Ixx = (1.0/12.0) * masa * (ly*ly + lz*lz);
    double Iyy = (1.0/12.0) * masa * (lx*lx + lz*lz);
    double Izz = (1.0/12.0) * masa * (lx*lx + ly*ly);
    return {Ixx, Iyy, Izz, 0.0, 0.0, 0.0};
}

// --- Masa puntual (motor trasero) ---
inline Inertia3x3 inerciaPuntual()
{
    return {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
}

// Teorema de ejes paralelos: traslada la inercia local al origen comun
inline Inertia3x3 trasladar(const Inertia3x3& I, double masa, const Vec3& r)
{
    Inertia3x3 out;
    out.Ixx = I.Ixx + masa * (r.y*r.y + r.z*r.z);
    out.Iyy = I.Iyy + masa * (r.x*r.x + r.z*r.z);
    out.Izz = I.Izz + masa * (r.x*r.x + r.y*r.y);
    out.Ixy = I.Ixy - masa * (r.x*r.y);
    out.Ixz = I.Ixz - masa * (r.x*r.z);
    out.Iyz = I.Iyz - masa * (r.y*r.z);
    return out;
}

// Propiedades de masa listas para alimentar la simulacion
struct DroneMassProperties {
    double mass;        // kg
    Vec3   cg;           // respecto al origen geometrico del fuselaje [m]
    Inertia J;           // forma reducida {Jx,Jy,Jz,Jxz} para el Bloque 4
    Inertia3x3 I_full;   // tensor completo respecto al CG, para reporte/verificacion
};

// Geometria de referencia: dron generico ala delta
//   fuselaje central (cilindro) + ala delta (placa triangular)
//   + compartimento delantero (caja) + motor trasero (masa puntual)
inline DroneMassProperties buildDeltaWingDrone()
{
    const double envergadura   = 2.5;   // m
    const double longitudTotal = 3.5;   // m
    const double cuerdaRaiz    = 0.95;  // m
    const double radioFuselaje = 0.16;  // m

    MassComponent comps[4];

    // 1) Fuselaje (cilindro), centrado en el origen
    comps[0] = { "Fuselaje",
                 90.0,
                 {0.0, 0.0, 0.0},
                 inerciaCilindro(90.0, radioFuselaje, longitudTotal) };

    // 2) Ala delta: vertice delantero a x_ala, centroide a 1/3 de la cuerda de raiz detras
    double x_ala = 0.30;
    comps[1] = { "Ala delta",
                 55.0,
                 {x_ala - cuerdaRaiz/3.0, 0.0, 0.0},
                 inerciaAlaDelta(55.0, envergadura, cuerdaRaiz) };

    // 3) Compartimento delantero (caja), en la nariz
    comps[2] = { "Compartimento delantero",
                 45.0,
                 {1.30, 0.0, 0.0},
                 inerciaCaja(45.0, 0.5, 0.28, 0.28) };

    // 4) Motor + helice (masa puntual), en la cola
    comps[3] = { "Motor trasero",
                 10.0,
                 {-1.55, 0.0, 0.0},
                 inerciaPuntual() };

    // Masa total y CG
    double masaTotal = 0.0;
    Vec3 cgAcumulado(0, 0, 0);
    for (auto& c : comps) {
        masaTotal += c.masa;
        cgAcumulado.x += c.masa * c.cg.x;
        cgAcumulado.y += c.masa * c.cg.y;
        cgAcumulado.z += c.masa * c.cg.z;
    }
    Vec3 cg(cgAcumulado.x/masaTotal, cgAcumulado.y/masaTotal, cgAcumulado.z/masaTotal);

    // Tensor de inercia total respecto al CG real (teorema de ejes paralelos)
    Inertia3x3 Itot = {0,0,0,0,0,0};
    for (auto& c : comps) {
        Vec3 r(c.cg.x - cg.x, c.cg.y - cg.y, c.cg.z - cg.z);
        Inertia3x3 Ic = trasladar(c.I_local, c.masa, r);
        Itot.Ixx += Ic.Ixx; Itot.Iyy += Ic.Iyy; Itot.Izz += Ic.Izz;
        Itot.Ixy += Ic.Ixy; Itot.Ixz += Ic.Ixz; Itot.Iyz += Ic.Iyz;
    }

    // Las ecuaciones de movimiento (Bloque 4) asumen simetria de masa respecto
    // al plano XZ (Ixy = Iyz = 0). Con esta geometria (todos los componentes
    // en Y=0) se cumple por construccion; se avisa si alguna vez deja de serlo.
    if (std::abs(Itot.Ixy) > 1e-6 || std::abs(Itot.Iyz) > 1e-6) {
        std::cerr << "[WARNING] La geometria no es simetrica respecto al plano XZ "
                     "(Ixy=" << Itot.Ixy << ", Iyz=" << Itot.Iyz << "); "
                     "rotational_dynamics() los ignora.\n";
    }

    Inertia J{ Itot.Ixx, Itot.Iyy, Itot.Izz, Itot.Ixz };

    return { masaTotal, cg, J, Itot };
}


// ─────────────────────────────────────────────────────────────────────────────
// BLOQUE 1 — Translational Kinematics
//
//   [pn_dot]   [cθcψ   sφsθcψ-cφsψ   cφsθcψ+sφsψ] [u]
//   [pe_dot] = [cθsψ   sφsθsψ+cφcψ   cφsθsψ-sφcψ] [v]
//   [pd_dot]   [-sθ    sφcθ           cφcθ        ] [w]
//
//   p_dot^v = R_b^v * v^b
// ─────────────────────────────────────────────────────────────────────────────
inline Vec3 translational_kinematics(const UAVState& s)
{
    const double cp = std::cos(s.phi),   sp = std::sin(s.phi);
    const double ct = std::cos(s.theta), st = std::sin(s.theta);
    const double cy = std::cos(s.psi),   sy = std::sin(s.psi);

    double pn_dot = (ct*cy)            * s.u
                  + (sp*st*cy - cp*sy) * s.v
                  + (cp*st*cy + sp*sy) * s.w;

    double pe_dot = (ct*sy)            * s.u
                  + (sp*st*sy + cp*cy) * s.v
                  + (cp*st*sy - sp*cy) * s.w;

    double pd_dot = (-st)              * s.u
                  + (sp*ct)            * s.v
                  + (cp*ct)            * s.w;

    return Vec3(pn_dot, pe_dot, pd_dot);
}


// ─────────────────────────────────────────────────────────────────────────────
// BLOQUE 2 — Translational Dynamics
//
//   [u_dot]   [rv - qw]   1  [fx]
//   [v_dot] = [pw - ru] + -- [fy]
//   [w_dot]   [qu - pv]   m  [fz]
// ─────────────────────────────────────────────────────────────────────────────
inline Vec3 translational_dynamics(const UAVState& s,
                                    double fx, double fy, double fz,
                                    double mass)
{
    double u_dot = (s.r * s.v - s.q * s.w) + fx / mass;
    double v_dot = (s.p * s.w - s.r * s.u) + fy / mass;
    double w_dot = (s.q * s.u - s.p * s.v) + fz / mass;

    return Vec3(u_dot, v_dot, w_dot);
}


// ─────────────────────────────────────────────────────────────────────────────
// Arrastre aerodinamico simplificado (fuerza de entrada para el Bloque 2)
//
// Beard & McLain dejan fx,fy,fz como fuerzas de entrada externas (empuje +
// gravedad + aerodinamica); aca se agrega el termino de arrastre que faltaba:
// sin el, un empuje constante acelera sin limite y soltar el acelerador no
// frena nada (1a ley de Newton). Con drag, cada posicion de palanca tiene un
// punto de equilibrio real (empuje = drag) a una velocidad de crucero fija,
// como en un avion. Forma estandar de drag: F = -k * v * |v|, por eje del
// marco cuerpo (kx,ky,kz en N/(m/s)^2, calibrados por el llamador).
// ─────────────────────────────────────────────────────────────────────────────
struct DragCoeffs { double kx, ky, kz; };

inline Vec3 aeroDrag(const UAVState& s, const DragCoeffs& k)
{
    auto term = [](double v, double kv) { return -kv * v * std::abs(v); };
    return Vec3(term(s.u, k.kx), term(s.v, k.ky), term(s.w, k.kz));
}


// ─────────────────────────────────────────────────────────────────────────────
// BLOQUE 3 — Rotational Kinematics
//
//   [phi_dot  ]   [1  sin(phi)*tan(theta)  cos(phi)*tan(theta)] [p]
//   [theta_dot] = [0  cos(phi)            -sin(phi)           ] [q]
//   [psi_dot  ]   [0  sin(phi)/cos(theta)  cos(phi)/cos(theta)] [r]
//
// WARNING: singularity at theta = ±90 deg (gimbal lock)
// ─────────────────────────────────────────────────────────────────────────────
inline Vec3 rotational_kinematics(const UAVState& s)
{
    const double sp = std::sin(s.phi), cp = std::cos(s.phi);
    const double ct = std::cos(s.theta), tt = std::tan(s.theta);

    if (std::abs(ct) < 1e-6)
        std::cerr << "[WARNING] Gimbal lock: theta near ±90 deg\n";

    const double sc = 1.0 / ct;  // sec(theta)

    double phi_dot   = s.p + sp*tt * s.q + cp*tt * s.r;
    double theta_dot =       cp    * s.q - sp    * s.r;
    double psi_dot   =       sp*sc * s.q + cp*sc * s.r;

    return Vec3(phi_dot, theta_dot, psi_dot);
}


// ─────────────────────────────────────────────────────────────────────────────
// BLOQUE 4 — Rotational Dynamics
//
//   p_dot = Γ1*p*q  - Γ2*q*r  + Γ3*l  + Γ4*n
//   q_dot = Γ5*p*r  - Γ6*(p²-r²)       + m/Jy
//   r_dot = Γ7*p*q  - Γ1*q*r  + Γ4*l  + Γ8*n
//
// Γ constants derived from the inertia tensor (see Beard & McLain eq. 3.17)
// ─────────────────────────────────────────────────────────────────────────────
inline Vec3 rotational_dynamics(const UAVState& s,
                                 double l, double m_torque, double n,
                                 const Inertia& J)
{
    const double G  = J.Jx * J.Jz - J.Jxz * J.Jxz;
    const double G1 = J.Jxz * (J.Jx - J.Jy + J.Jz) / G;
    const double G2 = (J.Jz * (J.Jz - J.Jy) + J.Jxz * J.Jxz) / G;
    const double G3 = J.Jz  / G;
    const double G4 = J.Jxz / G;
    const double G5 = (J.Jz - J.Jx) / J.Jy;
    const double G6 = J.Jxz / J.Jy;
    const double G7 = ((J.Jx - J.Jy) * J.Jx + J.Jxz * J.Jxz) / G;
    const double G8 = J.Jx / G;

    double p_dot = G1*s.p*s.q  - G2*s.q*s.r  + G3*l + G4*n;
    double q_dot = G5*s.p*s.r  - G6*(s.p*s.p - s.r*s.r)  + m_torque / J.Jy;
    double r_dot = G7*s.p*s.q  - G1*s.q*s.r  + G4*l + G8*n;

    return Vec3(p_dot, q_dot, r_dot);
}


// ─────────────────────────────────────────────────────────────────────────────
// Euler integration — advances the full 12-state UAV by one timestep dt
// (metodo de 1er orden; ver integrate_rk4 mas abajo para mayor precision)
// ─────────────────────────────────────────────────────────────────────────────
inline UAVState integrate_euler(const UAVState& s,
                                 double fx, double fy, double fz,
                                 double l,  double m_torque, double n,
                                 const Inertia& J,
                                 double mass, double dt)
{
    UAVState next = s;

    Vec3 pos_dot   = translational_kinematics(s);                // Bloque 1
    Vec3 vel_dot   = translational_dynamics(s, fx,fy,fz, mass);  // Bloque 2
    Vec3 euler_dot = rotational_kinematics(s);                   // Bloque 3
    Vec3 rate_dot  = rotational_dynamics(s, l, m_torque, n, J);  // Bloque 4

    next.pn  += pos_dot.x   * dt;
    next.pe  += pos_dot.y   * dt;
    next.pd  += pos_dot.z   * dt;

    next.u   += vel_dot.x   * dt;
    next.v   += vel_dot.y   * dt;
    next.w   += vel_dot.z   * dt;

    next.phi   += euler_dot.x * dt;
    next.theta += euler_dot.y * dt;
    next.psi   += euler_dot.z * dt;

    next.p   += rate_dot.x  * dt;
    next.q   += rate_dot.y  * dt;
    next.r   += rate_dot.z  * dt;

    return next;
}


// ─────────────────────────────────────────────────────────────────────────────
// Runge-Kutta 4 (RK4) — integrador de 4to orden para el mismo estado de 12
// componentes. Evalua los Bloques 1-4 (la derivada completa del estado) en
// 4 puntos por paso (k1..k4) y los combina con los pesos clasicos 1-2-2-1.
// Fuerzas y torques (fx,fy,fz,l,m_torque,n) se asumen constantes durante el
// paso dt — igual que en integrate_euler; solo cambia el orden de precision
// de la integracion, no el modelo de fuerzas.
// ─────────────────────────────────────────────────────────────────────────────

// Derivada completa del estado (Bloques 1-4 evaluados en un mismo instante)
struct StateDot {
    double pn, pe, pd;
    double u, v, w;
    double phi, theta, psi;
    double p, q, r;
};

inline StateDot state_derivative(const UAVState& s,
                                  double fx, double fy, double fz,
                                  double l,  double m_torque, double n,
                                  const Inertia& J, double mass)
{
    Vec3 pos_dot   = translational_kinematics(s);                // Bloque 1
    Vec3 vel_dot   = translational_dynamics(s, fx,fy,fz, mass);  // Bloque 2
    Vec3 euler_dot = rotational_kinematics(s);                   // Bloque 3
    Vec3 rate_dot  = rotational_dynamics(s, l, m_torque, n, J);  // Bloque 4

    return { pos_dot.x, pos_dot.y, pos_dot.z,
             vel_dot.x, vel_dot.y, vel_dot.z,
             euler_dot.x, euler_dot.y, euler_dot.z,
             rate_dot.x, rate_dot.y, rate_dot.z };
}

// Estado + h * derivada — construye los estados intermedios s2,s3,s4 de RK4
inline UAVState state_add_scaled(const UAVState& s, const StateDot& d, double h)
{
    UAVState out = s;
    out.pn    += h * d.pn;
    out.pe    += h * d.pe;
    out.pd    += h * d.pd;
    out.u     += h * d.u;
    out.v     += h * d.v;
    out.w     += h * d.w;
    out.phi   += h * d.phi;
    out.theta += h * d.theta;
    out.psi   += h * d.psi;
    out.p     += h * d.p;
    out.q     += h * d.q;
    out.r     += h * d.r;
    return out;
}

// Combina k1..k4 con los pesos clasicos de RK4 (1-2-2-1) sobre el estado base.
// Compartido con simulateStep() (simulate_step.h), que evalua las mismas
// 4 etapas pero con dinamica rotacional de tensor de inercia completo.
inline UAVState rk4_combine(const UAVState& s,
                             const StateDot& k1, const StateDot& k2,
                             const StateDot& k3, const StateDot& k4,
                             double dt)
{
    UAVState next = s;
    next.pn    += (dt / 6.0) * (k1.pn    + 2.0*k2.pn    + 2.0*k3.pn    + k4.pn);
    next.pe    += (dt / 6.0) * (k1.pe    + 2.0*k2.pe    + 2.0*k3.pe    + k4.pe);
    next.pd    += (dt / 6.0) * (k1.pd    + 2.0*k2.pd    + 2.0*k3.pd    + k4.pd);
    next.u     += (dt / 6.0) * (k1.u     + 2.0*k2.u     + 2.0*k3.u     + k4.u);
    next.v     += (dt / 6.0) * (k1.v     + 2.0*k2.v     + 2.0*k3.v     + k4.v);
    next.w     += (dt / 6.0) * (k1.w     + 2.0*k2.w     + 2.0*k3.w     + k4.w);
    next.phi   += (dt / 6.0) * (k1.phi   + 2.0*k2.phi   + 2.0*k3.phi   + k4.phi);
    next.theta += (dt / 6.0) * (k1.theta + 2.0*k2.theta + 2.0*k3.theta + k4.theta);
    next.psi   += (dt / 6.0) * (k1.psi   + 2.0*k2.psi   + 2.0*k3.psi   + k4.psi);
    next.p     += (dt / 6.0) * (k1.p     + 2.0*k2.p     + 2.0*k3.p     + k4.p);
    next.q     += (dt / 6.0) * (k1.q     + 2.0*k2.q     + 2.0*k3.q     + k4.q);
    next.r     += (dt / 6.0) * (k1.r     + 2.0*k2.r     + 2.0*k3.r     + k4.r);
    return next;
}

inline UAVState integrate_rk4(const UAVState& s,
                               double fx, double fy, double fz,
                               double l,  double m_torque, double n,
                               const Inertia& J,
                               double mass, double dt)
{
    StateDot k1 = state_derivative(s, fx,fy,fz, l,m_torque,n, J, mass);
    UAVState s2 = state_add_scaled(s, k1, dt / 2.0);

    StateDot k2 = state_derivative(s2, fx,fy,fz, l,m_torque,n, J, mass);
    UAVState s3 = state_add_scaled(s, k2, dt / 2.0);

    StateDot k3 = state_derivative(s3, fx,fy,fz, l,m_torque,n, J, mass);
    UAVState s4 = state_add_scaled(s, k3, dt);

    StateDot k4 = state_derivative(s4, fx,fy,fz, l,m_torque,n, J, mass);

    return rk4_combine(s, k1, k2, k3, k4, dt);
}

// ─────────────────────────────────────────────────────────────────────────────
// Conversiones de actitud: Euler <-> Cuaternion <-> Matriz de rotacion
//
// Cuaternion unitario e_b^i = (e0,e1,e2,e3): rotacion de cuerpo a inercial.
// e0 es la parte escalar, (e1,e2,e3) la parte vectorial.
//
// Formulas: euler_to_quaternions.jpeg y quaternion_to_rotationMatrix.jpeg
// Implementadas (no inline) en physics_UAV.cpp — son utilidades de conversion
// de actitud, separadas del lazo de integracion (integrate_euler), que sigue
// trabajando en angulos de Euler.
// ─────────────────────────────────────────────────────────────────────────────
struct Quaternion {
    double e0, e1, e2, e3;
};

struct Mat3x3 {
    double m[3][3];
};

Quaternion eulerToQuaternion(double phi, double theta, double psi);
Vec3       quaternionToEuler(const Quaternion& e);          // devuelve (phi, theta, psi)
Mat3x3     quaternionToRotationMatrix(const Quaternion& e); // R_b^i: cuerpo -> inercial


// ─────────────────────────────────────────────────────────────────────────────
// Piso — restriccion de contacto en el plano X-Y (colision con el suelo)
//
// El suelo es el plano horizontal del marco inercial NED, ubicado en
// pd = pisoPD (por defecto 0.0 = nivel de referencia). Como pd es positivo
// hacia abajo, cruzar pd > pisoPD significa que el dron atraveso el piso.
// ─────────────────────────────────────────────────────────────────────────────
struct GroundContact {
    bool   colision;
    double penetracion;   // cuanto se paso del piso (m); 0 si no hay colision
};

inline GroundContact checkGroundCollision(const UAVState& s, double pisoPD = 0.0)
{
    double pen = s.pd - pisoPD;
    return { pen > 0.0, pen > 0.0 ? pen : 0.0 };
}

// Respuesta inelastica minima: clava el dron al piso y anula su velocidad
// (frena en seco — no hay rebote ni deformacion, es una restriccion dura)
inline UAVState resolveGroundCollision(UAVState s, double pisoPD = 0.0)
{
    if (s.pd > pisoPD) {
        s.pd = pisoPD;
        s.u = s.v = s.w = 0.0;
    }
    return s;
}


// ─────────────────────────────────────────────────────────────────────────────
// Obstaculo — bloque rectangular fijo, colision AABB (caja alineada a ejes)
//
// Definido por su centro en el plano horizontal (pnCenter, peCenter) y sus
// dimensiones reales: ancho en pn [m], profundidad en pe [m] y altura desde
// el piso [m]. Igual que el piso, es una restriccion dura, sin deformacion
// ni rebote; el dron se trata como un punto (su CG) para la colision, misma
// simplificacion que usa checkGroundCollision().
// ─────────────────────────────────────────────────────────────────────────────
struct BoxObstacle {
    double pnCenter, peCenter;  // centro del bloque en el plano horizontal [m]
    double sizePn, sizePe;      // dimensiones del bloque en pn (X) y pe (Y) [m]
    double height;               // altura del bloque desde el piso [m]
    double pisoPD;                // pd del piso sobre el que se apoya el bloque
};

struct ObstacleContact {
    bool colision;
};

inline ObstacleContact checkObstacleCollision(const UAVState& s, const BoxObstacle& box)
{
    const double halfPn = box.sizePn / 2.0;
    const double halfPe = box.sizePe / 2.0;

    const bool dentroPn  = std::abs(s.pn - box.pnCenter) <= halfPn;
    const bool dentroPe  = std::abs(s.pe - box.peCenter) <= halfPe;
    const bool dentroAlt = (s.pd <= box.pisoPD) && (s.pd >= box.pisoPD - box.height);

    return { dentroPn && dentroPe && dentroAlt };
}

// Respuesta inelastica: empuja al dron fuera de la caja por el eje de menor
// penetracion (MTV — minimum translation vector) y anula su velocidad,
// misma filosofia "solido, sin rebote" que resolveGroundCollision pero en
// los 3 ejes, para que el bloque sea realmente solido y no se pueda
// atravesar sosteniendo el acelerador.
inline UAVState resolveObstacleCollision(UAVState s, const BoxObstacle& box)
{
    const double halfPn = box.sizePn / 2.0;
    const double halfPe = box.sizePe / 2.0;
    const double top    = box.pisoPD - box.height;   // pd del techo (arriba -> pd menor)
    const double bottom = box.pisoPD;                // pd del piso del bloque

    const double overlapPn  = halfPn - std::abs(s.pn - box.pnCenter);
    const double overlapPe  = halfPe - std::abs(s.pe - box.peCenter);
    const double overlapAlt = (bottom - s.pd < s.pd - top) ? (bottom - s.pd) : (s.pd - top);

    if (overlapPn <= overlapPe && overlapPn <= overlapAlt) {
        s.pn = (s.pn >= box.pnCenter) ? box.pnCenter + halfPn : box.pnCenter - halfPn;
    } else if (overlapPe <= overlapAlt) {
        s.pe = (s.pe >= box.peCenter) ? box.peCenter + halfPe : box.peCenter - halfPe;
    } else {
        s.pd = (s.pd <= (top + bottom) / 2.0) ? top : bottom;
    }

    s.u = s.v = s.w = 0.0;
    return s;
}

#endif // PHYSICS_UAV_H
