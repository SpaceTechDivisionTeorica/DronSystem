/**
 * drone_model.cpp
 *
 * 3D visualization — motor de vuelo completo (6DOF)
 *
 * Usa el mismo modelo fisico que physics_UAV.cpp (physics_UAV.h: Bloques
 * 0-4 de Beard & McLain Cap. 3) con la masa/inercia real del dron ala
 * delta (buildDeltaWingDrone()). El dron se dibuja centrado en pantalla;
 * el mundo (ejes, horizonte, grilla de suelo) se traslada en sentido
 * opuesto a su posicion inercial para que el vuelo se vea (camara
 * "chase" centrada en el dron).
 *
 * Controles:
 *   <- ->  arrows    p  (roll rate  ±0.02 rad/s, directo, Bloque 3)
 *   up dn  arrows    q  (pitch rate ±0.02 rad/s, directo, Bloque 3)
 *   A / D            r  (yaw rate   ±0.02 rad/s, directo, Bloque 3)
 *   w  (mantener)    palanca de gases + (throttle up,  Bloques 1-2)
 *   s  (mantener)    palanca de gases - (throttle down / reversa, Bloques 1-2)
 *   Space            pause / resume
 *   0                reset estado completo (posicion, velocidad, actitud, palanca)
 *   R                reset camara
 *   Shift+W          wireframe toggle
 *   + / -            zoom
 *   Q / Esc          quit
 */

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wdeprecated-declarations"
#include <GLUT/glut.h>
#pragma clang diagnostic pop

#include <OpenGL/glu.h>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "physics_UAV.h"

// ═══════════════════════════════════════════════════════════════════════
// FISICA — masa/inercia real (Bloque 0) + dinamica 6DOF (Bloques 1-4)
// ═══════════════════════════════════════════════════════════════════════

static const double GRAVITY = 9.81;
static const double PISO_PD = 0.0;   // suelo solido en el plano X-Y (pd=0, NED)

// ─────────────────────────────────────────────────────────────────────────
// PALANCA DE GASES (throttle) — W/S mueven una posicion de palanca 0..1
// (negativa hasta THROTTLE_MIN = reversa/freno), no una fuerza directa.
// Mientras se mantiene la tecla, rampa a THROTTLE_RATE fraccion/s (no salta
// por evento, para no depender del auto-repeat del teclado); se sostiene al
// soltar, como el acelerador de un avion. fx_thrust = throttle * FX_MAX.
//
// Por si sola la palanca no basta para "sentirse" como un avion: hace falta
// arrastre aerodinamico (mas abajo, K_DRAG_*) para que cada posicion tenga
// un punto de equilibrio real (empuje = drag) en vez de acelerar sin limite.
// ─────────────────────────────────────────────────────────────────────────
static const double THROTTLE_RATE =    0.4;          // fraccion/s mientras se mantiene w/s
static const double FX_MAX        = 4000.0;          // N — empuje maximo (throttle=1.0)
static const double FX_MIN        = -1500.0;         // N — empuje minimo (throttle negativo, reversa/freno)
static const double THROTTLE_MIN  = FX_MIN / FX_MAX;  // ~ -0.375

// Arrastre aerodinamico (Bloque 2, aeroDrag() en physics_UAV.h) — cuadratico
// por eje del marco cuerpo. K_DRAG_U se calibra para que a throttle=1.0 el
// dron se estabilice en V_CRUISE_MAX m/s de crucero longitudinal; K_DRAG_V/W
// son mayores porque el area expuesta de costado / arriba-abajo (fuselaje +
// ala) es mucho mayor que la frontal (solo la nariz) — tambien frena la
// deriva lateral/vertical que antes quedaba sin oponerse a nada.
static const double V_CRUISE_MAX = 28.0;                              // m/s @ throttle=1.0
static const double K_DRAG_U     = FX_MAX / (V_CRUISE_MAX*V_CRUISE_MAX);  // ~5.10 N/(m/s)^2
static const double K_DRAG_V     = K_DRAG_U * 4.0;
static const double K_DRAG_W     = K_DRAG_U * 4.0;
static const DragCoeffs g_dragCoeffs{ K_DRAG_U, K_DRAG_V, K_DRAG_W };

static const double U_INIT = 5.0;   // m/s crucero inicial (coincide con g_state.u en main())
// Throttle que sostiene U_INIT en equilibrio (empuje = drag a esa velocidad)
static const double THROTTLE_INIT = (K_DRAG_U * U_INIT * U_INIT) / FX_MAX;

// Obstaculo — bloque rectangular fijo (colision AABB, checkObstacleCollision()
// en physics_UAV.h), dimensiones fisicas reales en metros. Centrado 100 m
// adelante del origen (pn) y sobre el eje de crucero inicial (pe=0), para que
// se pueda ir a chocar contra el descendiendo mientras se vuela hacia adelante.
static const BoxObstacle g_obstacle{
    /*pnCenter*/ 100.0, /*peCenter*/ 0.0,
    /*sizePn*/     2.0, /*sizePe*/   3.0,
    /*height*/     6.0, /*pisoPD*/   PISO_PD
};
static bool g_obstacleHit = false;

static DroneMassProperties g_drone;   // masa, cg, tensor de inercia (Bloque 0)
static UAVState g_state;
static UAVState g_init;
static bool     g_crashed  = false;
static double   g_throttle = THROTTLE_INIT;   // posicion de palanca [THROTTLE_MIN, 1.0]
static double   g_fxThrust = 0.0;             // N — empuje resultante (throttle * FX_MAX), para HUD
static bool     g_keyW = false, g_keyS = false;

// ═══════════════════════════════════════════════════════════════════════
// TELEMETRY — historial de estado para la ventana secundaria de graficas
// (buffer circular; step() empuja una muestra por cada paso de fisica)
// ═══════════════════════════════════════════════════════════════════════

struct TelemetrySample {
    double t;
    double pn, pe, pd;
    double u, v, w;
    double phi, theta, psi;
    double p, q, r;
};

static const int TELEMETRY_CAPACITY = 3000;   // ~48 s de historial @ 60fps
static TelemetrySample g_telemetry[TELEMETRY_CAPACITY];
static int    g_telemetryHead  = 0;   // proximo indice a escribir
static int    g_telemetryCount = 0;   // muestras validas (satura en CAPACITY)
static double g_simTime        = 0.0;

static void pushTelemetry()
{
    TelemetrySample smp;
    smp.t     = g_simTime;
    smp.pn    = g_state.pn;    smp.pe    = g_state.pe;    smp.pd  = g_state.pd;
    smp.u     = g_state.u;     smp.v     = g_state.v;     smp.w   = g_state.w;
    smp.phi   = g_state.phi;   smp.theta = g_state.theta; smp.psi = g_state.psi;
    smp.p     = g_state.p;     smp.q     = g_state.q;     smp.r   = g_state.r;

    g_telemetry[g_telemetryHead] = smp;
    g_telemetryHead = (g_telemetryHead + 1) % TELEMETRY_CAPACITY;
    if (g_telemetryCount < TELEMETRY_CAPACITY) g_telemetryCount++;
}

static void resetTelemetry()
{
    g_telemetryHead = 0; g_telemetryCount = 0; g_simTime = 0.0;
}

// Muestra i-esima en orden cronologico (0 = la mas vieja aun en el buffer)
static const TelemetrySample& telemetryAt(int i)
{
    int start = (g_telemetryHead - g_telemetryCount + TELEMETRY_CAPACITY) % TELEMETRY_CAPACITY;
    return g_telemetry[(start + i) % TELEMETRY_CAPACITY];
}

// Bloque 3 (igual que antes): el usuario comanda p,q,r directamente con el
// teclado y se integran las tasas de Euler — control directo, sin pasar por
// torques/Bloque 4.
//
// Bloque 1-2 (cadena completa dinamica+cinematica): W/S mueven la palanca de
// gases g_throttle -> g_fxThrust; aeroDrag() (physics_UAV.h) le opone
// arrastre a la velocidad en los 3 ejes del marco cuerpo, dandole a cada
// posicion de palanca un punto de equilibrio real (empuje=drag) en vez de
// acelerar sin limite. translational_dynamics() convierte la fuerza neta en
// aceleracion de u,v,w (Bloque 2) y translational_kinematics() esa velocidad
// en desplazamiento inercial (Bloque 1) — a diferencia de la rotacion, aca
// si se recorre dinamica -> cinematica completa.
static void step(double dt)
{
    if (g_keyW) g_throttle += THROTTLE_RATE * dt;
    if (g_keyS) g_throttle -= THROTTLE_RATE * dt;
    if (g_throttle > 1.0)          g_throttle = 1.0;
    if (g_throttle < THROTTLE_MIN) g_throttle = THROTTLE_MIN;
    g_fxThrust = g_throttle * FX_MAX;

    const double fy_thrust =  0.0;
    const double fz_thrust = -g_drone.mass * GRAVITY;   // empuje vertical = peso -> altitud estable

    double fg_x = -g_drone.mass * GRAVITY * std::sin(g_state.theta);
    double fg_y =  g_drone.mass * GRAVITY * std::cos(g_state.theta) * std::sin(g_state.phi);
    double fg_z =  g_drone.mass * GRAVITY * std::cos(g_state.theta) * std::cos(g_state.phi);

    Vec3 drag = aeroDrag(g_state, g_dragCoeffs);   // Bloque 2 — arrastre por eje

    Vec3 pos_dot = translational_kinematics(g_state);                              // Bloque 1
    Vec3 vel_dot = translational_dynamics(g_state, g_fxThrust + fg_x + drag.x,
                                           fy_thrust + fg_y + drag.y,
                                           fz_thrust + fg_z + drag.z,
                                           g_drone.mass);                           // Bloque 2
    Vec3 euler_dot = rotational_kinematics(g_state);                               // Bloque 3

    g_state.pn += pos_dot.x * dt;
    g_state.pe += pos_dot.y * dt;
    g_state.pd += pos_dot.z * dt;

    g_state.u += vel_dot.x * dt;
    g_state.v += vel_dot.y * dt;
    g_state.w += vel_dot.z * dt;

    g_state.phi   += euler_dot.x * dt;
    g_state.theta += euler_dot.y * dt;
    g_state.psi   += euler_dot.z * dt;

    // p,q,r NO se integran via Bloque 4 — son el input directo del usuario

    GroundContact gc = checkGroundCollision(g_state, PISO_PD);   // piso solido
    if (gc.colision) {
        g_state  = resolveGroundCollision(g_state, PISO_PD);
        g_crashed = true;
    }

    ObstacleContact oc = checkObstacleCollision(g_state, g_obstacle);   // bloque fijo
    if (oc.colision) {
        g_state       = resolveObstacleCollision(g_state, g_obstacle);
        g_crashed     = true;
        g_obstacleHit = true;
    }

    g_simTime += dt;
    pushTelemetry();
}

// ═══════════════════════════════════════════════════════════════════════
// SIMULATION STATE
// ═══════════════════════════════════════════════════════════════════════

static const double DT = 0.016;   // ~60 fps integration step

static bool     g_paused = false;

// ═══════════════════════════════════════════════════════════════════════
// CAMERA
// ═══════════════════════════════════════════════════════════════════════

static float g_rotX =  20.0f;
static float g_rotY = -35.0f;
static float g_zoom = -22.0f;
static int   g_lastX, g_lastY;
static bool  g_drag = false;
static bool  g_wire = false;

// ═══════════════════════════════════════════════════════════════════════
// HUD
// ═══════════════════════════════════════════════════════════════════════

static void hudText(float x, float y, const char* s)
{
    glMatrixMode(GL_PROJECTION); glPushMatrix(); glLoadIdentity();
    glOrtho(0, 900, 0, 600, -1, 1);
    glMatrixMode(GL_MODELVIEW);  glPushMatrix(); glLoadIdentity();
    glRasterPos2f(x, y);
    while (*s) glutBitmapCharacter(GLUT_BITMAP_8_BY_13, *s++);
    glMatrixMode(GL_PROJECTION); glPopMatrix();
    glMatrixMode(GL_MODELVIEW);  glPopMatrix();
}

static void drawHUD()
{
    const double r2d = 180.0 / M_PI;
    char buf[128];

    glDisable(GL_LIGHTING);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

    // Title
    glColor3f(0.90f, 0.90f, 0.90f);
    hudText(10, 585, "UAV  —  Vuelo 6DOF (fisica real: masa/inercia ala delta)");

    if (g_paused) {
        glColor3f(1.0f, 0.80f, 0.10f);
        hudText(430, 585, "[PAUSED]");
    }

    // Posicion e altitud (Bloque 1)
    glColor3f(0.55f, 0.80f, 1.0f);
    if (g_crashed) {
        hudText(10, 560, "pn=--.-  pe=--.-  alt=--.-  m   [SIN SEÑAL]");
    } else {
        snprintf(buf, sizeof(buf), "pn=%.1f  pe=%.1f  alt=%.1f  m",
                 g_state.pn, g_state.pe, -g_state.pd);
        hudText(10, 560, buf);
    }

    // Velocidad (Bloque 2)
    double airspeed = std::sqrt(g_state.u*g_state.u + g_state.v*g_state.v + g_state.w*g_state.w);
    double vEq = (g_fxThrust > 0.0) ? std::sqrt(g_fxThrust / K_DRAG_U) : 0.0;
    glColor3f(0.70f, 0.90f, 0.60f);
    if (g_crashed) {
        hudText(10, 540, "u=--.-  v=--.-  w=--.-  |V|=--.-  m/s   |  palanca=---%  [SIN SEÑAL]");
    } else {
        snprintf(buf, sizeof(buf), "u=%.2f  v=%.2f  w=%.2f  |V|=%.2f  m/s   |  palanca=%.0f%%  (%.0f N -> Veq=%.1f m/s)",
                 g_state.u, g_state.v, g_state.w, airspeed, g_throttle*100.0, g_fxThrust, vEq);
        hudText(10, 540, buf);
    }

    // Euler angles (Bloque 3)
    glColor3f(0.50f, 1.0f, 0.55f);
    if (g_crashed) {
        hudText(10, 520, "phi=--.-  theta=--.-  psi=--.-  deg   [SIN SEÑAL]");
    } else {
        snprintf(buf, sizeof(buf), "phi=%.2f  theta=%.2f  psi=%.2f  deg",
                 g_state.phi*r2d, g_state.theta*r2d, g_state.psi*r2d);
        hudText(10, 520, buf);
    }

    // Angular rates — input directo del usuario (Bloque 3)
    glColor3f(1.0f, 0.75f, 0.35f);
    snprintf(buf, sizeof(buf), "p=%.3f  q=%.3f  r=%.3f  rad/s",
             g_state.p, g_state.q, g_state.r);
    hudText(10, 500, buf);

    // Masa / inercia (Bloque 0)
    glColor3f(0.65f, 0.65f, 0.70f);
    snprintf(buf, sizeof(buf), "masa=%.0f kg   Ixx=%.0f  Iyy=%.0f  Izz=%.0f  kg*m^2",
             g_drone.mass, g_drone.I_full.Ixx, g_drone.I_full.Iyy, g_drone.I_full.Izz);
    hudText(10, 480, buf);

    // Colision con el piso solido o con el obstaculo (bloque fijo)
    if (g_crashed) {
        glColor3f(1.0f, 0.25f, 0.20f);
        hudText(10, 460, "SEÑAL PERDIDA — INTERFERENCIA. Presiona 0 para reiniciar.");
    }

    // Singularity warning
    if (std::abs(std::cos(g_state.theta)) < 0.1) {
        glColor3f(1.0f, 0.2f, 0.2f);
        hudText(10, 440, "WARNING: Gimbal Lock — theta near ±90°");
    }

    // Axis legend
    glColor3f(1.0f, 0.35f, 0.35f); hudText(10,  30, "X_b");
    glColor3f(0.35f, 1.0f, 0.35f); hudText(45,  30, "Y_b");
    glColor3f(0.45f, 0.60f, 1.0f); hudText(80,  30, "Z_b (fwd)");

    // Controls
    glColor3f(0.50f, 0.50f, 0.50f);
    hudText(10, 10, "<- ->: roll p  |  up dn: pitch q  |  A D: yaw r  |  w/s: palanca de gases  "
                    "|  Space: pause  |  0: reset  |  Shift+W: wire  |  Q: quit");

    glEnable(GL_LIGHTING);
}

// ═══════════════════════════════════════════════════════════════════════
// REFERENCE GEOMETRY
// ═══════════════════════════════════════════════════════════════════════

// Fixed world axes (do not rotate with drone)
static void drawWorldAxes()
{
    glDisable(GL_LIGHTING);
    glLineWidth(1.2f);
    glBegin(GL_LINES);
        glColor3f(0.35f,0.10f,0.10f); glVertex3f(0,0,0); glVertex3f(8,0,0);
        glColor3f(0.10f,0.35f,0.10f); glVertex3f(0,0,0); glVertex3f(0,8,0);
        glColor3f(0.10f,0.10f,0.35f); glVertex3f(0,0,0); glVertex3f(0,0,-8);
    glEnd();
    glLineWidth(1.0f);
    glEnable(GL_LIGHTING);
}

// Body-frame axes (rotate with drone)
static void drawBodyAxes()
{
    glDisable(GL_LIGHTING);
    glLineWidth(2.5f);
    glBegin(GL_LINES);
        glColor3f(1.0f, 0.25f, 0.25f); glVertex3f(0,0,0); glVertex3f(7, 0, 0);
        glColor3f(0.25f, 1.0f, 0.25f); glVertex3f(0,0,0); glVertex3f(0,-7, 0);
        glColor3f(0.35f, 0.55f, 1.0f); glVertex3f(0,0,0); glVertex3f(0, 0,-7);
    glEnd();
    glLineWidth(1.0f);
    glEnable(GL_LIGHTING);
}

// Horizontal reference circle — helps see the roll visually
static void drawHorizonRing()
{
    glDisable(GL_LIGHTING);
    glColor3f(0.30f, 0.30f, 0.40f);
    glLineWidth(1.0f);
    glBegin(GL_LINE_LOOP);
    for (int i = 0; i < 64; i++) {
        float a = (float)i * 2.0f * (float)M_PI / 64.0f;
        glVertex3f(12.0f*std::cos(a), 0.0f, 12.0f*std::sin(a));
    }
    glEnd();
    glEnable(GL_LIGHTING);
}

// Piso solido en el plano X-Y (pd=PISO_PD, local y=0 en GL) — no es solo
// referencia visual: es la misma restriccion que checkGroundCollision()
// evalua en step(). Se pinta rojo cuando hay colision.
static void drawGroundGrid()
{
    const float extent = 200.0f, step_ = 10.0f;

    glDisable(GL_LIGHTING);

    // Relleno solido semitransparente del piso
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    if (g_crashed) glColor4f(0.75f, 0.15f, 0.12f, 0.55f);
    else           glColor4f(0.16f, 0.22f, 0.30f, 0.35f);
    glBegin(GL_QUADS);
        glVertex3f(-extent, 0.0f, -extent);
        glVertex3f( extent, 0.0f, -extent);
        glVertex3f( extent, 0.0f,  extent);
        glVertex3f(-extent, 0.0f,  extent);
    glEnd();
    glDisable(GL_BLEND);

    // Grilla encima, para dar referencia de escala/desplazamiento
    if (g_crashed) glColor3f(0.95f, 0.30f, 0.25f);
    else           glColor3f(0.30f, 0.30f, 0.40f);
    glLineWidth(1.0f);
    glBegin(GL_LINES);
    for (float v = -extent; v <= extent; v += step_) {
        glVertex3f(v, 0.0f, -extent); glVertex3f(v, 0.0f, extent);
        glVertex3f(-extent, 0.0f, v); glVertex3f(extent, 0.0f, v);
    }
    glEnd();

    glEnable(GL_LIGHTING);
}

// Obstaculo fijo — mismo bloque que evalua checkObstacleCollision() en
// step(). Coordenadas locales: x=pe, z=-pn (NED pn -> GL -Z, igual que el
// traslado de mundo en display() y drawBodyAxes(); la nariz del dron mira
// hacia -Z, asi que un obstaculo con pn>0 queda por delante, no detras),
// y=altura sobre el piso (pd=PISO_PD -> local y=0, techo del bloque -> local y=height).
// Se pinta rojo cuando el dron choco contra el.
static void drawObstacle()
{
    const float halfPn = (float)(g_obstacle.sizePn / 2.0);
    const float halfPe = (float)(g_obstacle.sizePe / 2.0);
    const float x0 = (float)g_obstacle.peCenter - halfPe, x1 = (float)g_obstacle.peCenter + halfPe;
    const float z0 = -(float)g_obstacle.pnCenter - halfPn, z1 = -(float)g_obstacle.pnCenter + halfPn;
    const float y0 = 0.0f, y1 = (float)g_obstacle.height;

    glDisable(GL_LIGHTING);

    if (g_obstacleHit) glColor3f(0.95f, 0.30f, 0.25f);
    else                glColor3f(0.80f, 0.55f, 0.15f);

    glBegin(GL_QUADS);
        // Caras laterales (-pn, +pn, -pe, +pe)
        glVertex3f(x0,y0,z0); glVertex3f(x1,y0,z0); glVertex3f(x1,y1,z0); glVertex3f(x0,y1,z0);
        glVertex3f(x1,y0,z1); glVertex3f(x0,y0,z1); glVertex3f(x0,y1,z1); glVertex3f(x1,y1,z1);
        glVertex3f(x0,y0,z1); glVertex3f(x0,y0,z0); glVertex3f(x0,y1,z0); glVertex3f(x0,y1,z1);
        glVertex3f(x1,y0,z0); glVertex3f(x1,y0,z1); glVertex3f(x1,y1,z1); glVertex3f(x1,y1,z0);
        // Techo
        glVertex3f(x0,y1,z0); glVertex3f(x1,y1,z0); glVertex3f(x1,y1,z1); glVertex3f(x0,y1,z1);
    glEnd();

    // Contorno para que se distinga el volumen desde lejos
    glColor3f(0.15f, 0.15f, 0.15f);
    glLineWidth(1.5f);
    glBegin(GL_LINE_LOOP);
        glVertex3f(x0,y0,z0); glVertex3f(x1,y0,z0); glVertex3f(x1,y1,z0); glVertex3f(x0,y1,z0);
    glEnd();
    glBegin(GL_LINE_LOOP);
        glVertex3f(x0,y0,z1); glVertex3f(x1,y0,z1); glVertex3f(x1,y1,z1); glVertex3f(x0,y1,z1);
    glEnd();
    glLineWidth(1.0f);

    glEnable(GL_LIGHTING);
}

// ═══════════════════════════════════════════════════════════════════════
// DRONE GEOMETRY  (nose at -Z, right wing at +X, up at +Y)
// ═══════════════════════════════════════════════════════════════════════

static void drawFuselage(GLUquadric* q)
{
    glColor3f(0.72f, 0.75f, 0.78f);
    glPushMatrix(); glTranslatef(0,0,-4.5f); glRotatef(180,0,1,0);
    gluDisk(q,0,0.05f,16,1); glPopMatrix();

    glPushMatrix(); glTranslatef(0,0,-4.5f);
    gluCylinder(q,0.05f,0.50f,1.5f,20,4); glPopMatrix();

    glPushMatrix(); glTranslatef(0,0,-3.0f);
    gluCylinder(q,0.50f,0.50f,5.0f,20,4); glPopMatrix();

    glPushMatrix(); glTranslatef(0,0, 2.0f);
    gluCylinder(q,0.50f,0.28f,2.0f,20,4); glPopMatrix();

    glColor3f(0.60f, 0.63f, 0.66f);
    glPushMatrix(); glTranslatef(0,0,4.0f);
    gluCylinder(q,0.30f,0.30f,0.8f,20,4); glPopMatrix();

    glPushMatrix(); glTranslatef(0,0,4.8f);
    gluDisk(q,0,0.30f,20,1); glPopMatrix();
}

static void drawWing(float side)
{
    float rx=side*0.50f, rzle=-0.8f, rzte=1.8f, ry=0.05f;
    float tx=side*10.0f, tzle= 0.5f, tzte=2.2f, ty=0.90f;

    glColor3f(0.72f, 0.75f, 0.78f);
    glNormal3f(0,1,0);
    glBegin(GL_QUADS);
        glVertex3f(rx,ry,     rzle); glVertex3f(rx,ry,     rzte);
        glVertex3f(tx,ty,     tzte); glVertex3f(tx,ty,     tzle);
    glEnd();
    glColor3f(0.62f, 0.65f, 0.68f);
    glNormal3f(0,-1,0);
    glBegin(GL_QUADS);
        glVertex3f(rx,ry-0.07f,rzle); glVertex3f(tx,ty-0.07f,tzle);
        glVertex3f(tx,ty-0.07f,tzte); glVertex3f(rx,ry-0.07f,rzte);
    glEnd();
    glColor3f(0.68f, 0.71f, 0.73f);
    glNormal3f(0,0,-1);
    glBegin(GL_QUADS);
        glVertex3f(rx,ry,      rzle); glVertex3f(tx,ty,      tzle);
        glVertex3f(tx,ty-0.07f,tzle); glVertex3f(rx,ry-0.07f,rzle);
    glEnd();
    glNormal3f(0,0,1);
    glBegin(GL_QUADS);
        glVertex3f(rx,ry,      rzte); glVertex3f(rx,ry-0.07f,rzte);
        glVertex3f(tx,ty-0.07f,tzte); glVertex3f(tx,ty,      tzte);
    glEnd();
    glNormal3f(side,0,0);
    glBegin(GL_QUADS);
        glVertex3f(tx,ty,      tzle); glVertex3f(tx,ty,      tzte);
        glVertex3f(tx,ty-0.07f,tzte); glVertex3f(tx,ty-0.07f,tzle);
    glEnd();
}

static void drawTailFin(float side)
{
    const float ang = 52.0f * (float)M_PI / 180.0f;
    const float span=3.2f, cr=1.8f, ct_val=0.85f, z0=3.8f;
    float tx=side*span*std::cos(ang), ty=span*std::sin(ang);   // ty>0: alerones hacia arriba
    float ax=0,ay=0,az=z0,  bx=0,by=0,bz=z0+cr;
    float cx=tx,cy=ty,cz=z0+0.5f, dx=tx,dy=ty,dz=z0+0.5f+ct_val;
    float nx=side*std::sin(ang), ny=-std::cos(ang);
    glColor3f(0.70f, 0.73f, 0.75f);
    glNormal3f(-nx,ny,0);
    glBegin(GL_QUADS);
        glVertex3f(ax,ay,az); glVertex3f(bx,by,bz);
        glVertex3f(dx,dy,dz); glVertex3f(cx,cy,cz);
    glEnd();
    glNormal3f(nx,-ny,0);
    glBegin(GL_QUADS);
        glVertex3f(ax,ay,az); glVertex3f(cx,cy,cz);
        glVertex3f(dx,dy,dz); glVertex3f(bx,by,bz);
    glEnd();
}

static void drawSensorBall()
{
    glColor3f(0.12f,0.12f,0.12f);
    glPushMatrix(); glTranslatef(0,-0.65f,-3.6f);
    glutSolidSphere(0.38,20,20); glPopMatrix();
}

static void drawSatcomDome()
{
    glColor3f(0.88f,0.88f,0.88f);
    glPushMatrix(); glTranslatef(0,0.60f,-0.8f);
    glScalef(0.48f,0.32f,0.48f);
    glutSolidSphere(1.0,20,20); glPopMatrix();
}

static void drawPropeller()
{
    glDisable(GL_LIGHTING);
    glColor3f(0.15f,0.15f,0.15f);
    glPushMatrix(); glTranslatef(0,0,4.85f);
    glutSolidSphere(0.12,12,8); glPopMatrix();
    for (int i = 0; i < 2; i++) {
        glPushMatrix(); glTranslatef(0,0,4.85f);
        glRotatef(i*90.0f,0,0,1);
        glBegin(GL_QUADS);
            glVertex3f(-0.07f,-1.9f,0.05f); glVertex3f(0.07f,-1.9f,0.05f);
            glVertex3f(0.15f,  1.9f,0.05f); glVertex3f(-0.15f,1.9f,0.05f);
        glEnd();
        glPopMatrix();
    }
    glEnable(GL_LIGHTING);
}

static void drawDrone()
{
    GLUquadric* q = gluNewQuadric();
    gluQuadricNormals(q, GLU_SMOOTH);
    drawFuselage(q);
    drawWing(+1.0f); drawWing(-1.0f);
    drawTailFin(+1.0f); drawTailFin(-1.0f);
    drawSensorBall(); drawSatcomDome();
    drawPropeller();
    gluDeleteQuadric(q);
}

// ═══════════════════════════════════════════════════════════════════════
// TELEMETRY WINDOW — ventana secundaria: evolucion temporal del estado
// (osciloscopio de vuelo, separado de la ventana 3D del dron)
// ═══════════════════════════════════════════════════════════════════════

static int g_telW = 520, g_telH = 760;
static int g_mainWin = 0, g_telemetryWin = 0;

// Scratch buffers reusados panel a panel (hasta 3 series por panel)
static double g_scratchA[TELEMETRY_CAPACITY];
static double g_scratchB[TELEMETRY_CAPACITY];
static double g_scratchC[TELEMETRY_CAPACITY];

static void telemetryText(float x, float y, const char* s)
{
    glRasterPos2f(x, y);
    while (*s) glutBitmapCharacter(GLUT_BITMAP_8_BY_13, *s++);
}

// Rango [lo,hi] con margen, evitando escala degenerada cuando la serie es plana
static void minMaxPadded(const double* v, int n, double& lo, double& hi)
{
    lo = v[0]; hi = v[0];
    for (int i = 1; i < n; ++i) {
        if (v[i] < lo) lo = v[i];
        if (v[i] > hi) hi = v[i];
    }
    if (hi - lo < 1e-6) { lo -= 0.5; hi += 0.5; }
    double pad = (hi - lo) * 0.08;
    lo -= pad; hi += pad;
}

// Ancho aproximado (px) de un string en GLUT_BITMAP_8_BY_13 (8px/caracter)
static float textWidth(const char* s) { return 8.0f * (float)std::strlen(s); }

// Traza una serie dentro del area de ploteo [x0,y0,w,h] (autoescalada a [minV,maxV])
static void plotSeries(float x0, float y0, float w, float h,
                        double minV, double maxV,
                        const double* vals, int count,
                        float r, float g, float b)
{
    if (count < 2 || maxV <= minV) return;
    glColor3f(r, g, b);
    glBegin(GL_LINE_STRIP);
    for (int i = 0; i < count; ++i) {
        float px = x0 + w * (float)i / (float)(count - 1);
        float t  = (float)((vals[i] - minV) / (maxV - minV));
        float py = y0 + t * h;
        glVertex2f(px, py);
    }
    glEnd();
}

struct Series {
    const double* data;
    const char*   label;   // nombre corto para la leyenda (u, v, w, ...)
    float r, g, b;
};

// Panel tipo osciloscopio completo: marco, titulo con unidad, leyenda de
// series, eje Y con valores min/max (+ linea de referencia en 0) y eje X
// con el rango de tiempo [t0,t1] en segundos que cubre el buffer visible.
static void drawTelemetryPanel(float x0, float y0, float w, float h,
                                const char* varName, const char* unit,
                                const Series* series, int nSeries,
                                double t0, double t1)
{
    // Layout interno: title (arriba), area de ploteo, eje X (abajo)
    const float titleH = 16.0f, axisH = 14.0f, padX = 6.0f;
    const float plotX = x0 + padX, plotW = w - 2.0f*padX;
    const float plotY = y0 + axisH, plotH = h - titleH - axisH;

    // Fondo + marco del panel
    glColor3f(0.13f, 0.13f, 0.17f);
    glBegin(GL_QUADS);
        glVertex2f(x0, y0); glVertex2f(x0+w, y0);
        glVertex2f(x0+w, y0+h); glVertex2f(x0, y0+h);
    glEnd();
    glColor3f(0.36f, 0.36f, 0.44f);
    glBegin(GL_LINE_LOOP);
        glVertex2f(x0, y0); glVertex2f(x0+w, y0);
        glVertex2f(x0+w, y0+h); glVertex2f(x0, y0+h);
    glEnd();

    // Titulo: nombre de la variable + unidad
    char title[80];
    snprintf(title, sizeof(title), "%s [%s]", varName, unit);
    glColor3f(0.85f, 0.85f, 0.90f);
    telemetryText(x0 + padX, y0 + h - 12, title);

    // Leyenda de series (u, v, w, ...) en el color de cada linea
    if (nSeries > 1) {
        float lx = x0 + w - padX;
        for (int i = nSeries - 1; i >= 0; --i) {
            float lw = textWidth(series[i].label);
            lx -= lw;
            glColor3f(series[i].r, series[i].g, series[i].b);
            telemetryText(lx, y0 + h - 12, series[i].label);
            lx -= 10.0f;
        }
    }

    if (g_telemetryCount < 2) return;

    // Rango combinado de todas las series (para compartir la misma escala)
    double lo = 0.0, hi = 0.0;
    for (int i = 0; i < nSeries; ++i) {
        double slo, shi;
        minMaxPadded(series[i].data, g_telemetryCount, slo, shi);
        if (i == 0) { lo = slo; hi = shi; }
        else        { lo = std::min(lo, slo); hi = std::max(hi, shi); }
    }

    // Eje Y: valores maximo (arriba) y minimo (abajo), alineados a la derecha
    char buf[32];
    glColor3f(0.60f, 0.60f, 0.66f);
    snprintf(buf, sizeof(buf), "%.2f", hi);
    telemetryText(plotX + plotW - textWidth(buf), plotY + plotH - 11, buf);
    snprintf(buf, sizeof(buf), "%.2f", lo);
    telemetryText(plotX + plotW - textWidth(buf), plotY + 1, buf);

    // Linea de referencia en 0, si el rango la cruza
    if (lo < 0.0 && hi > 0.0 && hi > lo) {
        float t  = (float)((0.0 - lo) / (hi - lo));
        float py = plotY + t * plotH;
        glColor3f(0.32f, 0.32f, 0.38f);
        glBegin(GL_LINES); glVertex2f(plotX, py); glVertex2f(plotX+plotW, py); glEnd();
    }

    // Series de datos
    for (int i = 0; i < nSeries; ++i)
        plotSeries(plotX, plotY, plotW, plotH, lo, hi,
                   series[i].data, g_telemetryCount, series[i].r, series[i].g, series[i].b);

    // Eje X: tiempo — extremos del rango visible, en segundos
    glColor3f(0.55f, 0.55f, 0.60f);
    snprintf(buf, sizeof(buf), "%.1f s", t0);
    telemetryText(plotX, y0 + 1, buf);
    snprintf(buf, sizeof(buf), "%.1f s", t1);
    telemetryText(plotX + plotW - textWidth(buf), y0 + 1, buf);
}

static void telemetryDisplay()
{
    glClear(GL_COLOR_BUFFER_BIT);

    glColor3f(0.85f, 0.85f, 0.90f);
    telemetryText(10, (float)g_telH - 20, "TELEMETRIA -- evolucion de estado");

    const int n = g_telemetryCount;
    if (n < 2) { glutSwapBuffers(); return; }

    const float margin  = 12.0f;
    const float topSkip = 30.0f;
    const int   NPANELS = 5;
    const float gap     = 8.0f;
    const float panelW  = (float)g_telW - 2*margin;
    const float panelH  = ((float)g_telH - topSkip - margin - (NPANELS-1)*gap) / NPANELS;
    const double r2d = 180.0 / M_PI;
    const double t0 = telemetryAt(0).t, t1 = telemetryAt(n-1).t;

    float y = (float)g_telH - topSkip - panelH;

    // Panel 1: Altitud sobre el piso (plano X-Y, pd=PISO_PD) [m]
    for (int i = 0; i < n; ++i) g_scratchA[i] = PISO_PD - telemetryAt(i).pd;
    {
        Series s[] = {{ g_scratchA, "alt", 0.55f, 0.80f, 1.0f }};
        drawTelemetryPanel(margin, y, panelW, panelH, "Altitud sobre el piso", "m", s, 1, t0, t1);
    }
    y -= panelH + gap;

    // Panel 2: Velocidad body u,v,w [m/s]
    for (int i = 0; i < n; ++i) {
        const TelemetrySample& smp = telemetryAt(i);
        g_scratchA[i] = smp.u; g_scratchB[i] = smp.v; g_scratchC[i] = smp.w;
    }
    {
        Series s[] = {{ g_scratchA, "u", 1.0f,0.40f,0.40f },
                      { g_scratchB, "v", 0.40f,1.0f,0.40f },
                      { g_scratchC, "w", 0.45f,0.60f,1.0f }};
        drawTelemetryPanel(margin, y, panelW, panelH, "Velocidad body", "m/s", s, 3, t0, t1);
    }
    y -= panelH + gap;

    // Panel 3: Actitud phi,theta,psi [deg]
    for (int i = 0; i < n; ++i) {
        const TelemetrySample& smp = telemetryAt(i);
        g_scratchA[i] = smp.phi*r2d; g_scratchB[i] = smp.theta*r2d; g_scratchC[i] = smp.psi*r2d;
    }
    {
        Series s[] = {{ g_scratchA, "phi", 1.0f,0.40f,0.40f },
                      { g_scratchB, "theta", 0.40f,1.0f,0.40f },
                      { g_scratchC, "psi", 0.45f,0.60f,1.0f }};
        drawTelemetryPanel(margin, y, panelW, panelH, "Actitud", "deg", s, 3, t0, t1);
    }
    y -= panelH + gap;

    // Panel 4: Tasas angulares p,q,r [deg/s]
    for (int i = 0; i < n; ++i) {
        const TelemetrySample& smp = telemetryAt(i);
        g_scratchA[i] = smp.p*r2d; g_scratchB[i] = smp.q*r2d; g_scratchC[i] = smp.r*r2d;
    }
    {
        Series s[] = {{ g_scratchA, "p", 1.0f,0.40f,0.40f },
                      { g_scratchB, "q", 0.40f,1.0f,0.40f },
                      { g_scratchC, "r", 0.45f,0.60f,1.0f }};
        drawTelemetryPanel(margin, y, panelW, panelH, "Tasas angulares", "deg/s", s, 3, t0, t1);
    }
    y -= panelH + gap;

    // Panel 5: Posicion horizontal pn,pe [m]
    for (int i = 0; i < n; ++i) {
        const TelemetrySample& smp = telemetryAt(i);
        g_scratchA[i] = smp.pn; g_scratchB[i] = smp.pe;
    }
    {
        Series s[] = {{ g_scratchA, "pn", 1.0f,0.40f,0.40f },
                      { g_scratchB, "pe", 0.40f,1.0f,0.40f }};
        drawTelemetryPanel(margin, y, panelW, panelH, "Posicion horizontal", "m", s, 2, t0, t1);
    }

    glutSwapBuffers();
}

static void telemetryReshape(int w, int h)
{
    if (h == 0) h = 1;
    g_telW = w; g_telH = h;
    glViewport(0, 0, w, h);
    glMatrixMode(GL_PROJECTION); glLoadIdentity();
    glOrtho(0, w, 0, h, -1, 1);
    glMatrixMode(GL_MODELVIEW); glLoadIdentity();
}

// Interferencia estilo "senal de TV perdida" — se dibuja cuando g_crashed
// esta activo (choque contra el piso o contra el obstaculo), sobre el
// HUD/escena ya renderizados, en coordenadas de pantalla (mismo ortho que
// hudText). Saturada a proposito: comunicacion totalmente perdida, no un
// glitch leve.
static void drawSignalLossOverlay()
{
    glMatrixMode(GL_PROJECTION); glPushMatrix(); glLoadIdentity();
    glOrtho(0, 900, 0, 600, -1, 1);
    glMatrixMode(GL_MODELVIEW);  glPushMatrix(); glLoadIdentity();
    glDisable(GL_LIGHTING); glDisable(GL_DEPTH_TEST);
    glEnable(GL_BLEND); glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // Fondo oscurecido — tapa casi toda la escena, comunicacion perdida
    glColor4f(0.0f, 0.0f, 0.0f, 0.55f);
    glBegin(GL_QUADS);
        glVertex2f(0,0); glVertex2f(900,0); glVertex2f(900,600); glVertex2f(0,600);
    glEnd();

    // "Nieve" densa — bloques de ruido aleatorio, estilo TV sin senal
    glBegin(GL_QUADS);
    for (int i = 0; i < 200; i++) {
        float x = (float)(rand() % 900), y = (float)(rand() % 600);
        float s = 3.0f + (float)(rand() % 5);
        float g = (float)(rand() % 100) / 100.0f;
        glColor4f(g, g, g, 0.55f);
        glVertex2f(x,y); glVertex2f(x+s,y); glVertex2f(x+s,y+s); glVertex2f(x,y+s);
    }
    glEnd();

    // Franjas de glitch horizontal (barras anchas desplazadas)
    glBegin(GL_QUADS);
    for (int i = 0; i < 25; i++) {
        float y     = (float)(rand() % 600);
        float h     = 3.0f + (float)(rand() % 14);
        float xOff  = (float)((rand() % 160) - 80);
        float shade = 0.2f + 0.7f * (float)(rand() % 100) / 100.0f;
        glColor4f(shade, shade, shade, 0.55f);
        glVertex2f(xOff,      y);   glVertex2f(900+xOff, y);
        glVertex2f(900+xOff,  y+h); glVertex2f(xOff,     y+h);
    }
    glEnd();

    // Parpadeo rojo de alarma — cada frame, mas caotico que un timer fijo
    if (rand() % 2 == 0) {
        glColor4f(0.65f, 0.0f, 0.0f, 0.20f);
        glBegin(GL_QUADS);
            glVertex2f(0,0); glVertex2f(900,0); glVertex2f(900,600); glVertex2f(0,600);
        glEnd();
    }

    glDisable(GL_BLEND);
    glEnable(GL_DEPTH_TEST); glEnable(GL_LIGHTING);
    glMatrixMode(GL_PROJECTION); glPopMatrix();
    glMatrixMode(GL_MODELVIEW);  glPopMatrix();
}

// ═══════════════════════════════════════════════════════════════════════
// GLUT CALLBACKS
// ═══════════════════════════════════════════════════════════════════════

static void display()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();

    glTranslatef(0, 0, g_zoom);
    glRotatef(g_rotX, 1, 0, 0);
    glRotatef(g_rotY, 0, 1, 0);

    // World reference — se traslada en sentido opuesto a la posicion inercial
    // del dron (camara "chase"): el dron queda centrado en pantalla y se ve
    // el mundo recular a medida que vuela. NED (pn,pe,pd) -> GL (pe,-pd,-pn).
    glPushMatrix();
    glTranslatef(-(float)g_state.pe, (float)g_state.pd, (float)g_state.pn);
    drawWorldAxes();
    drawHorizonRing();
    drawGroundGrid();
    drawObstacle();
    glPopMatrix();

    // Drone centrado en pantalla, orientado con la actitud actual
    // Rotation order ZYX (yaw → pitch → roll) — NED convention
    glPushMatrix();
    glRotatef(-(float)(g_state.psi   * 180.0/M_PI), 0, 1, 0);  // yaw
    glRotatef( (float)(g_state.theta * 180.0/M_PI), 1, 0, 0);  // pitch
    glRotatef(-(float)(g_state.phi   * 180.0/M_PI), 0, 0, 1);  // roll

    glPolygonMode(GL_FRONT_AND_BACK, g_wire ? GL_LINE : GL_FILL);
    drawBodyAxes();
    drawDrone();
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    glPopMatrix();

    drawHUD();
    if (g_crashed) drawSignalLossOverlay();
    glutSwapBuffers();
}

static void timerStep(int)
{
    if (!g_paused && !g_crashed)
        step(DT);

    glutSetWindow(g_mainWin);       glutPostRedisplay();
    glutSetWindow(g_telemetryWin);  glutPostRedisplay();
    glutTimerFunc(16, timerStep, 0);
}

static void reshape(int w, int h)
{
    if (h == 0) h = 1;
    glViewport(0, 0, w, h);
    glMatrixMode(GL_PROJECTION); glLoadIdentity();
    gluPerspective(45.0, (double)w/h, 0.1, 500.0);
    glMatrixMode(GL_MODELVIEW);
}

static void mouseButton(int btn, int state, int x, int y)
{
    if (btn == GLUT_LEFT_BUTTON) { g_drag=(state==GLUT_DOWN); g_lastX=x; g_lastY=y; }
    if (btn == 3) { g_zoom += 1.2f; glutPostRedisplay(); }
    if (btn == 4) { g_zoom -= 1.2f; glutPostRedisplay(); }
}

static void mouseMotion(int x, int y)
{
    if (!g_drag) return;
    g_rotY += (x - g_lastX) * 0.5f;
    g_rotX += (y - g_lastY) * 0.5f;
    g_lastX=x; g_lastY=y;
    glutPostRedisplay();
}

static void keyboard(unsigned char key, int, int)
{
    const double dp = 0.02;
    if (g_crashed && key != '0' && key != 'q' && key != 'Q' && key != 27) {
        glutPostRedisplay();
        return;
    }
    switch (key) {
        case 'a': case 'A': g_state.r -= dp; break;
        case 'd': case 'D': g_state.r += dp; break;
        case 'w': g_keyW = true; break;              // palanca +  (Bloques 1-2)
        case 's': case 'S': g_keyS = true; break;     // palanca -  (Bloques 1-2)
        case ' ': g_paused = !g_paused;    break;
        case '0': g_state = g_init; g_crashed = false; g_obstacleHit = false; g_throttle = THROTTLE_INIT; resetTelemetry(); break;
        case 'W': g_wire = !g_wire; break;             // Shift+W: wireframe
        case 'r': case 'R': g_rotX=20; g_rotY=-35; g_zoom=-22; break;
        case '+': case '=': g_zoom += 1.5f; break;
        case '-':           g_zoom -= 1.5f; break;
        case 'q': case 'Q': case 27: exit(0);
    }
    glutPostRedisplay();
}

// Suelta W/S — el empuje se sostiene en el valor alcanzado, no vuelve a cero
static void keyboardUp(unsigned char key, int, int)
{
    switch (key) {
        case 'w': g_keyW = false; break;
        case 's': case 'S': g_keyS = false; break;
    }
}

static void specialKey(int key, int, int)
{
    const double dp = 0.02;
    if (g_crashed) { glutPostRedisplay(); return; }
    switch (key) {
        case GLUT_KEY_LEFT:  g_state.p -= dp; break;
        case GLUT_KEY_RIGHT: g_state.p += dp; break;
        case GLUT_KEY_UP:    g_state.q += dp; break;
        case GLUT_KEY_DOWN:  g_state.q -= dp; break;
    }
    glutPostRedisplay();
}

// ═══════════════════════════════════════════════════════════════════════
// MAIN
// ═══════════════════════════════════════════════════════════════════════

int main(int argc, char** argv)
{
    // Bloque 0 — masa/inercia real del dron ala delta (drone_geometry)
    g_drone = buildDeltaWingDrone();

    g_state = UAVState{};
    g_state.pd = -50.0;    // 50 m de altitud inicial (NED: negativo = arriba)
    g_state.u  = U_INIT;   // m/s de crucero (coincide con THROTTLE_INIT)
    g_state.p  = (argc > 1) ? atof(argv[1]) : 0.0;
    g_state.q  = (argc > 2) ? atof(argv[2]) : 0.0;
    g_state.r  = (argc > 3) ? atof(argv[3]) : 0.0;
    g_init     = g_state;

    printf("Masa total: %.1f kg  |  Ixx=%.1f Iyy=%.1f Izz=%.1f kg*m^2\n",
           g_drone.mass, g_drone.I_full.Ixx, g_drone.I_full.Iyy, g_drone.I_full.Izz);
    printf("Rates iniciales: p=%.3f  q=%.3f  r=%.3f rad/s\n",
           g_state.p, g_state.q, g_state.r);

    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(900, 600);
    glutInitWindowPosition(50, 50);
    g_mainWin = glutCreateWindow("UAV — Vuelo 6DOF");

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_COLOR_MATERIAL);
    glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
    glShadeModel(GL_SMOOTH);

    GLfloat lpos[]={10,20,-5,0}, lamb[]={0.25f,0.25f,0.25f,1}, ldif[]={0.85f,0.85f,0.85f,1};
    glLightfv(GL_LIGHT0, GL_POSITION, lpos);
    glLightfv(GL_LIGHT0, GL_AMBIENT,  lamb);
    glLightfv(GL_LIGHT0, GL_DIFFUSE,  ldif);
    glClearColor(0.07f, 0.07f, 0.11f, 1.0f);

    glutDisplayFunc(display);
    glutReshapeFunc(reshape);
    glutMouseFunc(mouseButton);
    glutMotionFunc(mouseMotion);
    glutKeyboardFunc(keyboard);
    glutKeyboardUpFunc(keyboardUp);
    glutSpecialFunc(specialKey);

    // Ventana secundaria — telemetria (osciloscopio 2D, sin luces/profundidad)
    glutInitWindowSize(g_telW, g_telH);
    glutInitWindowPosition(970, 50);
    g_telemetryWin = glutCreateWindow("UAV — Telemetria");

    glDisable(GL_DEPTH_TEST);
    glDisable(GL_LIGHTING);
    glClearColor(0.05f, 0.05f, 0.07f, 1.0f);

    glutDisplayFunc(telemetryDisplay);
    glutReshapeFunc(telemetryReshape);

    glutSetWindow(g_mainWin);
    glutTimerFunc(16, timerStep, 0);

    glutMainLoop();
    return 0;
}
