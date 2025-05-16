#include "esp_rom_sys.h" 
extern "C" void early_print() __attribute__((constructor));
extern "C" void early_print() {
    ets_printf("\n[EARLY] made it into early_print()\n");
}

#include <Arduino.h>
#include <ESP32Servo.h>
#include <math.h>
#include "server.h"

#include <iostream>
#include <vector>
#include <cmath>

const float SEGMENT_LEN = 1;

/*
* HARDWARE DEFINITIONS
*/
const int SERVO_PINS[6] = {
    12,             // SG90 - 180deg - shoulder left/right
    14,             // SG90 - 180deg - shoulder up/down
    27,             // SG90 - 180deg - elbow rotation
    26,             // SG90 - 180deg - elbow folding
    33,             // SG90 - 180deg - hand/gripper rotation
    32              // MG996R - 360d - base
};

Servo servos[6];

// pulse lengths & freq for the servos
constexpr int MIN_PULSE_US = 500;
constexpr int MAX_PULSE_US = 2400;
constexpr int PWM_FREQ_HZ  = 50;    

// speeds for the MG996R
constexpr int     IDLE = 1500;
constexpr int  CW_SLOW = 1700;
constexpr int  CW_FAST = 1900;
constexpr int CCW_SLOW = 1300;
constexpr int CCW_FAST = 1100;


// lists of degrees just for safety purposes
double OFF_DEG[4] = { 90,  90,  90,  90};     
double MIN_DEG[4] = {  0,   0,   0,   0};  
double MAX_DEG[4] = {180, 180, 180, 180};

/*
* COOL MATH STUFF
*/

// Convert between radians and degrees
double convert_rad_deg(double r) { return r * 180.0 / PI; }
double convert_deg_rad(double d) { return d * PI / 180.0; }

// Fix any angle to be in the interval [-pi, pi]
double fixAngle(double angle)
{
    // downscale(?) angle to [-360, 360]
    angle = fmod(angle, 2.0 * PI);

    if (angle > PI) {
        return angle - (2 * PI);
    }
    else if (angle < -PI) {
        return angle + (2 * PI);
    }

    return angle;
}

// Calculate distance between two points in R^2 or R^3
double dist(std::vector<double> p1, std::vector<double> p2 = {0, 0, 0}) {
    if (p1.size() == 2) 
        // we are in R^2
        return std::sqrt((p1[0] - p2[0]) * (p1[0] - p2[0]) + (p1[1] - p2[1]) * (p1[1] - p2[1]));
    else
        // we are in R^3
        return std::sqrt((p1[0] - p2[0]) * (p1[0] - p2[0]) + (p1[1] - p2[1]) * (p1[1] - p2[1]) + (p1[2] - p2[2]) * (p1[2] - p2[2]));
}

// Cross product
std::vector<double> cross(const std::vector<double>& u,
                        const std::vector<double>& v)
{
    return {
        u[1] * v[2] - u[2] * v[1],
        u[2] * v[0] - u[0] * v[2],
        u[0] * v[1] - u[1] * v[0]
    };
}

// Multiply a matrix m by a vector v
std::vector<double> matVec(const std::vector<std::vector<double>>& m,
                        const std::vector<double>& v)
{
    std::vector<double> r(3, 0);
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            r[i] += m[i][j] * v[j];
    return r;
}

/*
* INVERSE KINEMATICS TYPE SHIT
*/

// Find the position of the first point
std::vector<double> param_intersection(const std::vector<double>& p, double o)
{
    double A = dist({p[0], p[1]});
    double d = dist(p);
    double R = sqrt(SEGMENT_LEN * SEGMENT_LEN - 0.25 * d * d);

    std::vector<double> V = A == 0  ? std::vector<double>{1, 0, 0}
                                    : std::vector<double>{-p[1] / A, p[0] / A, 0};
    std::vector<double> U = {p[0] / d, p[1] / d, p[2] / d};
    std::vector<double> Z = cross(U, V);

    std::vector<double> out(3);
    for (int i = 0; i < 3; ++i)
        out[i] = 0.5 * d * U[i] + R * cos(o) * V[i] + R * sin(o) * Z[i];
    return out;
}

// Calculate the angles required to get the segments aligned to a point
std::vector<double> calc_angles(const std::vector<double>& p)
{
    double xc = sqrt(SEGMENT_LEN * SEGMENT_LEN - p[1] * p[1]);
    return {
        -2 * atan2(p[2] - xc, p[0]),
        -2 * atan2(xc, p[1] + SEGMENT_LEN)
    };
}

// Find the correct angles for the shoulder/elbow to reach the point
std::vector<double> target_angles(const std::vector<double>& p, double o)
{
    auto s1 = param_intersection(p, o);
    auto j1 = calc_angles(s1);
    std::vector<double> w = {p[0] - s1[0], p[1] - s1[1], p[2] - s1[2]};

    std::vector<std::vector<double>> T = {
        { cos(j1[0]),                        0, -sin(j1[0])},
        {-sin(j1[0]) * sin(j1[1]),  cos(j1[1]), -cos(j1[0]) * sin(j1[1])},
        { sin(j1[0]) * cos(j1[1]),  sin(j1[1]),  cos(j1[0]) * cos(j1[1])}
    };

    auto j2 = calc_angles(matVec(T, w));

    std::vector<double> a = {
        fixAngle(j1[0]),
        fixAngle(j1[1]),
        fixAngle(j2[0]),
        fixAngle(j2[1])
    };
    return a;
}

// -----------------------------------------------------------------------------
// Servo mapping & movement
// -----------------------------------------------------------------------------
void move_joint(int idx, double rad)
{
    if (idx > 3) return;                            // safety
    double deg = convert_rad_deg(rad) + OFF_DEG[idx];
    deg = constrain(deg, MIN_DEG[idx], MAX_DEG[idx]);
    servos[idx].write(static_cast<int>(deg));       // 0–180° SG90 path
}

void move_segment(int seg, const std::vector<double>& a)
{
    if (seg == 0) {
        move_joint(0, a[0]);
        move_joint(1, a[1]);
    } else {
        move_joint(2, a[0]);
        move_joint(3, a[1]);
    }
}

// Check if the movement provided is physically possible with the servos
bool within_limits(const std::vector<double>& a)
{
    for (int i = 1; i < 4; ++i)
        if (fabs(a[i]) > PI / 2) return false;
    return true;
}

// Actually move the arm to the desired point
bool move_to(const std::vector<double>& p)
{
    auto ang = target_angles(p, 0);
    bool ok = within_limits(ang);

    if (!ok) {
        for (int i = 1; i < 72; ++i) {
            ang = target_angles(p, i * PI / 36);
            if (within_limits(ang)) {
                ok = true;
                break;
            }
        }
    }

    if (ok) {
        move_segment(0, {ang[0], ang[1]});
        move_segment(1, {ang[2], ang[3]});
    }
    return ok;
}

/*
* FIRMWARE SCHEDULER TYPE SHIT
*/
void setup()
{
    Serial.begin(115200);
    for (int i = 0; i < 6; ++i) {
        // initialize each of the servos
        servos[i].setPeriodHertz(PWM_FREQ_HZ);
        servos[i].attach(SERVO_PINS[i], MIN_PULSE_US, MAX_PULSE_US);
    }

    // sentence MG996R to idle
    servos[5].writeMicroseconds(IDLE);

    init_server();
}

// script3.py - make a square
static std::vector<std::vector<double>> square_path = []() {
    std::vector<std::vector<double>> v;
    for (int i = 0; i < 200; ++i) v.push_back({-1, 1.2, 1 - i / 100.0});
    for (int i = 0; i < 200; ++i) v.push_back({-1 + i / 100.0, 1.2, -1});
    for (int i = 0; i < 200; ++i) v.push_back({ 1, 1.2, -1 + i / 100.0});
    for (int i = 0; i < 200; ++i) v.push_back({ 1 - i / 100.0, 1.2, 1});
    return v;
}();

void loop()
{
    static size_t step = 0;

    if (!manual_mode) {
        if (move_to((square_path)[step])) {
            ++step;
            step %= square_path.size();
        }
    }
    delay(20);
}