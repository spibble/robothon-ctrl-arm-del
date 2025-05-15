#include <iostream>
#include <vector>
#include <cmath>

const double PI = 3.141592653589;
const double segment_length = 1;

double convert(double x, bool radians = true) {
    if (radians) return x * PI / 180;
    else return x * 180 / PI;
}

double dist(std::vector<double> p1, std::vector<double> p2 = {0, 0, 0}) {
    if (p1.size() == 2) 
        return std::sqrt((p1[0] - p2[0]) * (p1[0] - p2[0]) + (p1[1] - p2[1]) * (p1[1] - p2[1]));
    else
        return std::sqrt((p1[0] - p2[0]) * (p1[0] - p2[0]) + (p1[1] - p2[1]) * (p1[1] - p2[1]) + (p1[2] - p2[2]) * (p1[2] - p2[2]));
}

double constrain(double x, double min = -PI / 2, double max = PI / 2) {
    if (x < min)
        return min;
    else if (x > max)
        return max;
    else
        return x;
}

double fix_angle(double x) {
    x = std::fmod(x, 2 * PI);
    if (x > PI)
        return x - 2 * PI;
    else if (x < -PI)
        return x + 2 * PI;
    else
        return x;
}

void move_joint(int joint, double angle) {
    //need to fill in
    //joint 0 - shoulder
    //joints 1-3 - other 3 motors on the arm
    //joints 5 and 6 - hand motors
}

void move_segment(int joint, std::vector<double> angles, bool radians = false) {
    if (!radians)
        for (int i = 0; i < 3; ++i)
            angles[i] = convert(angles[i]);

    for (int i = 0; i < 3; ++i)
        angles[i] = fix_angle(angles[i]);

    switch (joint) {
        case 0:
            move_joint(0, angles[0]);
            move_joint(1, angles[1]);
            break;
        case 1:
            move_joint(2, angles[0]);
            move_joint(3, angles[1]);
            break;
    }
}

std::vector<double> cross_product(std::vector<double> u, std::vector<double> v) {
    std::vector<double> result(3);
    result[0] = u[1] * v[2] - u[2] * v[1];
    result[1] = u[2] * v[0] - u[0] * v[2];
    result[2] = u[0] * v[1] - u[1] * v[0];

    return result;
}

std::vector<double> parameterized_intersection(std::vector<double> point, double orientation) {
    std::vector<double> output;
    std::vector<double> xy = {point[0], point[1]};
    std::vector<double> V = {0, 0, 0};

    double A = dist(xy);
    double d = dist(point);
    double R = std::sqrt(segment_length * segment_length - 0.25 * d * d);

    if (!A)
        V = {1, 0, 0};
    else
        V = {1 / A * -point[1], 1 / A * point[0], 0};

    std::vector<double> U = {1 / d * point[0], 1 / d * point[1], 1 / d * point[2]};
    std::vector<double> Z = cross_product(U, V);
    

    for (int i = 0; i < 3; ++i)
        output.push_back(0.5 * d * U[i] + R * std::cos(orientation) * V[i] + R * std::sin(orientation) * Z[i]);

    return output;
}

std::vector<double> calculate_angles(std::vector<double> point) {
    double x_circ = std::sqrt(segment_length * segment_length - point[1] * point[1]);
    std::vector<double> angles = {-2 * std::atan2(point[2] - x_circ, point[0]), -2 * std::atan2(x_circ, point[1] + segment_length)};

    return angles;
}

std::vector<double> mult(std::vector<std::vector<double>> mat, std::vector<double> v) {
    std::vector<double> output = {0, 0, 0};

    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) 
            output[i] += mat[i][j] * v[j];

    return output;
}

std::vector<double> calculate_target_angles(std::vector<double> point, double orientation, bool radians = true) {
    std::vector<double> angles = {};
    std::vector<double> segment1_target = parameterized_intersection(point, orientation);
    std::vector<double> joint1_angles = calculate_angles(segment1_target);

    std::vector<double> w = {point[0] - segment1_target[0], point[1] - segment1_target[1], point[2] - segment1_target[2]};

    std::vector<double> row1 = {std::cos(joint1_angles[0]), 0, -std::sin(joint1_angles[0])};
    std::vector<double> row2 = {-std::sin(joint1_angles[0]) * std::sin(joint1_angles[1]), std::cos(joint1_angles[1]), -std::cos(joint1_angles[0]) * std::sin(joint1_angles[1])};
    std::vector<double> row3 = {std::sin(joint1_angles[0]) * std::cos(joint1_angles[1]), std::sin(joint1_angles[1]), std::cos(joint1_angles[0]) * std::cos(joint1_angles[1])};
    std::vector<std::vector<double>> transform_inverse = {row1, row2, row3};

    std::vector<double> W = mult(transform_inverse, w);
    std::vector<double> joint2_angles = calculate_angles(W);

    angles.push_back(fix_angle(joint1_angles[0]));
    angles.push_back(fix_angle(joint1_angles[1]));
    angles.push_back(fix_angle(joint2_angles[0]));
    angles.push_back(fix_angle(joint2_angles[1]));

    if (!radians)
        for (int i = 0; i < 4; ++i)
            angles[i] = convert(angles[i], false);
    
    return angles;
}

bool check_angle_bounds(std::vector<double> angles) {
    for (int i = 0; i < 3; ++i) 
        if (std::fabs(angles[i + 1]) > PI / 2)
            return false;

    return true;
}

bool move_to_point(std::vector<double> point) {
    std::vector<double> angles = calculate_target_angles(point, 0);
    bool success = check_angle_bounds(angles);

    if (!success) {
        for (int i = 0; i < 72; ++i) {
            angles = calculate_target_angles(point, i * PI / 36);
            if (check_angle_bounds(angles)) {
                success = true;
                break;
            }
        }
    }


    if (success) {
        move_segment(0, {angles[0], angles[1]}, true);
        move_segment(1, {angles[2], angles[3]}, true);
        
        std::cout << "Moving to point (" << point[0] << ", " << point[1] << ", " << point[2] << ") with angles [" << angles[0] << ", " << angles[1] << ", " << angles[2] << ", " << angles[3] << "]" << std::endl;
        return true;
    } else {
        std::cout << "Failed to find valid angles" << std::endl;
        return false;
    }
}

int main() {
    move_to_point({1, 1, 1});
    move_to_point({0, 1, 1});
    move_to_point({-1.5, 0.25, 1});
    move_to_point({-1, 0.5, -1});
    move_to_point({0.25, 0, 1.5});
    move_to_point({0, 2, 0});
    return 0;
}