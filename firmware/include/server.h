#pragma once
#include <vector> 
#include <ESP32Servo.h>  

/*
extern const std::vector<std::vector<double>> square_path;
extern const std::vector<std::vector<double>> spin_path;
extern const std::vector<std::vector<double>> pitch_sweep_path;
extern const std::vector<std::vector<double>>* current_path;
*/

extern bool manual_mode;  

void init_server();
void server_setup();
void server_loop();