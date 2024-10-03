/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: December 11, 2020
 *      Author: Mathilde Badoual
 **********************************************/

#include "pid_controller.h"
#include <iostream>
#include <math.h>

using namespace std;

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kpi, double Kii, double Kdi, double output_lim_maxi, double output_lim_mini) {
    Kp = Kpi;
    Kd = Kdi;
    Ki = Kii;
    output_lim_min = output_lim_mini;
    output_lim_max = output_lim_maxi;
}


void PID::UpdateError(double ctei) {
    cte = ctei;
    diffCte = dt > 0.0 ? (cte - prevCte)/dt : 0.0;
    intCte += cte*dt;
    prevCte = cte;
}

double PID::TotalError() {
    // Calculate and return the total error
    auto control = -Kp * cte - Kd * diffCte - Ki * intCte;
    return min(max(control, output_lim_min), output_lim_max);
}

double PID::UpdateDeltaTime(double new_delta_time) {
    // Update the delta time with new value
    dt = new_delta_time;
    return dt;
}