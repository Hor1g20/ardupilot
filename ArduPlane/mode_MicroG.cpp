#include "mode.h"
#include "Plane.h"
#include <AP_HAL/AP_HAL.h>

float init_airspeed_y;
float init_airspeed_x;
float des_airspeed_y;
float des_airspeed_x;
float init_time;

float RPM_a = 0.00001043038075; // Quadratic equation a term

bool ModeMicroG::_enter()
{
    //plane.next_WP_loc.alt = plane.current_loc.alt;

    float init_airspeed = AP::airspeed()->get_airspeed(); // Airspeed at mode start

    float init_pitch = (plane.nav_pitch_cd/100)*M_PI/100; // Pitch angle at mode start
    init_airspeed_y = init_airspeed*sinf(init_pitch); // y velocity at mode start
    init_airspeed_x = init_airspeed*cosf(init_pitch); // x velocity at mode start

    init_time = millis(); // Start time

    return true;
}

void ModeMicroG::update()
{
    plane.nav_roll_cd  = 0; // Ensure aircraft remains level

    float t = (millis()-init_time)/1000; // Current time of mode run in seconds

    float current_airspeed = AP::airspeed()->get_airspeed(); // Find current airspeed

    des_airspeed_y = init_airspeed_y - (9.81*t); // Ballistic trajectory equation for y velocity
    des_airspeed_x = init_airspeed_x; // x velocity doesn't change

    // Pitch control

    float des_FPA = atan2f(des_airspeed_y,des_airspeed_x); // Calculate desired FPA

    float current_aoa = ahrs.getAOA()*(M_PI/180); // Find an estimate of the current AOA, in radians

    plane.nav_pitch_cd = ((des_FPA+current_aoa)*180/M_PI)*100; // In hundreths of a degree

    // Throttle Control

    float current_drag = 0.5*1.225*(current_airspeed*current_airspeed)*0.488*0.040; // Drag estimate

    float T_coeff = -0.01714*(current_airspeed*current_airspeed) + 0.7971*current_airspeed - 6.7571; // T_coeff fit

    float RPM_b = -0.00322961*current_airspeed; // Quadratic equation b term
    float RPM_c = -((current_drag/T_coeff)-2)/((1.225*((M_PI*((0.0254*12)*(0.0254*12)))/4))*0.3297587892); // Quadratic equation c term

    float desired_RPM = (-RPM_b + sqrtf((RPM_b*RPM_b) - 4*RPM_a*RPM_c))/(2*RPM_a); // Quadratic equation (+/largest term only)

    float throttle_setting = 0.000000505779267359677*(desired_RPM*desired_RPM) + 0.00550006712336856*desired_RPM + 2.95421502145744; // Throttle setting fit, given desired RPM

    SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, throttle_setting); // Set throttle to desired setting

    plane.update_load_factor(); // For autopilot codes
    //hal.console->printf("Time: %f\n",t);
    //hal.console->printf("Throttle: %f\n",throttle_setting);
}
