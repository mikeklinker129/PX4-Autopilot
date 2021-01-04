// Company/Ownership - Rhoman Aerospace
// Title - AC Algorithm - Module 2 - Angle determination.
// Author - Created by Anant Koul for Rhoman Aerospace on 6/16/2020.
// This module of the code is responsible for determine the pitch, roll, and yaw angles necessary to move the craft
// towards a target position.
// Second module inside of control loop.

#include "Module2.hpp"
#include <lib/ecl/geo/geo.h>
#include <math.h>
#include "VariableDefinitions.h"
#include <matrix/math.hpp>

using matrix::wrap_2pi;


#include <iostream>

using namespace std;

void module2(const struct position_setpoint_triplet_s *pos_sp_triplet) {

    // Cycle time value of the control logic loop - measured from within the control chip.
    // Place holder value.
    dt = 1.0;

    // Determining desired position. Can be either an interpreted value from the controller
    // or pulled directly from a timestamped or geo-stamped mission profile.
    // Place holder value.

    if (globallocalconverter_initialized()){
        globallocalconverter_tolocal(pos_sp_triplet->current.lat, pos_sp_triplet->current.lon, pos_sp_triplet->current.alt, &x_float, &y_float, &z_float);
        // globallocalconverter_toglobal(xm, ym, zm,   &veh_lat, &veh_lon, &veh_alt);
        xdes = (double)x_float;
        ydes = (double)y_float;
        zdes = (double)z_float;

        // Dont use this. It is mode dependent. 
        //distancetotarget = (double) get_distance_to_next_waypoint( veh_lat, veh_lon,  pos_sp_triplet->current.lat, pos_sp_triplet->current.lon);
    
        if (!PX4_ISFINITE(yawdes)){
            yawdes = 0.0;
        }

    } else{
        xdes = xm;
        ydes = ym;
        zdes = zm;
        yawdes = 0.0;
    }

    //https://px4.github.io/Firmware-Doxygen/db/d35/geo_8h.html#a189032e49591f0ce0e1a6049e8446262
        
    // Convert desired values from NED to ENU frame.

    double temp_x_des = xdes;
    double temp_y_des = ydes;

    xdes = temp_y_des;
    ydes = temp_x_des;
    zdes = -zdes; 

    xdes =  2.0;
    ydes = 50.0;
    zdes = 5.1;


    // Storing old distance to target value.
    distancetotargetprev = distancetotarget; // [m]

    if (PX4_ISFINITE(xdes)){

        distancetotarget = sqrtf( (xdes-xm)*(xdes-xm) + (ydes-ym)*(ydes-ym) );

        if (distancetotarget > 1.0){
            yawdes = atan2(ydes-ym, xdes-xm);
        } else {
            yawdes = yawabsm;
        }
        
    } else{
        distancetotarget = 0.0;
        yawdes = yawabsm;
        xdes = xm;
        ydes = ym;
        zdes = zm;

    }

    // This sections determines the angles necessary to reach the desired position relative to the current position
    // and yaw. (xm,ym,zm,yawabs)
    // yawdes = 0.0;
    // yawabsm = 0;
    // pitchm = 0;
    // rollm = 0;
    // pitchdotm = 0;
    // rolldotm = 0;
    // yawabsdotm = 0;
    // xm = 0;
    // ym = 0;
    // zm = 2;
    // zdotm = 0;
    // distancetotarget = 0.0;

    PX4_INFO("Yaw: %f Pitch: %f  Roll: %f pitchdot: %f  rolldot: %f  yawabsdot: %f", yawabsm*180/M_PI, pitchm, rollm, pitchdotm, rolldotm, yawabsdotm );
    PX4_INFO("xm: %f  ym: %f  zm: %f  zdot: %f", xm, ym, zm, zdotm );
    PX4_INFO("xdes: %f  ydes: %f  zdes: %f  yawdes: %f", xdes, ydes, zdes, yawdes*180/M_PI);
    PX4_INFO("distancetotarget: %f", distancetotarget);



    // Calculating new distance to target.
    // distancetotarget = pow(sqrtf((xdes - xm)), 2) + pow((ydes - ym), 2); // [m]

    // Determining absolute yaw angles (yawabs, yawdes).
    // Range is constrained to +/- pi.
    // yawdes = atan2((ydes - ym), (xdes - xm)) - (M_PI / 2 );
    if (yawdes > M_PI) {
        yawdes -= 2 * M_PI;
    }
    else if(yawdes < (-M_PI)) {
        yawdes += 2 * M_PI;
    }

    if (yawdes > (M_PI / 2)) {
        if (yawabsm < (-M_PI / 2)) {
            yawdes -= 2 * M_PI;
        }
    }
    else if (yawdes < (-M_PI / 2)) {
        if (yawabsm > (M_PI / 2)) {
            yawdes += 2 * M_PI;
        }
    }

    // Determining difference between two angles and computing yawmove.
    // yawmove represents the shortest path between the two angles.
    // Range is constrained to +/- pi.
    if ((yawdes - yawabsm) > M_PI) {
        yawmove = yawdes - yawabsm - (2 * M_PI);
    }
    else if ((yawdes - yawabsm) < (-M_PI)) {
        yawmove = yawdes - yawabsm + (2 * M_PI);
    }
    else {
        yawmove = yawdes - yawabsm;
    }

    // Determining desired pitch and roll angles in radians
    // See detailed control documentation for what part is actually doing.
    kpdtt = 1;            // Default value of tuning constant.
    kpdttdot = 15;        // Default value of tuning constant.

    // These variables needs to be changed into a function.
    minpitchangle = -20;  // Function that determines a minimum allowable safe pitch angle TBD. -30 is a safe value. -70 is risky [degrees]
    maxpitchangle = 20;   // Function that determines a maximum allowable safe pitch angle TBD. 30 is a safe value. 70 is risky [degrees]
    pitchdes = copysign(1.0,cos(yawmove)) * fminl (fminl (fmaxl(kpdtt * distancetotarget + kpdttdot * (distancetotarget - distancetotargetprev) / dt, minpitchangle) , maxpitchangle), 180 * pitchm / 3.1415 + 100) * (3.1415 / 180 ) * fabs(cos(yawmove));

    // Checking whether we are pointing approximately at target, if yes, setting roll to zero.
    if (fabs((yawabsm - yawdes)) < (5 * 3.1415 / 180)) {
        rolldes = 0;
    }
    else {
        if (pitchm > 0) {
            if (pitchm < (3 * 3.1415 / 180)) {
                // Set roll to zero if the drone is hovering nearly flat.
                rolldes = 0;
            }
            else {
                // Allow rolls up to +/-10 degrees if the drone is pitched towards the target.
                rolldes = fminl(fmaxl(-yawdes + yawabsm, -10 * M_PI / 180), 10 * M_PI / 180);
            }
        }
        else {
            // No roll if pitched away from target.
            rolldes = 0;
        }
    }


    // A smooth takeoff can be guaranteed by pulling a preset desired path for the first two seconds.
    // This is done via desired angle override after all other calculation.
    if (times < 1) {
        pitchdes = 0;
        rolldes = 0;
        yawmove = 0;
    }
    // Fade in the actual desired angles after the warm-up period.
    else if (times < 3) {
        pitchdes = pitchdes * (times - 1) / 2;
        rolldes = rolldes * (times - 1) / 2;
        yawmove = yawmove * (times - 1) / 2;
    }

    // Adjusting the motors actual relative position for how the real drone CG has shifted versus the
    // initial input CG values. First calculate new actual CG shift. (TBD)
    // Place holder values.
    dcgx = 0.0;   // Real CG shift in X versus nominal position. [m]
    dcgy = 0.0;   // Real CG shift in Y versus nominal position. [m]
    dcgz = 0.0;   // Real CG shift in Z versus nominal position. [m]

    // Adjusting motor parameters accordingly to get the final x,y,z values needed for the lower level controller.
    for (int i = 0; i < Nmotors; i++) {
        motors[i].x = motors[i].xg-dcgx;
        motors[i].y = motors[i].yg-dcgy;
        motors[i].z = motors[i].zg-dcgz;



    }

     PX4_INFO("pitchdes: %f   rolldes: %f  yawmove: %f", pitchdes, rolldes, yawmove);


    // Module 3
    // Module 3 - Determining tuning constants.
    // Set tuning constants for lower level controller. These are some possible values.
    // Their absolute values are not particularly important, but their relative scaling's are.
    // THESE WILL BE DRONE SPECIFIC. Exactly where we are pulling them from is TBD, and will be Module 3.

    // kppitch=180;
    // kdpitch=160;
    // kproll=180;
    // kdroll=160;
    // kpyaw=1600;
    // kdyaw=800;
    // kpz=2;
    // kdz=4;

    kppitch=6; //2
    kdpitch=10; //20
    kproll=6;
    kdroll=10;
    kpyaw=1000;
    kdyaw=800;
    kpz=10;
    kdz=5;

}