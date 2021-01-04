// Company/Ownership - Rhoman Aerospace
// Title - AC Algorithm - Module 1 - State import and Kalman filter.
// Author - Created by Anant Koul for Rhoman Aerospace on 6/16/2020.
// This module of the code is responsible for importing all necessary state information and preparing it for use.
// First module inside of control loop.

#include <math.h>
#include "Module1.hpp"
#include "VariableDefinitions.h"
#include "KalmanLinearFilter.h"
#include <lib/ecl/geo/geo.h>

#include <matrix/math.hpp>

using matrix::wrap_2pi;
using matrix::wrap_pi;


// Function to locate the minimum value of an array.
int min_value(double array[]) {

    double minimum = array[0];
   // int location = 0;

    for (int i = 1; i < kalmanvals; i++) {
        if (array[i] < minimum) {
            minimum = array[i];
           // location = i+1;
        }
    }
    return minimum;
}




void module1(   const struct vehicle_attitude_s *att, 
                const struct vehicle_angular_velocity_s *ang_vel,
                const struct vehicle_local_position_s *local_pos) {

    double evaluation_time = 0.0;
    double min_val;
    double times_in[kalmanvals];


    matrix::Eulerf att_euler = matrix::Quatf(att->q);

    // double val = att_euler.theta();
    


    // First read in new sensor data if available. This is currently done by checking if enough time has passed
    // since the last check for each sensor. If it has, then read the new value and update the storage arrays defined
    // in module 0 appropriately.
    // As an example: If xms and locts were 1x10 originally and had values as [3 4 5 6 7 3 4 5 6 7] and
    // [.1 .2 .3 .4 .5 .6 .7 .8 .9 .10] and the new x reading was now 8 at 1.1 seconds, then xms now
    // becomes [4 5 6 7 3 4 5 6 7 8] and locts would be [.2 .3 .4 .5 .6 .7 .8 .9 .10 .11].

    // Reading the current system time. [seconds] - Initialized with a place holder.

    times = hrt_absolute_time()/1000000.0; //remove start time


    // PX4_INFO("%f", times);

    // If statement to check whether we have reached the next IMU/compass update time.
    if (times > angt) {

        // New Values are read here. Initialized with place holder.
        yawabsm1 = att_euler.psi();     // Yaw reading goes here. [rad]
        pitchm1 = (double) att_euler.theta();// * M_PI/180.0;     // Pitch reading goes here. [rad]
        rollm1 =  (double) att_euler.phi();;// * M_PI/180.0;       // Roll reading goes here. [rad]
        pitchdotm1  = ang_vel->xyz[1];   // Pitch rate reading goes here. [rad/s]
        rolldotm1   = ang_vel->xyz[0];    // Roll rate reading goes here. [rad/s]
        yawabsdotm1 = ang_vel->xyz[2];  // Yaw rate reading goes here. [rad/s]

        // Updating Storage Arrays
        for (int i = 0; i < (kalmanvals - 1); i++) {
            yawabsms[i] = yawabsms[i+1];
            pitchms[i] = pitchms[i+1];
            rollms[i] = rollms[i+1];
            pitchdotms[i] = pitchdotms[i+1];
            rolldotms[i] = rolldotms[i+1];
            yawabsdotms[i] = yawabsdotms[i+1];
        }

        yawabsms[kalmanvals-1] = yawabsm1;
        pitchms[kalmanvals-1] = pitchm1;
        rollms[kalmanvals-1] = rollm1;
        pitchdotms[kalmanvals-1] = pitchdotm1;
        rolldotms[kalmanvals-1] = rolldotm1;
        yawabsdotms[kalmanvals-1] = yawabsdotm1;

        // Set new update time
        angt = angt + angdt;

     }

    // If statement to check whether we have reached the next GPS update time.
    if (times > loct) {

        // New Values are read here. Initialized with place holder.
        xm1 = local_pos->x;  // X reading goes here. [m]
        ym1 = local_pos->y;  // Y reading goes here. [m]

        // Updating Storage Arrays
        for (int i = 0; i < (kalmanvals - 1); i++) {
            xms[i] = xms[i+1];
            yms[i] = yms[i+1];
            locts[i] = locts[i+1];
        }

        xms[kalmanvals-1] = xm1;
        yms[kalmanvals-1] = ym1;
        locts[kalmanvals-1] = times;

        // Set new update time
        loct = loct + locdt;
    }

    // If statement to check whether we have reached the next altimeter update time.
    if (times > altt) {

        // New Values are read here. Initialized with place holder.
        zm1 = local_pos->z;    // Z reading goes here. [m]
        zdotm1 = local_pos->vz; // YZ rate reading goes here. [m/s]

        // Updating Storage Arrays
        for (int i = 0; i < (kalmanvals - 1); i++) {
            zms[i] = zms[i+1];
            zdotms[i] = zdotms[i+1];
            altts[i] = altts[i+1];
        }

        zms[kalmanvals-1] = zm1;
        zdotms[kalmanvals-1] = zdotm1;
        altts[kalmanvals-1] = times;

        // Set new update time
        altt = altt + altdt;
    }

    // Set to true (1) to use a Kalman filter. False (0) if no filter.
    kalman = 0;
    // Order of the polynomial to use in the Kalman filter. (2 by default)
    kalmano = 2;

    // If statement to check Kalman filter needs to be used.
    if (kalman == 1) {

        // IFitting a polynomial of order kalmano to the data from each set of timestamp-shifted sensor measurements,
        // and then evaluating at the current time, providing interpolation, smoothing, and extrapolation versus
        // raw measurements. Timestamp shifted means that the lowest value is subtracted to make the first value zero,
        // and all values near zero. This improves accuracy.

        min_val = min_value(angts);
        for (int i = 0; i < kalmanvals; i++) {
            times_in[i] = angts[i] - min_val;
        }
        evaluation_time = times - min_val;

        yawabsm = kalmanlinearfilter(times_in, yawabsms, evaluation_time, kalmano, kalmanvals);
        pitchm = kalmanlinearfilter(times_in, pitchms, evaluation_time, kalmano, kalmanvals);
        rollm = kalmanlinearfilter(times_in, rollms, evaluation_time, kalmano, kalmanvals);
        pitchdotm = kalmanlinearfilter(times_in, pitchdotms, evaluation_time, kalmano, kalmanvals);
        rolldotm = kalmanlinearfilter(times_in, rolldotms, evaluation_time, kalmano, kalmanvals);
        yawabsdotm = kalmanlinearfilter(times_in, yawabsdotms, evaluation_time, kalmano, kalmanvals);

        min_val = min_value(locts);
        for (int i = 0; i < kalmanvals; i++) {
            times_in[i] = locts[i] - min_val;
        }
        evaluation_time = times - min_val;

        xm = kalmanlinearfilter(times_in, xms, evaluation_time, kalmano, kalmanvals);
        ym = kalmanlinearfilter(times_in, yms, evaluation_time, kalmano, kalmanvals);

        min_val = min_value(altts);
        for (int i = 0; i < kalmanvals; i++) {
            times_in[i] = altts[i] - min_val;
        }
        evaluation_time = times - min_val;

        zm = kalmanlinearfilter(times_in, zms, evaluation_time, kalmano, kalmanvals);
        zdotm = kalmanlinearfilter(times_in, zdotms, evaluation_time, kalmano, kalmanvals);



    }

    // Else statement incase Kalman Filter is not used.
    else {

        yawabsm = yawabsm1;
        pitchm = pitchm1;
        rollm = rollm1;
        pitchdotm = pitchdotm1;
        rolldotm = rolldotm1;
        yawabsdotm = yawabsdotm1;
        xm = xm1;
        ym = ym1;
        zm = zm1;
        zdotm = zdotm1;

    }


    // Convert to ENU Convention
    // yawabsm = wrap_2pi(M_PI/2.0-yawabsm);
    yawabsm = wrap_2pi(yawabsm+2*M_PI);
    yawabsm = wrap_2pi(M_PI/2.0-yawabsm);
    yawabsm = wrap_pi(yawabsm);

    yawabsdotm = -yawabsdotm;
    
    pitchm = -pitchm;
    // rollm = rollm;

    pitchdotm = -pitchdotm;
    rolldotm = rolldotm;
    

    double old_x = xm;
    double old_y = ym;

    xm = old_y;
    ym = old_x;
    zm = -zm;

    zdotm = -zdotm;

        // yawabsm = yawabsm1;
        // pitchm = pitchm1;
        // rollm = rollm1;
        // pitchdotm = pitchdotm1;
        // rolldotm = rolldotm1;
        // yawabsdotm = yawabsdotm1;
        // xm = xm1;
        // ym = ym1;
        // zm = zm1;
        // zdotm = zdotm1;


    // PX4_INFO("Yaw: %f Pitch: %f  Roll: %f pitchdot: %f  rolldot: %f  yawabsdot: %f", yawabsm, pitchm, rollm, pitchdotm, rolldotm, yawabsdotm );
    // PX4_INFO("xm: %f  ym: %f  zm: %f  zdot: %f", xm, ym, zm, zdotm );


}