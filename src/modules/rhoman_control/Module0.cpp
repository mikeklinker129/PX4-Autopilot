// Company/Ownership - Rhoman Aerospace
// Title - AC Algorithm - Module 0 - Variable Initializations.
// Author - Created by Anant Koul for Rhoman Aerospace on 6/16/2020.
// This module of the code is responsible for initializing all necessary variables when the device is turned on.
// This section only needs to be run once at startup.

#include <stdlib.h>
#include <math.h>
#include "Module0.hpp"
#include "VariableDefinitions.h"



void module0() {

    // Initializing general vehicle properties - Initialized with place holder values.
    // These are values that need to be measured or otherwise known.
    // Not included the properties that are only used within the simulation environment,
    // as they would not be in the implemented version.
    // Note on coordinate frame: X is front of craft, Y is left of craft, Z is up.
    dcgx = 0.0; // CG deviation from geometric center in X. This will shift as payload does. [meters]
    dcgy = 0.0; // CG deviation from geometric center in Y. This will shift as payload does. [meters]
    dcgz = 0.0; // CG deviation from geometric center in Z. This will shift as payload does. [meters]
    icpitch = 36.9377; // Moment of inertia in pitch of craft+payload. [kg*meters^2]
    icroll = 36.9377; // Moment of inertia in roll of craft+payload. [kg*meters^2]
    icyaw = 37.4336;  // Moment of inertia in yaw of craft+payload. [kg*meters^2]
    g = 9.81;     // Gravitational Constant. [m/s^2]
    Nmotors = 4;  // How many motors does the craft actually have. [>=1]

    int dir = -1;

    double rnom = 0.925;
    double snom = 1.016;
    m = 114.8;
    double Tmax = 36.5*9.81;
    double Tmin=0;

    // double payloadheight = -24 * .0254; // Meters above center
    // double cmz = payloadheight / m;
    //icpitch = 15*((rnom/2)*(rnom/2)+cmz*cmz) + 4*10*(  pow(rnom/((double) sqrtf(2)),2)  +cmz*cmz) + 45.5*pow((payloadheight-cmz),2); // Contribution from payload
        

    //icroll=icpitch;  // Symmetric in roll/pitch
    //icyaw = 4*10*rnom*rnom+15*(rnom/2)*(rnom/2); // Estimate
    // Creating structure of size Nmotors.
    //craftmotors motors[Nmotors];

    // double i1mrotor = 0.3*pow((39.37/42),5); // Moment of inertia of a 1 m rotor; % Normally 0.3
    // double kf1mrotor = 0.00592*pow((39.37/42),4); // Kf for a typical 1 m rotor;

    // Initializing objects of motors. Initialized with place holder values.
    for (int i = 0; i < Nmotors; i++) {

        double pi = (double) M_PI_F;
        double angle=2*pi*(i+0.5)/Nmotors;

        motors[i].ang = angle; // Reference angle as defined above to help user understand where motor is. [radians]
        motors[i].xg = rnom*cos(angle);  // Geometric X of the center of rotor i. [meters]
        motors[i].yg = rnom*sin(angle);  // Geometric Y of the center of rotor i. [meters]
        motors[i].zg = 0.1;  // Geometric Z of the center of rotor i. [meters]
        motors[i].dir = dir; // Direction of rotation for positive thrust of rotor i. [+1 for positive about z, -1 for negative about z]
        
        motors[i].size = snom; // Prop diameter. [meters]
        motors[i].Tmax = Tmax; // Maximum thrust of prop. [Newtons]
        motors[i].Tmin = Tmin; // Minimum thrust of prop. Can be negative. [Newtons]
        motors[i].irotor = 0.2351; //i1mrotor*pow(snom,5); // Moment of inertia of rotor. [kg*meters^2]
        motors[i].kfrotor = 0.0049; //kf1mrotor*pow(snom,4); // Combined thrust coefficient of propeller. [N/(rad/s)]
        motors[i].kq = 0.007;   // Propeller torque constant. [N-m/N]

        motors[i].eta = 0.9;  // Estimated combined efficiency of ESC-motor combination. [0-1]
        motors[i].t = 0.0;    // Current thrust value estimate of rotor. Initialize to zero. [Newtons]
        motors[i].thrustdes = 0.0; // Current desired thrust value of rotor. Initialize to zero. [Newtons]
        motors[i].weighting = 0; // Set to 1 to add a constraint function specific to this motor. [1 or 0]
        motors[i].presetval = 0.0; // Target desired thrust in motor-specific constraint, if enabled. [Tmin->Tmax]
        motors[i].omega = 0.0; // Current motor speed. [radians/second]
        motors[i].alpha = 0.0; // Current rotor acceleration in positive z. [radians/second^2]
        motors[i].omegas[0] = 0; // Rotational speed tracking array. Initialize with two zero entries. [rad/s rad/s ...]
        motors[i].omegas[1] = 0; // Rotational speed tracking array. Initialize with two zero entries. [rad/s rad/s ...]


        dir = -dir;
       // printf("%d.....", i);
       // printf("%f     ", motors[i].irotor);
       // printf("%f     ", motors[i].kfrotor);
       // printf("%f     ", motors[i].ang);
       // printf("%f     ", motors[i].xg);
       // printf("%f     ", motors[i].yg); 
       // printf("\n");
    }

    // Initializing all measured state variables into the Kalman Filter.
    // Number of values to use within Kalman Filter for calculations.
    kalmanvals = 10;

    // Allocating memory location for orientation arrays of size 'kalmanvals'.
    // yawabsms -> Measured yaw value from compass. [rad]
    // pitchms -> Measured pitch value from IMU. [rad]
    // rollms -> Measured roll value from IMU. [rad]
    yawabsms = (double *)malloc(kalmanvals*sizeof(double));
    pitchms = (double *)malloc(kalmanvals*sizeof(double));
    rollms = (double *)malloc(kalmanvals*sizeof(double));

    // Allocating memory location for angular rate arrays of size 'kalmanvals'.
    // pitchdotms -> Measured pitch rate value from IMU. [rad/s]
    // rolldotms -> Measured roll rate value from IMU. [rad/s]
    // yawabsdotms -> Measured yaw rate value from IMU. [rad/s]
    pitchdotms = (double *)malloc(kalmanvals*sizeof(double));
    rolldotms = (double *)malloc(kalmanvals*sizeof(double));
    yawabsdotms = (double *)malloc(kalmanvals*sizeof(double));

    // Allocating memory location for position arrays of size 'kalmanvals'.
    // xms -> Measured x position from GPS (east=positive). [meters]
    // yms -> Measured y position from GPS (north=positive). [meters]
    // zms -> Measured altitude from barometer. [meters]
    xms = (double *)malloc(kalmanvals*sizeof(double));
    yms = (double *)malloc(kalmanvals*sizeof(double));
    zms = (double *)malloc(kalmanvals*sizeof(double));

    // Allocating memory location for vertical speed array of size 'kalmanvals'.
    // zdotms -> Measured vertical speed from barometer. [meters/s]
    zdotms = (double *)malloc(kalmanvals*sizeof(double));

    // Initializing orientation, angular rate, position and vertical speed array to zeros.
    for (int i = 0; i < kalmanvals; i++) {
        yawabsms[i] = 0.0;
        pitchms[i] = 0.0;
        rollms[i] = 0.0;
        pitchdotms[i] = 0.0;
        rolldotms[i] = 0.0;
        yawabsdotms[i] = 0.0;
        xms[i] = 0.0;
        yms[i] = 0.0;
        zms[i] = 0.0;
        zdotms[i] = 0.0;
    }

    //Allocating memory location for timing arrays of size 'kalmanvals'.
    angts = (double *)malloc(kalmanvals*sizeof(double));
    locts = (double *)malloc(kalmanvals*sizeof(double));
    altts = (double *)malloc(kalmanvals*sizeof(double));

    // Initializing timing arrays by a specific implementation of 'Linspace' from Matlab - Not a general implementation.
    // Timing arrays start at negative values and go to zero to enable a smooth startup.
    // angts -> Used to track measurement received time for IMU and compass measurements.
    // locts -> Used to track measurement received time for GPS measurements.
    // altts -> Used to track measurement received time for barometer measurements.
    for (int i = 0; i < kalmanvals; i++) {
        angts[i] = ((double)(i) / ((double)(kalmanvals) - 1)) * (- kalmanvals);
        locts[i] = ((double)(i) / ((double)(kalmanvals) - 1)) * (- kalmanvals);
        altts[i] = ((double)(i) / ((double)(kalmanvals) - 1)) * (- kalmanvals);
    }

    // Initializing Sensor properties - Current values are from Pixhawk 4 sensors.
    anghz = 200;            // Frequency at which IMU/compass generates new readings.
    angdt = 1/anghz;        // Time between new sensor angular measurements (IMU sensor dependent).
    angt = 0;               // Next Simulation time for new angular reading measurements. Initialize to zero.
    accelu = g / 8192;      // Minimum resolvable linear acceleration. [m/s^2]
    angu = accelu;          // Uncertainty in angular potions= D tan(accel1/accel2) ~ D Accel1. [rad]
    dangu = (0.014/sqrt(anghz)) * M_PI/180; // uncertainty in angular velocities=sqrt(2)*angu. [rad/s]
    lochz = 10;             // Frequency at which GPS generates new readings
    locdt = 1 / lochz;      // Time between new sensor x,y pos measurements (GPS sensor dependent)).
    loct = 0;               // Next Simulation time for new x,y pos reading measurements. Initialize to zero.
    locu = 0;               // Uncertainty in x,y pos measurements. [m]
    althz = 200;            // Frequency at which barometer generates new readings.
    altdt = 1 / althz;      // Time between new sensor z measurements (altitude sensor dependent)).
    altt = 0;               // Next Simulation time for new z reading measurements. Initialize to zero.
    altu = 0.1;             // Uncertainty in z pos measurements. [m]
    daltu = sqrt(2) * altu / 2; // Uncertainty in z speed measurements. [m/s]

    // Initializing desired variables to current measured position - Currently has placeholder values.
    xdes = 0.0; // [m]
    ydes = 0.0; // [m]
    zdes = 0.0; // [m]

    // Do not free memory here - variables utilized in other modules
    //free(x); free(y);

}














