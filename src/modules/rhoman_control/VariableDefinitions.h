// Company/Ownership - Rhoman Aerospace
// Title - AC Algorithm - Header file for Variable declarations.
// Author - Created by Anant Koul for Rhoman Aerospace on 6/16/2020.

#ifndef RHOMANAEROSPACE_VARIABLEDEFINITIONS_H
#define RHOMANAEROSPACE_VARIABLEDEFINITIONS_H

#include <stdio.h>



// Module 0 Variables
double dcgx, dcgy, dcgz, icpitch, icroll, icyaw, g;
double *angts, *locts, *altts;
double anghz, angt, angdt, accelu, angu, dangu, lochz, locdt, loct, locu, althz, altdt, altu, altt, daltu;
int Nmotors, kalmanvals;
double *yawabsms, *pitchms, *rollms, *pitchdotms, *rolldotms, *yawabsdotms, *xms, *yms, *zms, *zdotms;

// Structure holding objects (properties) of motors

typedef struct {
    double ang, x, y, z, xg, yg, zg, size, Tmax, Tmin, irotor, kfrotor, kq, t, thrustdes, presetval, omega, alpha, eta, omegas[2];
    int dir, weighting;
} craftmotors;

craftmotors motors[6];

// Module 1 Variables
double times;
int kalman, kalmano;
double yawabsm, pitchm, rollm, pitchdotm, rolldotm, yawabsdotm, xm, ym, zm, zdotm;
double yawabsm1, pitchm1, rollm1, pitchdotm1, rolldotm1, yawabsdotm1, xm1, ym1, zm1, zdotm1;

// Module 2 Variables
double xdes, ydes, zdes, dt;
double veh_lat, veh_lon, yaw_des2;
float veh_alt;
double distancetotarget, distancetotargetprev, yawdes, yawmove, rolldes, pitchdes;
int kpdtt, kpdttdot, minpitchangle, maxpitchangle;
float x_float, y_float, z_float;

// Module 3 Variables
double kppitch, kdpitch, kproll, kdroll, kpyaw, kdyaw, kpz, kdz;


#endif //RHOMANAEROSPACEMODULES_VARIABLEDEFINITIONS_H
