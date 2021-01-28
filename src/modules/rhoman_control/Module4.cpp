// Company/Ownership - Rhoman Aerospace
// Title - AC Algorithm - Module 4 - Lower Level Controller for an N-Copter
// Author - Created by Anant Koul for Rhoman Aerospace on 6/18/2020.
// This module of the code is responsible for thrust calculations, which are directly fed to actuator controls.

#include "Module4.hpp"
#include "VariableDefinitions.h"
#include "math.h"
#include <iostream>

#include "../../lib/eigen/eigen-master/Eigen/SVD"
#include "../../lib/eigen/eigen-master/Eigen/Dense"
#include "../../lib/eigen/eigen-master/Eigen/LU"
#include "../../lib/eigen/eigen-master/Eigen/QR"

using namespace std;
using namespace Eigen;


// add comment for how to do this. 

// https://gist.github.com/pshriwise/67c2ae78e5db3831da38390a8b2a209f

// method for calculating the pseudo-Inverse as recommended by Eigen developers
template<typename _Matrix_Type_>
_Matrix_Type_ pseudoInverse(const _Matrix_Type_ &a, double epsilon = std::numeric_limits<double>::epsilon())
{

  Eigen::JacobiSVD< _Matrix_Type_ > svd(a ,Eigen::ComputeFullU | Eigen::ComputeFullV);
        // For a non-square matrix
        // Eigen::JacobiSVD< _Matrix_Type_ > svd(a ,Eigen::ComputeThinU | Eigen::ComputeThinV);
  double tolerance = epsilon * std::max(a.cols(), a.rows()) *svd.singularValues().array().abs()(0);
  return svd.matrixV() *  (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal() * svd.matrixU().adjoint();
}



void module4() {

    //VectorXf thrusts(Nmotors);
    
    // Determining the number of extra constraint equations.

    int row_count = 0;

    for (int i = 0; i < Nmotors; i++) {
        if (motors[i].weighting != 4) {
            row_count += 1;
        }
    }

    

    //cout<<"NMotors are: "<<Nmotors<<endl;

    /* row_count will also have a portion to account for non motor-specific constraint equations from ML algorithm.
     * These equations are linear and could therefore be stored within a matrix. This addition would require
     * something like the line: row_count=row_count+length(additionalequations) */

    /* Rikita - Defining Constraint Matrix A[][Nmotors] and forcing term b and initializing all elements to 0.
     * I am using 15 as a placeholder as that provides enough space (Not that concerned with overflow right now as
     * the for loop will constrain it to relevant values) but since its going to be
     * in C++, might make sense to do cleaner dynamic arrays specially depending on what the pinv math func requires.
     */
    //MatrixXf A(15, Nmotors);
    //VectorXf b(15);
    Matrix4d A;
    Vector4d b;
    /* Fill the controller constraint equations (rows 1-4) accordingly. Note that variables of the form {motors{j}.?}
     * reflect property ? of motor object j that can vary between motor. There are Nmotor motor objects on the vehicle.
     * Other variables are properties of the vehicle as a whole.
        */
    int row_index = 1;
    for (int i = 0; i < Nmotors; i++) {
        A(0, i) = -(1 / icpitch) * motors[i].x;
        A(1, i) = -(1 / icroll) * motors[i].y;
        A(2, i) = -(motors[i].irotor * cos(pitchm) * motors[i].dir / (icyaw * motors[i].kfrotor)) - (sin(pitchm) / icroll) * motors[i].y;
        A(3, i) = 1 / m; // This needs to be 1 / m. Right now g is a place holder. Confirm 'm' with James.

        // Add individual motor-specific constraint equations, if the exist.
        if (motors[i].weighting != 0) {
            A(3 + row_index, i) = motors[i].weighting;
            b(3 + row_index) = motors[i].presetval;
            row_index += 1;
        }
    }


    PX4_INFO("rolldes: %f  kproll: %f", rolldes, kproll);


    /* Portion for additional constraint equations goes here once we have figured out what that looks like.
     * These must be added to A and b at the ending rows. Ie, if we had three additional constraint equations and one
     * motor-specific constraint equation on a 6 motor craft, then Fthe three additional equations would form the last
     * three rows of A and b, which would be a 8x5 matrix and 8x1 vector, respectively. */



    // Fill in remaining values of forcing term. (rows 1-4)
    // k_() are tuning constants from module 3, currently in module 2.

    b(0) = kppitch * (pitchdes - pitchm) - kdpitch * pitchdotm;
    b(1) = kproll * (rolldes - rollm) - kdroll * rolldotm;
    b(2) = -kpyaw * yawmove + kdyaw * yawabsdotm;
    b(3) = (kpz * (zdes - zm) - kdz * zdotm) / (cos(pitchm) * cos(rollm) ) + g/(cos(pitchm) * cos(rollm));

    // Ritika - Need to implement the pinv function below to calc thrusts = pinv(A' * A) * (A' * b)
    // If A is L x Nmotors and b is L x 1 or simply L, then thrusts will be of dimensions Nmotors x 1.


    // cout<<A<<endl;+_
    // cout<<b<<endl;

    // thrusts = ((A.transpose()*A).inverse())*(A.transpose()*b);
    
    Matrix4d C = A.transpose()*A;

    

    // cout<< C << endl << endl;


    Matrix4d D = pseudoInverse(C);

    // cout<< D << endl << endl;

    // Matrix4d D = C.completeOrthogonalDecomposition().pseudoInverse();

    Vector4d E = (A.transpose()*b);
    thrusts_r = D * E;


    // cout<<thrusts_r(0)<<"   "<<thrusts_r(1)<<"   "<<thrusts_r(2)<<"   "<<thrusts_r(3)<<endl;
    cout<< "==========================" << endl;
    // Look up matlab pseudo-inverse (pdotinv)

    // do these thrust values need to be assigned to motors[i].thrustdes parameter?
    // cout<<"new"<<endl;
    // cout<<thrusts<<endl;
    //cout<<b<<endl;
}