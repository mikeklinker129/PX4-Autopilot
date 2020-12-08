// Company/Ownership - Rhoman Aerospace
// Title - Linear Kalman Filter Implementation
// Author - Created by Anant Koul for Rhoman Aerospace on 6/8/2020.

#ifndef RHOMANAEROSPACEMODULES_KALMANLINEARFILTER_H
#define RHOMANAEROSPACEMODULES_KALMANLINEARFILTER_H

#include<math.h>

// Function to multiply two matrices with dimensions (m1 x m2) and (n1 x n2)
/* void matrix_multiply(int m1, int m2, int n1, int n2, double mat1[][m2], double mat2[][n2], double r_matrix[m1][n2]) {

    for (int i = 0; i < m1; i++) {
        for (int j = 0; j < n2; j++) {
            r_matrix[i][j] = 0;
            for (int k = 0; k < m2; k++) {
                *(*(r_matrix + i) + j) += *(*(mat1 + i) + k) * *(*(mat2 + k) + j);
            }
        }
    }*/
/*
    for (i = 0; i < m1; i++)
    {
        for (j = 0; j < n2; j++)
        {
            printf("%f ", *(*(r_matrix + i) + j));
        }
        printf("\n");
    }
*/
// }

//  Linear Kalman Filter function
double kalmanlinearfilter(const double times_in[], const double values_in[], double evaluation_time, int order, int VDIM) {

    double A_mat[VDIM][order + 1];
    double A_mat_transpose[order + 1][VDIM];
    double b_mat[VDIM];
    double x_term[order + 1][order + 1];
    double y_term[order + 1][order + 1];
    double z_term[order + 1][VDIM];
    double coefficients[order + 1];

    // Initializing A Matrix and filling all elements
    for (int i = 0; i < VDIM; i++) {
        for (int j = 0; j < order + 1; j++) {
           A_mat[i][j] = pow(times_in[i], j);
        }
    }

    // Initializing B Matrix - This is a redundant step and should be removed
    for (int i = 0; i < VDIM; i++) {
        b_mat[i] = values_in[i];
        }

    // Computing transpose of A_matrix, similar to .' operator in Matlab
    for (int i = 0; i < VDIM; i++) {
        for (int j = 0; j < order + 1; j++) {
            A_mat_transpose[j][i] = A_mat[i][j];
        }
    }

    // Code for inverse calculations in C, similar to inv() Matlab
    // coefficients = inv(A_mat'*A_mat)*A_mat'*b_mat;
    // Nomenclature ->
    // x_term = A_mat'*A_mat
    // y_term = inv(A_mat'*A_mat)
    // z_term = inv(A_mat'*A_mat)*A_mat'

    // Calculating x_term
    // matrix_multiply(order + 1, VDIM, VDIM, order + 1, A_mat_transpose, A_mat, x_term);
    for (int i = 0; i < order + 1; i++) {
        for (int j = 0; j < order + 1; j++) {
            x_term[i][j] = 0;
            for (int k = 0; k < VDIM; k++) {
                *(*(x_term + i) + j) += *(*(A_mat_transpose + i) + k) * *(*(A_mat + k) + j);
            }
        }
    }

    // Calculating y_term
    // Matrix_inverse Implementation - Github code has been modified as required.
    // Final implementation will be rewritten within a header file MatrixInverse.h

    double matrix[10][10], ratio,a;
    int n;
    n = order + 1;

    for(int i = 0; i < n; i++) {
        for(int j = 0; j < n; j++) {
            matrix[i][j] = x_term[i][j];
        }
    }
    for(int i = 0; i < n; i++) {
        for(int j = n; j < 2*n; j++) {
            if(i==(j-n))
                matrix[i][j] = 1.0;
            else
                matrix[i][j] = 0.0;
        }
    }
    for(int i = 0; i < n; i++) {
        for(int j = 0; j < n; j++) {
            if(i!=j) {
                ratio = matrix[j][i]/matrix[i][i];
                for(int k = 0; k < 2*n; k++) {
                    matrix[j][k] -= ratio * matrix[i][k];
                }
            }
        }
    }
    for(int i = 0; i < n; i++) {
        a = matrix[i][i];
        for(int j = 0; j < 2*n; j++) {
            matrix[i][j] /= a;
        }
    }

    for(int i = 0; i < n; i++) {
        for(int j = 0; j < n; j++) {
            y_term[i][j] = matrix[i][n + j];
        }
    }

    // Calculating z_term
    // matrix_multiply(order + 1, order + 1, order + 1, VDIM, y_term, A_mat_transpose, z_term);
    for (int i = 0; i < order + 1; i++) {
        for (int j = 0; j < VDIM; j++) {
            z_term[i][j] = 0;
            for (int k = 0; k < order + 1; k++) {
                *(*(z_term + i) + j) += *(*(y_term + i) + k) * *(*(A_mat_transpose + k) + j);
            }
        }
    }

    // Calculating Coefficients
    double sum = 0;
    for (int i = 0; i < order + 1; i++) {
        for (int j = 0; j < 1; j++) {
            for (int k = 0; k < VDIM; k++) {
                sum = sum + z_term[i][k] * b_mat[k];
            }
            coefficients[i] = sum;
            sum = 0;
            }
        }

    // Calculating Predictive Value
    double predicted_value = 0;
    for (int i = 0; i < order + 1; i++) {
        predicted_value += coefficients[i] * pow(evaluation_time, i);
    }

    return predicted_value;
}

#endif //RHOMANAEROSPACEMODULES_KALMANLINEARFILTER_H
