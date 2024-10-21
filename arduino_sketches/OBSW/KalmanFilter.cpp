#include "KalmanFilter.h"
#include <math.h>

// -------------- CONSTRUCTOR --------------
KalmanFilter::KalmanFilter(){
};

// -------------- INITIALIZATION --------------
void KalmanFilter::init() {

  // Build matrices of type: N x ...
  for (int i = 0; i < N; i++) {

    x[i] = 0.0;

    for (int j = 0; j < N; j++) {
      A[i][j] = 0.0;
      P[i][j] = 0.0;
      Q[i][j] = 0.0;
    }

    for (int j = 0; j < U; j++) {
      B[i][j] = 0.0;
    }

    for (int j = 0; j < M; j++) {
      R[i][j] = 0.0;
      K[i][j] = 0.0;
    }
  }

  // Build matrices of type: M x ...
  for (int i = 0; i < M; i++) {

    z[i] = 0.0;

    for (int j = 0; j < M; j++) {
      R[i][j] = 0.0;
    }

    for (int j = 0; j < N; j++) {
      H[i][j] = 0.0;
    }
  }

  // Build matrices of type U x ...
  for (int i = 0; i < U; i++) {
    u[i] = 0.0;
  }

  float psi_des = 0.0;

  // Compute A 
  A[1][4] = 1.0;
  A[2][5] = 1.0;
  A[3][6] = 1.0;
  A[4][7] = g * sin(psi_des);
  A[4][8] = g * cos(psi_des);
  A[5][7] = - g * cos(psi_des);
  A[5][8] = g * sin(psi_des);
  A[7][10] = 1.0;
  A[8][11] = 1.0;
  A[9][12] = 1.0;

  // Compute B
  B[6][1] = 1.0;


  
}
