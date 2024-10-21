// MPU6050.h

#ifndef KalmanFilter_h

#define KalmanFilter_h


// ---------------- LIBRARIES --------------
#include <Arduino.h>
#include <math.h>

// ---------------- DATA TYPES --------------


// ---------------- CLASS DEFINITION --------------
class KalmanFilter {
private:
  // CONSTANTS
  #define N 12              // State size
  #define U 4               // Input size
  #define M 12              // Measurements size

  const float g = 9.81;     // Gravity acceleration

  // VARIABLES

public:
  // VARIABLES
  float x[N];               // System state vector
  float u[U];               // Inputs vector
  float z[M];               // Measurements vector

  float A[N][N];            // System state matrix
  float B[N][U];            // System input matrix
  float P[N][N];            // State covariance matrix
  float Q[N][N];            // Process noise covariance matrix
  float H[M][N];            // Observability matrix
  float R[M][M];            // Measurement noise covariance matrix
  float K[N][M];            // Kalman gain


  // FUNCTIONS
  KalmanFilter();
  void init();
  void prediction();
  void update();

};

#endif