typedef struct {
  double q; //process noise covariance
  double r; //measurement noise covariance
  float x; //value
  double p; //estimation error covariance
  double k; //kalman gain
} kalman_state;
