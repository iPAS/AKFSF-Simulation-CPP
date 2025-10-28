// ------------------------------------------------------------------------------- //
// Advanced Kalman Filtering and Sensor Fusion Course - Linear Kalman Filter
//
// ####### STUDENT FILE #######
//
// Usage:
// -Rename this file to "kalmanfilter.cpp" if you want to use this code.

#include "kalmanfilter.h"
#include "utils.h"

// -------------------------------------------------- //
// YOU CAN USE AND MODIFY THESE CONSTANTS HERE
constexpr bool INIT_ON_FIRST_PREDICTION = false;  // true
constexpr double INIT_POS_STD = 0;  // 5
constexpr double INIT_VEL_STD = 0;  // 5./3.
constexpr double ACCEL_STD = 0.1;  // .1
constexpr double GPS_POS_STD = 3.0;
// -------------------------------------------------- //

void KalmanFilter::predictionStep(double dt)
{
    if (!isInitialised() && INIT_ON_FIRST_PREDICTION)
    {
        // Implement the State Vector and Covariance Matrix Initialisation in the
        // section below if you want to initialise the filter WITHOUT waiting for
        // the first measurement to occur. Make sure you call the setState() /
        // setCovariance() functions once you have generated the initial conditions.
        // Hint: Assume the state vector has the form [X,Y,VX,VY].
        // Hint: You can use the constants: INIT_POS_STD, INIT_VEL_STD
        // ----------------------------------------------------------------------- //
        // ENTER YOUR CODE HERE
            VectorXd state = Vector4d::Zero();  // State vector [X, Y, VX, VY]
            MatrixXd cov = Matrix4d::Zero();    // Matrix P: state-estimation error covariance

            // Assume the initial position is (X,Y) = (0,0) m
            // Assume the initial velocity is 5 m/s at 45 degrees (VX,VY) = (5*cos(45deg),5*sin(45deg)) m/s
            // state << 0, 0, 5*cos(M_PI/4), 5*sin(M_PI/4);
            state << 0, 0, 0, 0;

            cov(0,0) = INIT_POS_STD * INIT_POS_STD;
            cov(1,1) = INIT_POS_STD * INIT_POS_STD;
            cov(2,2) = INIT_VEL_STD * INIT_VEL_STD;
            cov(3,3) = INIT_VEL_STD * INIT_VEL_STD;

            setState(state);
            setCovariance(cov);
        // ----------------------------------------------------------------------- //
    }

    if (isInitialised())
    {
        VectorXd state = getState();     // State vector [X, Y, VX, VY]
        MatrixXd cov = getCovariance();  // Matrix P: state-estimation error covariance

        // Implement The Kalman Filter Prediction Step for the system in the
        // section below.
        // Hint: You can use the constants: ACCEL_STD
        // ----------------------------------------------------------------------- //
        // ENTER YOUR CODE HERE

        MatrixXd F = MatrixXd::Identity(4, 4);  // State transition matrix
        F(0, 2) = dt;
        F(1, 3) = dt;

        MatrixXd Q = Matrix2d::Zero();  // Process noise covariance matrix
        Q(0, 0) = ACCEL_STD * ACCEL_STD;
        Q(1, 1) = ACCEL_STD * ACCEL_STD;

        MatrixXd L = MatrixXd(4, 2);  // Process-model noise sensitivity matrix
        L << (0.5*dt*dt),           0,
                       0, (0.5*dt*dt),
                      dt,           0,
                       0,          dt;

        state = F * state;  // Where is the input ?
        cov = F * cov * F.transpose() + L * Q * L.transpose();

        // ----------------------------------------------------------------------- //

        setState(state);
        setCovariance(cov);
    }
}

void KalmanFilter::handleGPSMeasurement(GPSMeasurement meas)
{
    if(isInitialised())
    {
        VectorXd state = getState();     // State vector [X, Y, VX, VY]
        MatrixXd cov = getCovariance();  // Matrix P: state-estimation error covariance

        // Implement The Kalman Filter Update Step for the GPS Measurements in the
        // section below.
        // Hint: Assume that the GPS sensor has a 3m (1 sigma) position uncertainty.
        // Hint: You can use the constants: GPS_POS_STD
        // ----------------------------------------------------------------------- //
        // ENTER YOUR CODE HERE

        Vector2d z;  // Measurement vector. Read from GPS
        z << meas.x,
             meas.y;

        MatrixXd H = MatrixXd(2,4);  // Measurement matrix. Maps state to measurement space
        H << 1,0,0,0,
             0,1,0,0;

        MatrixXd R = Matrix2d::Zero();  // Measurement noise covariance matrix
        R(0,0) = GPS_POS_STD * GPS_POS_STD;
        R(1,1) = GPS_POS_STD * GPS_POS_STD;

        VectorXd z_hat = H * state;  // Predicted measurement
        VectorXd y     = z - z_hat;  // Measurement residual, or 'Innovation'
        MatrixXd S     = H * cov * H.transpose() + R;  // Innovation (or residual) covariance
        MatrixXd K     = cov * H.transpose() * S.inverse();  // Optimal 'Kalman gain'

        state = state + K * y;
        cov = (MatrixXd::Identity(4,4) - K * H) * cov;

        // ----------------------------------------------------------------------- //

        setState(state);
        setCovariance(cov);
    }
    else
    {
        // Implement the State Vector and Covariance Matrix Initialisation in the
        // section below. Make sure you call the setState/setCovariance functions
        // once you have generated the initial conditions.
        // Hint: Assume the state vector has the form [X,Y,VX,VY].
        // Hint: You can use the constants: GPS_POS_STD, INIT_VEL_STD
        // ----------------------------------------------------------------------- //
        // ENTER YOUR CODE HERE
            VectorXd state = Vector4d::Zero();  // State vector [X, Y, VX, VY]
            MatrixXd cov = Matrix4d::Zero();    // Matrix P: state-estimation error covariance

            state(0) = meas.x;
            state(1) = meas.y;

            cov(0,0) = GPS_POS_STD * GPS_POS_STD;
            cov(1,1) = GPS_POS_STD * GPS_POS_STD;
            cov(2,2) = INIT_VEL_STD * INIT_VEL_STD;
            cov(3,3) = INIT_VEL_STD * INIT_VEL_STD;

            setState(state);
            setCovariance(cov);
        // ----------------------------------------------------------------------- //
    }
}

Matrix2d KalmanFilter::getVehicleStatePositionCovariance()
{
    Matrix2d pos_cov = Matrix2d::Zero();
    MatrixXd cov = getCovariance();
    if (isInitialised() && cov.size() != 0){pos_cov << cov(0,0), cov(0,1), cov(1,0), cov(1,1);}
    return pos_cov;
}

VehicleState KalmanFilter::getVehicleState()
{
    if (isInitialised())
    {
        VectorXd state = getState(); // STATE VECTOR [X,Y,VX,VY]
        double psi = std::atan2(state[3],state[2]);
        double V = std::sqrt(state[2]*state[2] + state[3]*state[3]);
        return VehicleState(state[0],state[1],psi,V);
    }
    return VehicleState();
}

void KalmanFilter::predictionStep(GyroMeasurement gyro, double dt){predictionStep(dt);}
void KalmanFilter::handleLidarMeasurements(const std::vector<LidarMeasurement>& dataset, const BeaconMap& map){}
void KalmanFilter::handleLidarMeasurement(LidarMeasurement meas, const BeaconMap& map){}

