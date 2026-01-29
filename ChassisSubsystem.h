#ifndef TR_EMBEDDED_CHASSIS_SUBSYSTEM_H
#define TR_EMBEDDED_CHASSIS_SUBSYSTEM_H

#include "mbed.h"
#include "../util/peripherals/imu/BNO055.h"

#include <motor/DJIMotor.h>
#include <communications/CANHandler.h>
#include <peripherals/imu/BNO055.h>
#include <peripherals/oled/SSD1308.h>
// #include <subsystems/ChassisKalman.h>
// #include <algorithms/WheelKalman.h>
#include <algorithms/Pose2D.h>
#include <algorithms/PID.h>
// #include <algorithms/WheelSpeeds.h>
// #include <algorithms/ChassisSpeeds.h>
#include "algorithms/eigen-3.4.0/Eigen/QR"

#define CAN_BUS_TYPE CANHandler::CANBUS_1
#define MOTOR_TYPE M3508
#define M3508_POST_MAX_RPM 469
#define INPUT_THRESHOLD 0.01

#define I2C_SDA PB_7
#define I2C_SCL PB_8
#define IMU_RESET PA_8

#define PI 3.14159265
#define SECONDS_PER_MINUTE 60
#define TICKS_PER_ROTATION 8192.0
#define WHEEL_DIAMETER_METERS 0.146

#define MAX_BEYBLADE_SPEED 1800
#define BEYBLADE_ACCELERATION 0.05
#define MAX_VEL 2.92

struct OmniKinematics
{
    double r1x, r1y, r2x, r2y, r3x, r3y, r4x, r4y;
};

struct WheelSpeeds
{
    double LF;
    double RF;
    double LB;
    double RB;

    void operator*=(double scalar)
    {
        LF *= scalar;
        RF *= scalar;
        LB *= scalar;
        RB *= scalar;
    }

    char *to_string()
    {
        char buffer[56];
        sprintf(buffer, "LF: %4.2f RF: %4.2f LB: %4.2f RB: %4.2f\0", LF, RF, LB, RB);
        return buffer;
    }
};

struct ChassisSpeeds
{
    double vX;
    double vY;
    double vOmega;
};

struct OmniKinematicsLimits
{
    double max_Vel;
    double max_vOmega;
};

class ChassisSubsystem
{
public:
    ChassisSubsystem(short lfId, short rfId, short lbId, short rbId, BNO055 &imu, double radius);

    enum BrakeMode
    {
        BRAKE,
        COAST
    };

    enum MotorLocation
    {
        LEFT_FRONT,
        RIGHT_FRONT,
        LEFT_BACK,
        RIGHT_BACK
    };

    enum SPEED_UNIT
    {
        RAD_PER_SECOND,
        RPM,
        METER_PER_SECOND,
    };

    enum DRIVE_MODE
    {
        YAW_ORIENTED,
        REVERSE_YAW_ORIENTED,
        ROBOT_ORIENTED
    };

    float previousRPM[4] = {0, 0, 0, 0};

    static float limitAcceleration(float desiredRPM, float previousRPM, int power);

    static float p_theory(int LeftFrontPower, int RightFrontPower, int LeftBackPower, int RightBackPower,
                          int LeftFrontRpm, int RightFrontRpm, int LeftBackRpm, int RightBackRpm);

    static float Bisection(int LeftFrontPower, int RightFrontPower, int LeftBackPower, int RightBackPower,
                           int LeftFrontRpm, int RightFrontRpm, int LeftBackRpm, int RightBackRpm,
                           float chassisPowerLimit);

    float power_limit; 

    // ===== NEW: Measured-power PI limiter =====
    // call this every loop with measured power (Watts) and dt (seconds)
    void updatePowerLimiter(float measuredPower_W, float dt_s);

    // optional tuning
    void setPowerLimiterGains(float kP, float kI, float alpha);

    float getPowerScale() const { return m_powerScale; }
    float getMeasuredPowerFiltered() const { return m_powerFiltered_W; }

    // =========================================

    WheelSpeeds getWheelSpeeds() const;

    float setWheelSpeeds(WheelSpeeds wheelSpeeds);

    WheelSpeeds normalizeWheelSpeeds(WheelSpeeds wheelSpeeds) const;

    void setWheelPower(WheelSpeeds wheelPower);

    ChassisSpeeds getChassisSpeeds() const;

    float setChassisSpeeds(ChassisSpeeds desiredChassisSpeeds_, DRIVE_MODE mode = ROBOT_ORIENTED);

    ChassisSpeeds rotateChassisSpeed(ChassisSpeeds speeds, double yawCurrent);

    DJIMotor &getMotor(MotorLocation location);

    void setMotorSpeedPID(MotorLocation location, float kP, float kI, float kD);

    void setSpeedIntegralCap(MotorLocation location, double cap);

    void setSpeedFeedforward(MotorLocation location, double FF);

    void setSpeedFF_Ks(double Ks);

    BrakeMode getBrakeMode();

    void setBrakeMode(BrakeMode brakeMode);

    void initializeImu();

    void periodic();

    static double radiansToDegrees(double radians);

    static double degreesToRadians(double degrees);

    int getHeadingDegrees() const;

    Pose2D getPose();

    double getMotorSpeed(MotorLocation location, SPEED_UNIT unit);

    void setOmniKinematicsLimits(double max_Vel, double max_vOmega);

    void readImu();

    void setYawReference(DJIMotor *motor, double initial_offset_ticks);

    ChassisSpeeds desiredChassisSpeeds;
    WheelSpeeds desiredWheelSpeeds;

    OmniKinematicsLimits m_OmniKinematicsLimits;
    WheelSpeeds chassisSpeedsToWheelSpeeds(ChassisSpeeds chassisSpeeds);
    ChassisSpeeds wheelSpeedsToChassisSpeeds(WheelSpeeds wheelSpeeds);
    char *MatrixtoString(Eigen::MatrixXd mat);

    ChassisSpeeds m_chassisSpeeds;
    WheelSpeeds m_wheelSpeeds;

    int PEAK_POWER_ALL;
    int PEAK_POWER_SINGLE;

    PID pid_LF;
    PID pid_RF;
    PID pid_LB;
    PID pid_RB;

    uint32_t lastPIDTime = 0;

    double prevVel;

    double yawPhase;
    int testData[300][4];
    int testDataIndex = 0;

    static double radiansToTicks(double radians);
    static double ticksToRadians(double ticks);

private:
    DJIMotor LF, RF, LB, RB;
    DJIMotor *yaw = 0;

    BrakeMode brakeMode;

    BNO055 imu;
    BNO055_ANGULAR_POSITION_typedef imuAngles;

    static double rpmToRadPerSecond(double RPM);
    static double radPerSecondToRPM(double radPerSecond);

    OmniKinematics m_OmniKinematics;
    float chassis_radius;
    void setOmniKinematics(double radius);

    double FF_Ks;

    void setMotorPower(MotorLocation location, double power);
    void setMotorSpeedRPM(MotorLocation location, double speed);
    double getMotorSpeedRPM(MotorLocation location);
    int motorPIDtoPower(MotorLocation location, double speed, uint32_t dt);

    double testAngle;
    int lastTimeMs;

    short lfId;
    short rfId;
    short lbId;
    short rbId;

    
    float m_powerScale = 1.0f;       
    float m_powerFiltered_W = 0.0f;  
    float m_powerIntegral = 0.0f;    
    
    float m_kP = 0.0025f;
    float m_kI = 0.0008f;
    float m_alpha = 0.10f;

    static float clampf(float x, float lo, float hi)
    {
        return (x < lo) ? lo : (x > hi) ? hi : x;
    }
};

#endif // TR_EMBEDDED_CHASSIS_SUBSYSTEM_H

