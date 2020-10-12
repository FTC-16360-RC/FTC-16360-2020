package org.firstinspires.ftc.teamcode.control;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class IMU {
    // The IMU sensor object
    private BNO055IMU imu;

    // State used for updating telemetry
    private Orientation angles;
    private Acceleration gravity;

    private double driveKP = 0;
    private double driveKI = 0;
    private double driveKD = 0;
    private double turnKP = 0;
    private double currentHeading = 0;
    private double targetHeading = 0;
    private double headingError = 0;
    private double lastHeading = 0;
    private double headingIntegral = 0;
    private double headingDerivative = 0;

    /*
    //  METHODS
    */

    //constructor
    public IMU(HardwareMap hardwareMap) {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = false;

        //initialize IMU
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        gravity = imu.getGravity();
    }

    private double heading(){
        // and return the rotation in degrees on the Z-axis of the imu
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle + 180;
    }

    public void update(){
        currentHeading = heading();
    }

    public double getDriveCorrection() {
        headingError = currentHeading - targetHeading;
        if(headingError > 180) {
            headingError = headingError - 360;
        }
        if(headingError < -180) {
            headingError = headingError + 360;
        }
        headingIntegral += headingError;
        headingDerivative = lastHeading - currentHeading;
        return headingError*driveKP + headingIntegral*driveKI + headingDerivative*driveKD;
    }

    public double getTurnSpeedCoefficient(boolean turnLeft) {
        headingError = targetHeading - currentHeading;
        if(headingError > 180 && !turnLeft) {
            headingError = headingError - 360;
        }
        if(headingError < -180 && turnLeft) {
            headingError = headingError + 360;
        }
        return Math.abs(headingError)*turnKP;
    }

    public void setDrivePID(double driveKP, double driveKI, double driveKD) {
        this.driveKP = driveKP;
        this.driveKP = driveKI;
        this.driveKP = driveKD;
    }

    public void setTurnKP(double turnKP) {
        this.turnKP = turnKP;
    }

    public double getHeading() {
        return currentHeading;
    }

    public void setTargetHeading(double targetHeading) {
        this.targetHeading = targetHeading;
        headingError = 0;
        headingIntegral = 0;
        headingDerivative = 0;
    }
}
