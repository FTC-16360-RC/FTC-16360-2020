package org.firstinspires.ftc.teamcode.lib.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class MecanumControl {
    //declare the drive motors
    DcMotor front_left;
    DcMotor front_right;
    DcMotor rear_left;
    DcMotor rear_right;
    Telemetry telemetry;

    protected double front_left_power = 0;
    protected double front_right_power = 0;
    protected double rear_left_power = 0;
    protected double rear_right_power = 0;

    /*
    //  METHODS
    */

    //constructor
    public MecanumControl(HardwareMap hardwareMap, DcMotor.RunMode runMode, DcMotor.ZeroPowerBehavior brakeMode, Telemetry t)
    {
        telemetry = t;

        //configure intake motors
        front_left = hardwareMap.get(DcMotor.class, "front left");
        front_right = hardwareMap.get(DcMotor.class, "front right");
        rear_left = hardwareMap.get(DcMotor.class, "rear left");
        rear_right = hardwareMap.get(DcMotor.class, "rear right");

        front_left.setDirection(DcMotor.Direction.FORWARD);
        front_right.setDirection(DcMotor.Direction.FORWARD);
        rear_left.setDirection(DcMotor.Direction.REVERSE);
        rear_right.setDirection(DcMotor.Direction.REVERSE);

        front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rear_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rear_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        front_left.setMode(runMode);
        front_right.setMode(runMode);
        rear_left.setMode(runMode);
        rear_right.setMode(runMode);

        front_left.setZeroPowerBehavior(brakeMode);
        front_right.setZeroPowerBehavior(brakeMode);
        rear_left.setZeroPowerBehavior(brakeMode);
        rear_right.setZeroPowerBehavior(brakeMode);
    }

    public void setPower()
    {
        front_left.setPower(front_left_power);
        front_right.setPower(front_right_power);
        rear_left.setPower(rear_left_power);
        rear_right.setPower(rear_right_power);
        telemetry.addData("front left", front_left.getPower());
        telemetry.addData("front right", front_right.getPower());
        telemetry.addData("rear left", rear_left.getPower());
        telemetry.addData("rear right", rear_right.getPower());
    }
    public void setPower(double fl, double fr, double rl, double rr)
    {
        front_left_power = fl;
        front_right_power = fr;
        rear_left_power = rl;
        rear_right_power = rr;
        setPower();
    }
    public void setPower(double leftY, double leftX, double rightX, double maxDriveSpeed, double maxTurnSpeed)
    {
        front_left_power = (-leftY + leftX) * maxDriveSpeed - rightX * maxTurnSpeed;
        front_right_power = (-leftY - leftX) * maxDriveSpeed - rightX * maxTurnSpeed;
        rear_left_power = (-leftY - leftX ) * maxDriveSpeed + rightX * maxTurnSpeed;
        rear_right_power = (-leftY + leftX) * maxDriveSpeed + rightX * maxTurnSpeed;
        setPower();
    }


    public void stop()
    {
        front_left_power = 0;
        front_right_power = 0;
        rear_left_power = 0;
        rear_right_power = 0;
        setPower();
    }

    protected void brakeFloat()
    {
        front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rear_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rear_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }
}
