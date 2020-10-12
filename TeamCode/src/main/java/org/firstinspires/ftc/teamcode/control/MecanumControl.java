package org.firstinspires.ftc.teamcode.control;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class MecanumControl {
    //declare the drive motors
    protected DcMotor front_left = null;
    protected DcMotor front_right = null;
    protected DcMotor rear_left = null;
    protected DcMotor rear_right = null;

    protected double front_left_power = 0;
    protected double front_right_power = 0;
    protected double rear_left_power = 0;
    protected double rear_right_power = 0;

    /*
    //  METHODS
    */

    //constructor
    public MecanumControl(HardwareMap hardwareMap, DcMotor.RunMode runMode, DcMotor.ZeroPowerBehavior brakeMode)
    {
        //configure intake motors
        front_left = hardwareMap.get(DcMotor.class, "front left");
        front_right = hardwareMap.get(DcMotor.class, "front right");
        rear_left = hardwareMap.get(DcMotor.class, "rear left");
        rear_right = hardwareMap.get(DcMotor.class, "rear right");

        front_left.setDirection(DcMotor.Direction.REVERSE);
        front_right.setDirection(DcMotor.Direction.FORWARD);
        rear_left.setDirection(DcMotor.Direction.REVERSE);
        rear_right.setDirection(DcMotor.Direction.FORWARD);

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

    public void updatePower()
    {
        front_left.setPower(front_left_power);
        front_right.setPower(front_right_power);
        rear_left.setPower(rear_left_power);
        rear_right.setPower(rear_right_power);
    }

    public void stop()
    {
        front_left_power = 0;
        front_right_power = 0;
        rear_left_power = 0;
        rear_right_power = 0;
        updatePower();
    }

    protected void brakeFloat()
    {
        front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rear_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rear_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }
}
