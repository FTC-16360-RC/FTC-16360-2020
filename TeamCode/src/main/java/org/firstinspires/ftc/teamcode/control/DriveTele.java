package org.firstinspires.ftc.teamcode.control;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DriveTele extends MecanumControl {

    //matches superclass constructor
    public DriveTele(HardwareMap hardwareMap, DcMotor.RunMode runMode, DcMotor.ZeroPowerBehavior brakeMode) {
        super(hardwareMap, runMode, brakeMode);
    }

    public void setPower(double leftY, double leftX, double rightX, double maxDriveSpeed, double maxTurnSpeed)
    {
        front_left_power = (-leftY + leftX) * maxDriveSpeed + rightX * maxTurnSpeed;
        front_right_power = (-leftY - leftX) * maxDriveSpeed - rightX * maxTurnSpeed;
        rear_left_power = (-leftY - leftX ) * maxDriveSpeed + rightX * maxTurnSpeed;
        rear_right_power = (-leftY + leftX) * maxDriveSpeed - rightX * maxTurnSpeed;
    }

    public double getFront_left_power() {
        return front_left_power;
    }

    public double getFront_right_power() {
        return front_right_power;
    }

    public double getRear_left_power() {
        return rear_left_power;
    }

    public double getRear_right_power() {
        return rear_right_power;
    }
}
