package org.firstinspires.ftc.teamcode.newLib.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.newLib.Comms;
import org.firstinspires.ftc.teamcode.newLib.Robot;

@Config
public class Intake {

    // Define 3 states. on, off or reverse
    private DcMotor intake;
    private Servo servo;

    private Robot.Mode mode;
    private Robot.Mode lastMode;

    public Intake() {
        servo = Comms.hardwareMap.get(Servo.class, "intake");
        intake = Comms.hardwareMap.get(DcMotor.class, "intake");
        intake.setDirection(DcMotor.Direction.REVERSE);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intake.setPower(0);
        mode = Robot.Mode.IDLE;
    }

    public Robot.Mode getMode() {
        return this.mode;
    }

    public void lowerIntake() {
        //servo.setPosition(1);
    }

    public Robot.Mode getLastMode(){
        return lastMode;
    }

    public void setMode(Robot.Mode mode) {
        if (this.mode != mode) {
            lastMode = this.mode;
        }
        this.mode = mode;
        switch (this.mode)
        {
            case IDLE: //no power
                intake.setPower(0);
                break;
            case RUNNING: //intake
                intake.setPower(1);
                break;
            case REVERSE: //outtake
                intake.setPower(-1);
                break;
        }
    }
    public void update () {

    }
}