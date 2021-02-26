package org.firstinspires.ftc.teamcode.newLib.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.newLib.Comms;
import org.firstinspires.ftc.teamcode.newLib.Robot;

@Config
public class Transfer {

    private DcMotor transfer;

    private Robot.Mode mode;
    private Robot.Mode lastMode;
    private Robot.Mode nextMode;

    Telemetry telemetry;

    public Transfer() {
        this.telemetry = telemetry;

        transfer = Comms.hardwareMap.get(DcMotor.class, "transfer");
        transfer.setDirection(DcMotor.Direction.REVERSE);
        transfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transfer.setPower(0);
        mode = Robot.Mode.IDLE;
    }

    public Robot.Mode getMode() {
        return this.mode;
    }

    public void setNextMode(Robot.Mode mode) {
        nextMode = mode;
    }

    public Robot.Mode getLastMode(){
        return lastMode;
    }

    public void setMode() {
        if (mode != nextMode) {
            lastMode = mode;
        }
        mode = nextMode;
        switch (mode)
        {
            case IDLE: //no power
                transfer.setPower(0);
                break;
            case RUNNING: //intake
                transfer.setPower(1);
                break;
            case REVERSE: //outtake
                transfer.setPower(-1);
                break;
        }
    }

    public void update () {
    }
}