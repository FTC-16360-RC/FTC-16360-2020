package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class Intake {

    // Define 3 states. on, off or reverse
    public enum Mode {
        NORMAL,
        IDLE,
        REVERSE
    }

    private DcMotor intake;

    private Mode mode;

    public Intake(HardwareMap hardwaremap) {
        intake = hardwaremap.get(DcMotor.class, "intake");
        intake.setDirection(DcMotor.Direction.REVERSE);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intake.setPower(0);
        mode = Mode.IDLE;
    }

    public Mode getMode() {
        return this.mode;
    }

    public void setMode(Mode mode) {
        this.mode = mode;
        switch (this.mode)
        {
            case IDLE: //no power
                intake.setPower(0);
                break;
            case NORMAL: //intake
                intake.setPower(1);
                break;
            case REVERSE: //outtake
                intake.setPower(-1);
                break;
        }
    }

}