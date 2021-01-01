package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class Transfer {

    // Define 3 states. on, off or reverse
    public enum Mode {
        NORMAL,
        IDLE,
        REVERSE
    }

    private DcMotor transfer;

    private Mode mode;

    public Transfer(HardwareMap hardwaremap) {
        transfer = hardwaremap.get(DcMotor.class, "intake");
        transfer.setDirection(DcMotor.Direction.FORWARD);
        transfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transfer.setPower(0);
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
                transfer.setPower(0);
                break;
            case NORMAL: //intake
                transfer.setPower(1);
                break;
            case REVERSE: //outtake
                transfer.setPower(-0.3);
                break;
        }
    }

}