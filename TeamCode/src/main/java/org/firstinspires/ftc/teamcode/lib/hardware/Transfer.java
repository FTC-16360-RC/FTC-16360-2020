package org.firstinspires.ftc.teamcode.lib.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lib.Globals;
import org.firstinspires.ftc.teamcode.lib.datatypes.TUtil;
import org.firstinspires.ftc.teamcode.lib.datatypes.UTuple;
import org.firstinspires.ftc.teamcode.lib.datatypes.util.Adresses;
import org.firstinspires.ftc.teamcode.lib.datatypes.util.Instructions;

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
    Telemetry telemetry;

    public Transfer() {
        this.telemetry = telemetry;

        transfer = Globals.hardwareMap.get(DcMotor.class, "transfer");
        transfer.setDirection(DcMotor.Direction.REVERSE);
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
                transfer.setPower(-1);
                break;
        }
    }

    public TUtil update (TUtil instructions) {

        TUtil messages = new TUtil();
        for (UTuple i : instructions.list) {
            switch (i.a_ins) {
                case SET_TRANSFER_IDLE:
                    setMode(Mode.IDLE);
                    break;
                case SET_TRANSFER_ON:
                    setMode(Mode.NORMAL);
                    break;
                case SET_TRANSFER_REVERSE:
                    setMode(Mode.REVERSE);
                    break;
                default:
                    break;
            }
        }
        return messages;
    }
}