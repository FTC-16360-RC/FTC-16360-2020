package org.firstinspires.ftc.teamcode.lib.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.lib.datatypes.TUtil;
import org.firstinspires.ftc.teamcode.lib.datatypes.UTuple;
import org.firstinspires.ftc.teamcode.lib.datatypes.util.Adresses;
import org.firstinspires.ftc.teamcode.lib.datatypes.util.Instructions;

@Config
public class Intake {

    // Define 3 states. on, off or reverse
    public enum Mode {
        NORMAL,
        IDLE,
        REVERSE
    }

    private DcMotor intake;
    private Servo servo;

    private Mode mode;

    public Intake(HardwareMap hardwaremap) {
        servo = hardwaremap.get(Servo.class, "intake");
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

    public void lowerIntake() {
        //servo.setPosition(1);
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
    public TUtil update (TUtil instructions) {
        TUtil messages = new TUtil();
        for (UTuple i : instructions.list) {
            switch (i.a_ins) {
                case LOWER_INTAKE_DEBUG:
                    lowerIntake();
                    break;
                case SET_INTAKE_IDLE:
                    setMode(Mode.IDLE);
                    break;
                case SET_INTAKE_ON:
                    setMode(Mode.NORMAL);
                    break;
                case SET_INTAKE_REVERSE:
                    setMode(Mode.REVERSE);
                    break;
                default:
                    break;
            }
        }
        return messages;
    }
}