package org.firstinspires.ftc.teamcode.lib.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lib.G;
import org.firstinspires.ftc.teamcode.lib.datatypes.UTuple;

import java.util.ArrayList;

public class Intake {

    HardwareMap hardwareMap;
    Telemetry telemetry;
    DcMotor motor1;
    DcMotor motor2;
    Servo servo;

    public Intake(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        motor1 = hardwareMap.get(DcMotor.class, "intake");
        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor2 = hardwareMap.get(DcMotor.class, "lift");
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        servo = hardwareMap.get(Servo.class, "intake");
    }

    public void lowerIntake() {
        servo.setPosition(1);
    }
    private void enableIntake() {
        motor1.setPower(1);
        motor2.setPower(1);
        telemetry.addLine("Succ mode activated");
    }
    private void disableIntake() {
        motor1.setPower(0);
        motor2.setPower(0);
    }

    //debugging stuff
    private void enableIntakeOnly() {
        motor1.setPower(1);
    }
    private void disableIntakeOnly() {
        motor1.setPower(0);
    }
    private void enableLiftOnly() {
        motor2.setPower(1);
    }
    private void disableLiftOnly() {
        motor2.setPower(0);
    }

    public ArrayList<UTuple> update(ArrayList<UTuple> instructions) {

        for (UTuple i : instructions) {
            switch (i.a_ins) {
                case ENABLE_INTAKE:
                    enableIntake();
                    break;
                case DISABLE_INTAKE:
                    disableIntake();
                    break;

                case ENABLE_INTAKE_DEBUG:
                    enableIntakeOnly();
                    break;
                case DISABLE_INTAKE_DEBUG:
                    disableIntakeOnly();
                    break;
                case ENABLE_LIFT_DEBUG:
                    enableLiftOnly();
                    break;
                case DISABLE_LIFT_DEBUG:
                    disableLiftOnly();
                    break;
                case LOWER_INTAKE_DEBUG:
                    lowerIntake();
                    break;
                default:
                    break;
            }
        }

        ArrayList<UTuple> output = new ArrayList<>();
        return output;
    }

}
