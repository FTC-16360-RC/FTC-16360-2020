package org.firstinspires.ftc.teamcode.lib.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lib.G;
import org.firstinspires.ftc.teamcode.lib.datatypes.INTuple;
import org.firstinspires.ftc.teamcode.lib.datatypes.Twople;

import java.util.ArrayList;

public class Intake {

    HardwareMap hardwareMap;
    Telemetry telemetry;
    DcMotor motor;
    Servo servo;

    public Intake(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        motor = hardwareMap.get(DcMotor.class, "intake");
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        servo = hardwareMap.get(Servo.class, "intake");
    }

    public void lowerIntake() {
        servo.setPosition(100);
        servo.setPosition(0);
    }

    public void enableIntake() {
        motor.setPower(1);
        telemetry.addLine("Succ mode activated");
    }

    public void disableIntake() {
        motor.setPower(0);
    }


    public ArrayList<Twople> update(ArrayList<INTuple> instructions) {

        for (INTuple i : instructions) {
            switch (i.a) {
                case ENABLE_INTAKE:
                    enableIntake();
                    break;
                case DISABLE_INTAKE:
                    disableIntake();
                    break;
                default:
                    break;
            }
        }

        ArrayList<Twople> output = new ArrayList<>();
        return output;
    }

}
