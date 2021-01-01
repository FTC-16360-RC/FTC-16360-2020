package org.firstinspires.ftc.teamcode.lib.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.lib.G;
import org.firstinspires.ftc.teamcode.lib.datatypes.INTuple;
import org.firstinspires.ftc.teamcode.lib.datatypes.Twople;

import java.util.ArrayList;

public class Shooter {

    HardwareMap hardwareMap;
    DcMotorEx motor1;
    DcMotorEx motor2;

    Servo flap;
    Servo feeder;
    Servo blocker;

    public Shooter (HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;

        motor1 = hardwareMap.get(DcMotorEx.class, "shooter1");
        motor2 = hardwareMap.get(DcMotorEx.class, "shooter2");
        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        flap = hardwareMap.get(Servo.class, "flap");
        feeder = hardwareMap.get(Servo.class, "feeder");
        blocker = hardwareMap.get(Servo.class, "blocker");
    }

    public void enableShooter() {
        motor1.setVelocity(6000);
        motor2.setVelocity(6000);
    }

    public void disableShooter() {
        motor1.setPower(0);
        motor2.setPower(0);
    }

    public void shootOne() {

    }
    public void shootThree() {

    }
    public void adjustFlap(double angle) {

    }

    public ArrayList<Twople> update(ArrayList<INTuple> instructions) {

        for (INTuple i : instructions) {
            switch (i.a) {
                case ENABLE_SHOOTER:
                    enableShooter();
                    break;
                case DISABLE_SHOOTER:
                    disableShooter();
                    break;
                case SHOOT_ONE:
                    shootOne();
                    break;
                case SHOOT_THREE:
                    adjustFlap(i.b_double);
                default:
                    break;
            }
        }


        ArrayList<Twople> output = new ArrayList<>();
        return output;
    }
}

