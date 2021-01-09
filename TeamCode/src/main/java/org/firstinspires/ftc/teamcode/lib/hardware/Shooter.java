package org.firstinspires.ftc.teamcode.lib.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.lib.G;
import org.firstinspires.ftc.teamcode.lib.datatypes.UTuple;

import java.util.ArrayList;

public class Shooter {

    public class Goal {
        double x;
        double y;
        double z;

        public Goal(double x, double y, double z) {
            this.x = x;
            this.y = y;
            this.z = z;
        }
    }

    HardwareMap hardwareMap;
    DcMotorEx motor1;
    DcMotorEx motor2;

    Servo flap;
    Servo feeder;

    Goal HighGoal = new Goal(1,2, 3);
    double[] position;

    public Shooter (HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;

        motor1 = hardwareMap.get(DcMotorEx.class, "shooter1");
        motor2 = hardwareMap.get(DcMotorEx.class, "shooter2");
        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        flap = hardwareMap.get(Servo.class, "flap");
        feeder = hardwareMap.get(Servo.class, "feeder");
    }

    private void enableShooter() {
        motor1.setVelocity(6000);
        motor2.setVelocity(6000);
    }

    private void disableShooter() {
        motor1.setPower(0);
        motor2.setPower(0);
    }

    public void shootOne() {
        adjustFlap();
        feeder.setPosition(1);
        feeder.setPosition(0);
    }
    private void shootThree() {
        shootOne();
        shootOne();
        shootOne();
    }
    private double g_inverse() {
        double u = 1;
        double v = 1;

        double d_x = position[0] - HighGoal.x;
        double d_y = position[1] - HighGoal.y;
        double d_s = Math.sqrt(d_x * d_x + d_y * d_y);

        return u * d_s + v * HighGoal.z;
    }
    private void adjustFlap() {
        double angle = g_inverse();
        double max_angle = 45;
        flap.setPosition(angle / max_angle);
    }

    //debug
    private void feedRing() {
        feeder.setPosition(1);
        feeder.setPosition(0);
    }

    public ArrayList<UTuple> update(ArrayList<UTuple> instructions) {

        for (UTuple i : instructions) {
            switch (i.a_ins) {
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
                    shootThree();
                    break;
                case RECIEVE_POSITION:
                    position = i.b_arr;
                    break;

                case ADJUST_FLAP_DEBUG:
                    adjustFlap();
                    break;
                case FEED_RING_DEBUG:
                    feedRing();
                    break;
                default:
                    break;
            }
        }


        ArrayList<UTuple> output = new ArrayList<>();
        return output;
    }
}

