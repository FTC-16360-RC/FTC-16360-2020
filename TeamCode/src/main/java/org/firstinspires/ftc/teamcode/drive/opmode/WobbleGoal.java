package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class WobbleGoal {

    public enum Mode {
        NOTHING,
        HOLDING,
        LIFTING
    }

    private Servo arm; //the one that goes up

    private Servo hand; // the one that holds the wobble goal

    private final double armStartPosition = 0.375;
    private final double armLiftingPosition = 0.7;

    private final double handStartPosition = 0.9;
    private final double handHoldingPosition = 0.3;

    private WobbleGoal.Mode mode;


    public WobbleGoal(HardwareMap hardwaremap){
        arm = hardwaremap.get(Servo.class, "arm");
        hand = hardwaremap.get(Servo.class, "hand");
        arm.setPosition(armStartPosition);
        hand.setPosition(handHoldingPosition);
        mode = Mode.NOTHING;
    }

    double waitTime1 = 0.7;
    ElapsedTime waitTimer1 = new ElapsedTime();

    double waitTime2 = 0.7;
    ElapsedTime waitTimer2 = new ElapsedTime();

    public void armHigh () {
        arm.setPosition(armLiftingPosition);
    }

    public void armGround () {
        arm.setPosition(armStartPosition);
    }

    public void handStart () {
        hand.setPosition(handStartPosition);
    }

    public void handHolding () {
        hand.setPosition(handHoldingPosition);
    }

    public void holdIt() {
        hand.setPosition(handHoldingPosition);
        mode = Mode.HOLDING;
        waitTimer1.reset();
        while (waitTimer1.seconds() <= waitTime1) {

        }
        arm.setPosition(armLiftingPosition);
        mode = Mode.LIFTING;
    }

    public void depositWobbleGoal () {
        arm.setPosition(armStartPosition);
        mode = Mode.HOLDING;
        waitTimer2.reset();
        while (waitTimer2.seconds() <= waitTime2) {

        }
        hand.setPosition(handStartPosition);
        mode = Mode.NOTHING;
    }
}
