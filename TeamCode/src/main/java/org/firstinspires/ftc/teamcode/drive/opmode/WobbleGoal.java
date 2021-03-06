package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class WobbleGoal {

    public enum Mode {
        NOTHING,    // down, open
        HOLDING,    // down, closed
        LIFTING,    // high, closed
        WAIT_1,
        WAIT_2,
        WAIT_3
    }

    private Servo arm1; //the one that goes up
    private Servo arm2;

    private Servo hand; // the one that holds the wobble goal

    private final double arm1StartPosition = 1;
    private final double arm1LiftingPosition = 0.05;

    private final double arm2StartPosition = 0;
    private final double arm2LiftingPosition = 0.95;

    private final double handStartPosition = 0.9;
    private final double handHoldingPosition = 0.3;

    private WobbleGoal.Mode mode;


    public WobbleGoal(HardwareMap hardwaremap){
        arm1 = hardwaremap.get(Servo.class, "arm1");
        arm2 = hardwaremap.get(Servo.class, "arm2");
        hand = hardwaremap.get(Servo.class, "hand");
        arm1.setPosition(arm1StartPosition); //arm1LiftingPosition
        arm2.setPosition(arm2StartPosition); //arm2LiftingPosition
        hand.setPosition(handStartPosition); //handHoldingPosition
        mode = Mode.NOTHING;
    }

    double waitTime1 = 1;
    ElapsedTime waitTimer1 = new ElapsedTime();

    double waitTime2 = 1;
    ElapsedTime waitTimer2 = new ElapsedTime();

    double waitTime3 = 2;
    ElapsedTime waitTimer3 = new ElapsedTime();

    public void setMode (Mode mode){
        this.mode = mode;
        switch (this.mode)
        {
            case NOTHING:
                arm1.setPosition(arm1StartPosition);
                arm2.setPosition(arm2StartPosition);
                this.mode = Mode.WAIT_1;
                waitTimer1.reset();
                break;
            case WAIT_1:
                if (waitTimer1.seconds() >= waitTime1) {
                    hand.setPosition(handStartPosition);
                }
                break;
            case HOLDING:
                arm1.setPosition(arm1StartPosition);
                arm2.setPosition(arm2StartPosition);
                this.mode = Mode.WAIT_2;
                waitTimer2.reset();
                break;
            case WAIT_2:
                if (waitTimer2.seconds() >= waitTime2) {
                    hand.setPosition(handHoldingPosition);
                }
                break;
            case LIFTING:
                hand.setPosition(handHoldingPosition);
                waitTimer3.reset();
                this.mode = Mode.WAIT_3;
                break;
            case WAIT_3:
                if (waitTimer3.seconds() >= waitTime3) {
                    arm1.setPosition(arm1LiftingPosition);
                    arm2.setPosition(arm2LiftingPosition);
                }
                break;
        }

    }
}
