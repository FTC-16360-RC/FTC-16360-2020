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

    private Servo arm; //the one that goes up

    private Servo hand; // the one that holds the wobble goal

    private final double armStartPosition = 0.9;
    private final double armLiftingPosition = 0.375;

    private final double handStartPosition = 0.9;
    private final double handHoldingPosition = 0.3;

    private WobbleGoal.Mode mode;


    public WobbleGoal(HardwareMap hardwaremap){
        arm = hardwaremap.get(Servo.class, "arm");
        hand = hardwaremap.get(Servo.class, "hand");
        arm.setPosition(armLiftingPosition);
        hand.setPosition(handHoldingPosition);
        mode = Mode.NOTHING;
    }

    double waitTime1 = 10;
    ElapsedTime waitTimer1 = new ElapsedTime();

    double waitTime2 = 1;
    ElapsedTime waitTimer2 = new ElapsedTime();

    double waitTime3 = 1.5;
    ElapsedTime waitTimer3 = new ElapsedTime();

    public void setMode (Mode mode){
        this.mode = mode;
        switch (this.mode)
        {
            case NOTHING:
                arm.setPosition(armStartPosition);
                mode = Mode.WAIT_1;
                waitTimer1.reset();
                break;
            case WAIT_1:
                if (waitTimer1.seconds() >= waitTime1) {
                    hand.setPosition(handStartPosition);
                }
                break;
            case HOLDING:
                arm.setPosition(armStartPosition);
                mode = Mode.WAIT_2;
                waitTimer2.reset();
                break;
            case WAIT_2:
                if (waitTimer2.seconds() >= waitTime2) {
                    hand.setPosition(handHoldingPosition);
                }
                break;
            case LIFTING:
                hand.setPosition(handHoldingPosition);
                mode = Mode.WAIT_3;
                waitTimer3.reset();
                break;
            case WAIT_3:
                if (waitTimer3.seconds() >= waitTime3){
                    arm.setPosition(armLiftingPosition);
                }
                break;
        }

    }
}
