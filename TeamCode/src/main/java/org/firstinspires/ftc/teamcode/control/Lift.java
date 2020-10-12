package org.firstinspires.ftc.teamcode.control;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Lift {

    private DcMotor lift_left;
    private DcMotor lift_right;

    private static final double gearRatio = 5.2;     //gear ratio of lift_left motor
    private static final double rawEncoderPPR = 28;  //pulses per revolution of encoder, not output shaft
    private static final double spoolDiameter = 3.8;

    private static final int EPC = (int)Math.round(rawEncoderPPR*gearRatio/(spoolDiameter*Math.PI)); //encoder counts per cm of string

    private final double foundationHeight = 3.5;
    private final double nubHeight = 2.5;
    private final double stoneHeight = 9.4;
    private final double capstoneHeight = 6;  //6

    private double offset = 2.5;

    private int level = 0;

    private double aimingOffset = 0;

    private boolean stateChanged = false;

    private boolean extensionRetracted = false;

    private boolean alreadyAimed;

    public enum LiftState
    {
        EXTENDED,
        RETRACTED,
        EXTENDING,
        AIMING,
        CAPSTONE,
        BLOCK
    }

    private LiftState liftState;

    /*
    //  METHODS
     */

    //constructor
    public Lift(HardwareMap hardwareMap)
    {
        //configure intake motors
        lift_left = hardwareMap.get(DcMotor.class, "lift left");
        lift_right = hardwareMap.get(DcMotor.class, "lift right");
        lift_right.setDirection(DcMotor.Direction.REVERSE);
        lift_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift_left.setTargetPosition(lift_left.getCurrentPosition());
        lift_right.setTargetPosition(lift_left.getCurrentPosition());
        lift_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        liftState = LiftState.RETRACTED;
    }

    public int getLevel()
    {
        return level;
    }

    public void setLevel(int increment)
    {
        level += increment;
        if(level >= 10)
        {
            level = 10;
        }
        if(level <= 0)
        {
            level = 0;
        }
        if(liftState == LiftState.EXTENDED && increment > 0) {
           alreadyAimed = true;
           liftUp();
        }
    }

    public void changeOffset(double increment)
    {
        offset += increment;
        if(liftState == LiftState.EXTENDED) {
            liftUp();
        }
        if(liftState == LiftState.AIMING) {
            liftAim();
        }

    }

    public void resetOffset()
    {
        offset = 0;
    }

    public void liftUp()
    {
        if(level < 10 && !alreadyAimed)
        {
            aimingOffset = 1;
        } else {
            aimingOffset = 0.35;
        }
        if(lift_left.getMode() == DcMotor.RunMode.RUN_WITHOUT_ENCODER)
        {
            lift_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        lift_left.setTargetPosition(-1*(int)Math.round((foundationHeight+nubHeight+stoneHeight*(level+aimingOffset)+offset)*EPC));
        lift_right.setTargetPosition(-1*(int)Math.round((foundationHeight+nubHeight+stoneHeight*(level+aimingOffset)+offset)*EPC));
        lift_left.setPower(1);
        lift_right.setPower(1);
    }

    private void liftAim()
    {
        lift_left.setTargetPosition(-1 * (int) Math.round((foundationHeight + stoneHeight * level + offset) * EPC));
        lift_right.setTargetPosition(-1 * (int) Math.round((foundationHeight + stoneHeight * level + offset) * EPC));
    }

    private void liftCapstone()
    {
        lift_left.setTargetPosition(-1 * (int) Math.round(capstoneHeight * EPC));
        lift_right.setTargetPosition(-1 * (int) Math.round(capstoneHeight * EPC));
        lift_left.setPower(1);
        lift_right.setPower(1);
    }

    private void liftBlock()
    {
        lift_left.setTargetPosition(-1 * (int) Math.round(2 * EPC));
        lift_right.setTargetPosition(-1 * (int) Math.round(2 * EPC));
        lift_left.setPower(1);
        lift_right.setPower(1);
    }

    private void liftDown()
    {
        if(lift_left.getMode() != DcMotor.RunMode.RUN_TO_POSITION)
        {
            lift_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        lift_left.setTargetPosition(0);
        lift_right.setTargetPosition(0);
        lift_left.setPower(0.55);
        lift_right.setPower(0.55);
    }

    public void resetMinimum()
    {
        lift_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void update(double manualPower, double aimingPower, boolean extensionRetracted, boolean gripperClosed)
    {
        this.extensionRetracted = extensionRetracted;
        if(Math.abs(manualPower) > 0.05)
        {
            if(lift_left.getMode() != DcMotor.RunMode.RUN_WITHOUT_ENCODER)
            {
                lift_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                lift_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
            lift_left.setPower(0.3*manualPower);
            lift_right.setPower(0.3*manualPower);
        } else {
            if(lift_left.getMode() != DcMotor.RunMode.RUN_TO_POSITION)
            {
                lift_left.setTargetPosition(lift_left.getCurrentPosition());
                lift_right.setTargetPosition(lift_right.getCurrentPosition());
                lift_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift_left.setPower(1);
                lift_right.setPower(1);
            }
            if(liftState == LiftState.AIMING) {
                if(level == 10)
                {
                    lift_left.setPower(1);
                    lift_right.setPower(1);
                } else {
                    lift_left.setPower(aimingPower*0.15);  //0.25
                    lift_right.setPower(aimingPower*0.15);
                }
            }
            if(liftState == LiftState.RETRACTED && gripperClosed && extensionRetracted) {
                stateChanged = true;
                liftState =  LiftState.BLOCK;
            }
            if (stateChanged)
                {
                if(liftState == LiftState.RETRACTED && extensionRetracted)
                {
                    liftDown();
                    alreadyAimed = false;
                    stateChanged = false;
                } else {
                    if(liftState != LiftState.RETRACTED) {
                        stateChanged = false;
                    }
                    switch (liftState) {
                        case EXTENDED:
                            liftUp();
                            break;
                        case AIMING:
                            liftAim();
                            alreadyAimed = true;
                            break;
                        case CAPSTONE:
                            liftCapstone();
                            break;
                        case BLOCK:
                            liftBlock();
                            liftState = LiftState.RETRACTED;
                            break;
                        default:
                            break;
                    }
                }
            }
        }
    }

    public LiftState getLiftState() {
        if(liftState == LiftState.EXTENDED && Math.abs(lift_left.getTargetPosition()-lift_left.getCurrentPosition()) > stoneHeight*EPC)
        {
            return LiftState.EXTENDING;
        } else {
            return liftState;
        }
    }

    public void setLiftState(LiftState lift_leftState)
    {
        this.liftState = lift_leftState;
        stateChanged = true;
    }

    public void unsetAim(){
        alreadyAimed = false;
    }
}