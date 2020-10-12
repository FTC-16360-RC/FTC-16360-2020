package org.firstinspires.ftc.teamcode.control;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class FoundationPuller {

    private Servo foundation_left;
    private Servo foundation_right;
    private Servo grabber_right;
    private Servo grabber_left;

    public enum ServoState
    {
        OPEN,
        PREPARED,
        CLOSED,
        AUTO_MOVE,
        DEPOSIT,
        HOLD_STONE
    }

    public enum GrabberState
    {
        OPEN,
        PREPARED,
        CLOSED
    }

    private ServoState pullerState = ServoState.OPEN;
    private GrabberState rightGrabberState = GrabberState.OPEN;
    private GrabberState leftGrabberState = GrabberState.OPEN;

    //constructor
    public FoundationPuller(HardwareMap hardwareMap)
    {
        //Initialize the servos
        foundation_right = hardwareMap.get(Servo.class, "foundation right");
        foundation_left = hardwareMap.get(Servo.class, "foundation left");
        grabber_right = hardwareMap.get(Servo.class, "skystone grabber right");
        grabber_left = hardwareMap.get(Servo.class, "skystone grabber left");

        grabber_right.setPosition(0.9);
        grabber_left.setPosition(0.25);
        foundation_right.setPosition(0.25);
        foundation_left.setPosition(0.7);//starting position to free camera
    }

    public void setServoState(ServoState pullerState)
    {
        this.pullerState = pullerState;
        switch(pullerState) {
            case OPEN:
                foundation_right.setPosition(0.25);
                foundation_left.setPosition(1);
                break;
            case CLOSED:
                foundation_right.setPosition(0.98);
                foundation_left.setPosition(0);
                break;
            case PREPARED:
                foundation_right.setPosition(0.77);
                foundation_left.setPosition(0.3);
                break;
            case AUTO_MOVE:
                foundation_right.setPosition(0.87);
                foundation_left.setPosition(0.2);
                break;
            case DEPOSIT:
                foundation_right.setPosition(0.68);
                foundation_left.setPosition(0.43);
                break;
            case HOLD_STONE:
                foundation_right.setPosition(0.5);
                foundation_left.setPosition(0.6);
                break;
            default:
                foundation_right.setPosition(0.4);
                foundation_left.setPosition(0.75);
                break;
        }
        setGrabberStateRight(rightGrabberState);
        setGrabberStateLeft(leftGrabberState);
    }

    public void setGrabberStateRight(GrabberState grabberState)
    {
        this.rightGrabberState = grabberState;
        switch(grabberState) {
            case OPEN:
                switch(pullerState) {
                    case OPEN:
                        grabber_right.setPosition(0.9);
                        break;
                    case HOLD_STONE:
                        grabber_right.setPosition(0.6);
                        break;
                    case DEPOSIT:
                        grabber_right.setPosition(0.4);
                        break;
                    default:
                        grabber_right.setPosition(0);
                        break;
                }
                break;
            case CLOSED:
                grabber_right.setPosition(0.75);
                break;
            case PREPARED:
                grabber_right.setPosition(0.35);
                break;
            default:
                grabber_right.setPosition(0.45);
                break;
        }
    }

    public void setGrabberStateLeft(GrabberState grabberState)
    {
        this.leftGrabberState = grabberState;
        switch(grabberState) {
            case OPEN:
                switch(pullerState) {
                    case OPEN:
                        grabber_left.setPosition(0.2);
                        break;
                    case HOLD_STONE:
                        grabber_left.setPosition(0.4);
                        break;
                    case DEPOSIT:
                        grabber_left.setPosition(0.6);
                        break;
                    default:
                        grabber_left.setPosition(1);
                        break;
                }
                break;
            case CLOSED:
                grabber_left.setPosition(0.25);
                break;
            case PREPARED:
                grabber_left.setPosition(0.65);
                break;
            default:
                grabber_left.setPosition(0.55);
                break;
        }
    }

    public ServoState getServoState()
    {
        return pullerState;
    }
}
