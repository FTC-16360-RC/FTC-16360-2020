package org.firstinspires.ftc.teamcode.control;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Gripper {

    private Servo gripper;

    //open/closed claws
    public enum ClawMode
    {
        OPENED,
        OPENING,
        CLOSED,
        CLOSING
    }

    private double currentRuntime = 0;
    private double timerStart = 0;
    private double grippingTime = 0.4;
    private double releasingTime = 0.2;
    private ClawMode clawMode;

    /*
    //  METHODS
    */

    //constructor
    public Gripper(HardwareMap hardwareMap)
    {
        gripper = hardwareMap.get(Servo.class, "gripper");

        //initialise servo
        gripper.setPosition(0.3);
        clawMode = ClawMode.OPENED;
    }

    public void open()
    {
        gripper.setPosition(0.3);
        clawMode = ClawMode.OPENING;
        timerStart = currentRuntime;
    }

    public void close()
    {
        gripper.setPosition(0.52); //0.54
        clawMode = ClawMode.CLOSING;
        timerStart = currentRuntime;
    }

    public void update(double currentRuntime)
    {
        this.currentRuntime = currentRuntime;
    }

    public ClawMode getClawMode() {
        if(clawMode == ClawMode.OPENING && currentRuntime > timerStart+releasingTime) {
            clawMode = ClawMode.OPENED;
        }
        if(clawMode == ClawMode.CLOSING && currentRuntime > timerStart+grippingTime)
        {
            clawMode = ClawMode.CLOSED;
        }
        return clawMode;
    }
}
