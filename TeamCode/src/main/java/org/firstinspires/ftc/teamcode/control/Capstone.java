package org.firstinspires.ftc.teamcode.control;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Capstone {
    private Servo capstone_grabber;
    private Servo capstone_turner;

    private double turningTime = 0.5;
    private double openingTime = 0.4;
    private double retractingTime = 0.4;
    private double currentRuntime = 0;
    private double timerStart = 0;

    private boolean deployCapstone = false;

    public enum CapstoneState {
        RETRACTED,
        RETRACTING,
        PREPARED,
        OPENED
    }

    private CapstoneState capstoneState = CapstoneState.RETRACTED;

    /*
    //  METHODS
    */

    //constructor
    public Capstone(HardwareMap hardwareMap)
    {
        //configure capstone servos
        capstone_grabber = hardwareMap.get(Servo.class, "capstone grabber");
        capstone_turner = hardwareMap.get(Servo.class, "capstone turner");

        //set capstone servos to holding position
        capstone_turner.setPosition(0.45);
        capstone_grabber.setPosition(0.66);
    }

    public void deploy()
    {
        deployCapstone = true;
    }

    public void close_grabber()
    {
        capstone_grabber.setPosition(0.66);
    }


    public void update(double currentRuntime, boolean blockIntaked)
    {
        this.currentRuntime = currentRuntime;
        if(deployCapstone)
        {
            switch(capstoneState)
            {
                case RETRACTED:
                    capstone_turner.setPosition(0.9);
                    capstoneState = CapstoneState.PREPARED;
                    timerStart = currentRuntime;
                    break;
                case PREPARED:
                    if(currentRuntime > timerStart+turningTime)
                    {
                        capstone_grabber.setPosition(0);
                        capstoneState = CapstoneState.OPENED;
                        timerStart = currentRuntime;
                    }
                    break;
                case OPENED:
                    if(currentRuntime > timerStart+openingTime)
                    {
                        capstone_turner.setPosition(0.45);
                        capstoneState = CapstoneState.RETRACTING;
                        timerStart = currentRuntime;
                    }
                    break;
                case RETRACTING:
                    if(currentRuntime > timerStart+retractingTime)
                    {
                        capstoneState = CapstoneState.RETRACTED;
                        deployCapstone = false;
                    }
                default:
                    break;
            }
        }

    }
}
