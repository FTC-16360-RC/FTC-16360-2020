package org.firstinspires.ftc.teamcode.control;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Extension {

    private Servo extension;

    public enum ExtensionState
    {
        EXTENDED,
        EXTENDING,
        RETRACTED,
        RETRACTING,
        PUSHING
    }

    private boolean stateChanged = false;

    private double extendingTime = 0.5;
    private double currentRuntime = 0;
    private double timerStart = 0;

    private ExtensionState extensionState = ExtensionState.RETRACTED;

    /*
    //  METHODS
     */

    //constructor
    public Extension(HardwareMap hardwareMap)
    {
        //configure intake motors
        extension = hardwareMap.get(Servo.class, "extension");

        //set extension to fit in space
        extension.setPosition(0.4);
    }

    public void extend()
    {
        extensionState = ExtensionState.EXTENDING;
        stateChanged = true;
    }

    public void retract()
    {
        extensionState = ExtensionState.RETRACTING;
        stateChanged = true;
    }

    public void push()
    {
        extension.setPosition(0.6);
        extensionState = ExtensionState.PUSHING;
    }

    public void forceRetract()
    {
        extension.setPosition(0.13); // 0.05
        extensionState = ExtensionState.RETRACTED;
    }

    public void update(double currentRuntime, boolean gripperOpened, boolean liftExtended)
    {
        this.currentRuntime = currentRuntime;
        if(stateChanged) {
            if (extensionState == ExtensionState.EXTENDING && liftExtended) //only extend if the lift is extended
            {
                extension.setPosition(0.695); //0.62
                timerStart = currentRuntime;
                stateChanged = false;
            }
            if (extensionState == ExtensionState.RETRACTING && gripperOpened) //only retract if the gripper is opened
            {
                extension.setPosition(0.13);
                timerStart = currentRuntime;
                stateChanged = false;
            }
        } else if(currentRuntime > timerStart + extendingTime)
        {
            if(extensionState == ExtensionState.EXTENDING)
            {
                extensionState = ExtensionState.EXTENDED;
            }
            if(extensionState == ExtensionState.RETRACTING)
            {
                extensionState = ExtensionState.RETRACTED;
            }
        }
    }

    public ExtensionState getExtensionState() {

        return extensionState;
    }
}
