package org.firstinspires.ftc.teamcode.lib.hardware;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Wobble {
    private Servo wobbleArm1, wobbleArm2, wobbleGripper;

    // define servo upper and lower boundaries wobbleArm2 is 1-wobbleArm1
    private final double wobbleArm1StartPos = 0;
    private final double wobbleArm1lowerPos = 0;
    private final double wobbleGripperOpen = 0;
    private final double wobbleGripperClosed = 0;

    public enum ArmState {
        START_POS,
        INTAKE,
        OUTTAKE,
        STORED,
        RELEASE
    }

    public enum GripperState {
        OPEN,
        CLOSED_TIGHT,
        CLOSED_LOOSE
    }

    private ArmState armState;

    private GripperState gripperState;

    public Wobble(HardwareMap hardwareMap) {
        wobbleArm1 = hardwareMap.get(Servo.class, "wobbleArmLeft");
        wobbleArm2 = hardwareMap.get(Servo.class, "wobbleArmRight");
        wobbleGripper = hardwareMap.get(Servo.class, "wobbleGripper");
    }

    // in between bounds, automatically sets second servo according to first one
    private void setWobbleArmPosition (double targetPosition) {
        wobbleArm1.setPosition(targetPosition*(wobbleArm1lowerPos-wobbleArm1StartPos) + wobbleArm1StartPos);
        wobbleArm2.setPosition(1-wobbleArm1.getPosition()); //since they are mirrored, boundaries should match
    }

    // in between bounds
    private void setWobbleGripperPosition (double targetPosition) {
        wobbleGripper.setPosition(targetPosition*(wobbleGripperOpen-wobbleGripperClosed) + wobbleGripperClosed);

    }

    public ArmState getArmState() {
        return armState;
    }

    public void setArmState(ArmState armState) {
        this.armState = armState;
        switch (armState) {
            case INTAKE:
                setWobbleArmPosition(0);
                break;
            case STORED:
                setWobbleArmPosition(0.25);
                break;
            case RELEASE:
                setWobbleArmPosition(0.5);
                break;
            case OUTTAKE:
                setWobbleArmPosition(0.75);
                break;
            case START_POS:
                setWobbleArmPosition(1);
                break;
        }
    }

    public GripperState getGripperState() {
        return gripperState;
    }

    public void setGripperState(GripperState gripperState) {
        this.gripperState = gripperState;
        switch (gripperState) {
            case OPEN:
                setWobbleGripperPosition(0);
                break;
            case CLOSED_LOOSE:
                setWobbleGripperPosition(0.5);
                break;
            case CLOSED_TIGHT:
                setWobbleGripperPosition(1);
                break;
        }
    }
}
