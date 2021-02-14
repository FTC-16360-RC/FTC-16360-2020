package org.firstinspires.ftc.teamcode.newLib.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.lib.Targets;
import org.firstinspires.ftc.teamcode.newLib.Comms;

@Config
public class Shooter {

    // Define 3 states. on, off or reverse
    public enum Mode {
        SHOOTING,
        IDLE,
    }

    private enum FeederState {
        PUSHING,
        RETRACTING,
        RETRACTED
    }

    private DcMotorEx shooter1;
    private DcMotorEx shooter2;

    private Servo feeder;
    private Servo flap;

    private Mode mode;

    private FeederState feederState;

    private double currentRuntime;
    private double actuationTime = 0.19;
    private double startTime;

    private final double feederStartPosition = 0.33;
    private final double feederExtendedPosition = 0.5;

    private double targetVelocity;

    private double flapPos = 0.6;

    public Shooter() {
        shooter1 = Comms.hardwareMap.get(DcMotorEx.class, "shooter1");
        shooter2 = Comms.hardwareMap.get(DcMotorEx.class, "shooter2");
        feeder = Comms.hardwareMap.get(Servo.class, "feeder");
        flap = Comms.hardwareMap.get(Servo.class, "flap");
        feeder.setPosition(feederStartPosition);
        feederState = FeederState.RETRACTED;
        shooter1.setDirection(DcMotorEx.Direction.FORWARD);
        shooter2.setDirection(DcMotorEx.Direction.FORWARD);
        shooter1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooter1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        shooter2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        shooter1.setPower(0);
        shooter2.setPower(0);
        mode = Mode.IDLE;

        setTargetVelocity(5000);
    }

    public Mode getMode() {
        return mode;
    }

    public double getShooterVelocity() {
        return 0.5*(shooter1.getVelocity() + shooter2.getVelocity());
    }
    
    public double getTargetVelocity() {
        return targetVelocity;
    }

    public void setTargetVelocity(double targetVelocity) {
        this.targetVelocity = targetVelocity;
    }

    public void shoot() {
        if(feederState == FeederState.RETRACTED && shooter1.getPower() != 0) {
            feeder.setPosition(feederExtendedPosition);
            startTime = currentRuntime;
            feederState = FeederState.PUSHING;
        }
    }

    public void reset() { //ONLY for autonomous
        feeder.setPosition(feederStartPosition);
        feederState = FeederState.RETRACTED;
    }

    //flap
    public void setFlapPos(double flapPos) {
        this.flapPos = flapPos;
    }

    public void toggleFlap(Boolean enabled) {
        if (enabled) {
            flap.setPosition(flapPos);
        } else {
            flap.setPosition(0.51);
        }
    }

    public void updateFlapPos(double[] position) {
        double distance = Math.sqrt(Math.pow(position[0] - Targets.currentTarget.getX(), 2)
                + Math.pow(position[1] - Targets.currentTarget.getY(), 2));
        flapPos = flapPos(distance);
    }

    private double flapPos(double distance) {
        double baseFlapValue = 0;
        double a = 0;
        double b = 0;
        return baseFlapValue + a * Math.sqrt(b * distance);
    }

    public void setMode(Mode mode) {
        this.mode = mode;
        switch (this.mode)
        {
            case IDLE: //no power
                shooter1.setPower(0);
                shooter2.setPower(0);
                break;
            case SHOOTING: //shoot
                shooter1.setVelocity(targetVelocity);
                shooter2.setVelocity(targetVelocity);
                break;
        }
    }

    public void update(double currentRuntime) {
        this.currentRuntime = currentRuntime;
    }
}