package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Shooter {

    // Define 3 states. on, off or idle
    public enum Mode {
        SHOOTING,
        IDLE,
        OFF
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

    private final double flapMinPosition = 0.49;
    private final double flapMaxPosition = 0.35;

    private double targetVelocity;

    public Shooter(HardwareMap hardwaremap) {
        shooter1 = hardwaremap.get(DcMotorEx.class, "shooter1");
        shooter2 = hardwaremap.get(DcMotorEx.class, "shooter2");
        feeder = hardwaremap.get(Servo.class, "feeder");
        flap = hardwaremap.get(Servo.class, "flap");
        feeder.setPosition(feederStartPosition);
        flap.setPosition(flapMinPosition);
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

    public void setTargetVolicty(double targetVelocity) {
        this.targetVelocity = targetVelocity;
    }

    public void shoot() {
        if(feederState == FeederState.RETRACTED) {
            feeder.setPosition(feederExtendedPosition);
            startTime = currentRuntime;
            feederState = FeederState.PUSHING;
        }
    }

    public double getFlapPosition() {
        return Math.round((flapMinPosition-flap.getPosition())/(flapMinPosition-flapMaxPosition)*100);
    }

    public void setFlapPosition(double targetPosition) {
        flap.setPosition(flapMinPosition+(flapMaxPosition-flapMinPosition)*targetPosition);
    }

    public void reset() { //ONLY for autonomous
        feeder.setPosition(feederStartPosition);
        feederState = FeederState.RETRACTED;
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
        if(feederState == FeederState.PUSHING && (this.currentRuntime-startTime > actuationTime)) {
            feeder.setPosition(feederStartPosition);
            startTime = this.currentRuntime;
            feederState = FeederState.RETRACTING;
        }
        if(feederState == FeederState.RETRACTING && (this.currentRuntime-startTime > actuationTime)) {
            feederState = FeederState.RETRACTED;
        }
    }

}