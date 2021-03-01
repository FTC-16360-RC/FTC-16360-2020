package org.firstinspires.ftc.teamcode.lib.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.lib.VelocityPIDFController;
import org.firstinspires.ftc.teamcode.lib.datatypes.UTuple;

import java.util.ArrayList;

@Config
public class Shooter {
    // PID coefficients
    public static PIDCoefficients MOTOR_VELO_PID = new PIDCoefficients(0, 0, 0);

    // feedforward gains
    public static double kV = 1 / TuningController.rpmToTicksPerSecond(TuningController.MOTOR_MAX_RPM);
    public static double kA = 0;
    public static double kStatic = 0;

    // Timer for calculating desired acceleration
    // Necessary for kA to have an affect
    private final ElapsedTime veloTimer = new ElapsedTime();
    private double lastTargetVelo = 0.0;

    // Our velocity controller
    private final VelocityPIDFController veloController = new VelocityPIDFController(MOTOR_VELO_PID, kV, kA, kStatic);

    // Define 3 states. on, off or coast
    public enum Mode {
        SHOOTING,
        IDLE,
        COASTING
    }

    private enum FeederState {
        PUSHING,
        RETRACTING,
        RETRACTED
    }

    private DcMotorEx shooter1, shooter2;

    private Servo feeder, flap;

    private Mode mode;

    private FeederState feederState;

    private double currentRuntime;
    private double actuationTime = 0.25;
    private final ElapsedTime feederTimer = new ElapsedTime();

    private final double feederStartPosition = 0.33;
    private final double feederExtendedPosition = 0.5;

    private final double flapRestPosition = 0.59;

    private double targetVelocity;

    public Shooter(HardwareMap hardwaremap) {
        // motors
        shooter1 = hardwaremap.get(DcMotorEx.class, "shooter1");
        shooter2 = hardwaremap.get(DcMotorEx.class, "shooter2");

        //shooter1.setDirection(DcMotorEx.Direction.REVERSE);
        //shooter2.setDirection(DcMotorEx.Direction.REVERSE);

        shooter1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        shooter1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        shooter2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        // servos
        feeder = hardwaremap.get(Servo.class, "feeder");
        flap = hardwaremap.get(Servo.class, "flap");

        feeder.setPosition(feederStartPosition);
        flap.setPosition(flapRestPosition);
        feederState = FeederState.RETRACTED;

        mode = Mode.IDLE;
    }

    public Mode getMode() {
        return mode;
    }

    public double getShooterVelocity() {
        return shooter1.getVelocity();
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
            //startTime = currentRuntime;
            feederState = FeederState.PUSHING;
        }
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

