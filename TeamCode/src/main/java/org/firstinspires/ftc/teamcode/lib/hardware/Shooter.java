package org.firstinspires.ftc.teamcode.lib.hardware;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lib.Globals;
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
    private double lastTargetVelocity = 0.0;

    private double currentVelocity = 0.0;

    // Our velocity controller
    private final VelocityPIDFController veloController = new VelocityPIDFController(MOTOR_VELO_PID, kV, kA, kStatic);

    private final FtcDashboard dashboard = FtcDashboard.getInstance();

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
        return Globals.ticksPerSecondToRpm(currentVelocity, 1);
    }

    public double getTargetVelocity() {
        return Globals.ticksPerSecondToRpm(targetVelocity, 1);
    }

    public void setTargetVolicty(double targetVelocity) {
        this.targetVelocity = Globals.rpmToTicksPerSecond(targetVelocity, 1);
    }

    public void shoot(boolean onTarget) {
        // check if all requirements are met
        if(feederState == FeederState.RETRACTED && mode == Mode.SHOOTING && targetVelocity * 0.95 <= currentRuntime && currentVelocity <= targetVelocity * 1.05 && onTarget) {
            feeder.setPosition(feederExtendedPosition);
            feederTimer.reset();
            feederState = FeederState.PUSHING;
        }
    }

    public void setMode(Mode mode) {
        this.mode = mode;
        if(this.mode == Mode.SHOOTING)
            veloTimer.reset();
    }

    public void update() {
        // Get the velocity from the motor with the encoder
        currentVelocity = shooter1.getVelocity();

        //packet for dashboard graph
        TelemetryPacket packet = new TelemetryPacket();

        // values to make graph look better
        packet.put("lower bound", 0.0);
        packet.put("upper bound", 6000.0);

        switch (this.mode)
        {
            case IDLE: // no power
                shooter1.setPower(0);
                shooter2.setPower(0);
                break;
            case COASTING: // coasting power to keep inertia
                shooter1.setPower(0.3);
                shooter2.setPower(0.3);
                break;
            case SHOOTING: // shooting speed including pidf
                // Call necessary controller methods
                veloController.setTargetVelocity(targetVelocity);
                veloController.setTargetAcceleration((targetVelocity - lastTargetVelocity) / veloTimer.seconds());
                veloTimer.reset();

                lastTargetVelocity = targetVelocity;

                // Get the position from the motor with the encoder
                double motorPos = shooter1.getCurrentPosition();

                // Update the controller and set the power for each motor
                double power = veloController.update(motorPos, currentVelocity);
                shooter1.setPower(power);
                shooter2.setPower(power);
                break;
        }
        packet.put("currentVelo", Globals.ticksPerSecondToRpm(currentVelocity, 1));
        packet.put("targetVelo", Globals.ticksPerSecondToRpm(targetVelocity, 1));

        // control feeder arm
        if(feederState == FeederState.PUSHING && feederTimer.seconds() > actuationTime) {
            feeder.setPosition(feederStartPosition);
            feederTimer.reset();
            feederState = FeederState.RETRACTING;
        }
        if(feederState == FeederState.RETRACTING && feederTimer.seconds() > actuationTime) {
            feederState = FeederState.RETRACTED;
        }

        // send shooter speeds to dashboard
        dashboard.sendTelemetryPacket(packet);
    }

}

