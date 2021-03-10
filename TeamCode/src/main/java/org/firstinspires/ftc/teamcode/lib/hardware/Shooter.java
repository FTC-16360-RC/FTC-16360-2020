package org.firstinspires.ftc.teamcode.lib.hardware;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.arcrobotics.ftclib.util.InterpLUT; // gsehsch das?
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.lib.Globals;
import org.firstinspires.ftc.teamcode.lib.Targets;
import org.firstinspires.ftc.teamcode.lib.VelocityPIDFController;

@Config
public class Shooter {
    // PID coefficients
    public static PIDCoefficients MOTOR_VELO_PID = new PIDCoefficients(0.0038, 0, 0.000015);

    // feedforward gains
    public static double kV = 0.00034;
    public static double kA = 0.000135;
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
        //COASTING
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

    private double actuationTime = 0.22;
    private final ElapsedTime feederTimer = new ElapsedTime();

    private final double feederStartPosition = 0.25;
    private final double feederExtendedPosition = 0.42;

    private final double flapRestPosition = 0.7;

    private double targetVelocity;
    private static double distance;

    //Init the Look up table
    InterpLUT lutHighgoal = new InterpLUT();
    InterpLUT lutPowershots = new InterpLUT();


    public Shooter(HardwareMap hardwaremap) {
        // motors
        shooter1 = hardwaremap.get(DcMotorEx.class, "shooter1");
        shooter2 = hardwaremap.get(DcMotorEx.class, "shooter2");

        //shooter1.setDirection(DcMotorEx.Direction.REVERSE);
        //shooter2.setDirection(DcMotorEx.Direction.REVERSE);

        shooter1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        shooter2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        shooter1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        shooter2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        // servos
        feeder = hardwaremap.get(Servo.class, "feeder");
        flap = hardwaremap.get(Servo.class, "flap");

        feeder.setPosition(feederStartPosition);
        flap.setPosition(flapRestPosition);
        feederState = FeederState.RETRACTED;

        mode = Mode.IDLE;

        // values for high goal lut
        lutHighgoal.add(-1000000, 0.5);
        lutHighgoal.add(70, 0.52);
        lutHighgoal.add(75, 0.535);
        lutHighgoal.add(80, 0.545);
        lutHighgoal.add(85, 0.555);
        lutHighgoal.add(90, 0.57);
        lutHighgoal.add(95, 0.575);
        lutHighgoal.add(100, 0.57);
        lutHighgoal.add(105, 0.58);
        lutHighgoal.add(110, 0.585);
        lutHighgoal.add(115, 0.585);
        lutHighgoal.add(120, 0.59);
        lutHighgoal.add(125, 0.59);
        lutHighgoal.add(200000000, 0.58);

        //generating final equation for lutHighgoal
        lutHighgoal.createLUT();

        // same for power shots
        lutPowershots.add(5, 1);
        lutPowershots.add(4.1, 0.9);
        lutPowershots.add(3.6, 0.75);
        lutPowershots.add(2.7, .5);
        lutPowershots.add(1.1, 0.2);

        //generating final equation for lutPowershots
        //lutPowershots.createLUT();
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

    public static void setDistance(double distance) {
        Shooter.distance = distance;
    }

    public void shoot() {
        // check if all requirements are met
        if(feederState == FeederState.RETRACTED && mode == Mode.SHOOTING ) {//&& targetVelocity * 0.95 <= currentVelocity && currentVelocity <= targetVelocity * 1.05) {
            feeder.setPosition(feederExtendedPosition);
            feederTimer.reset();
            feederState = FeederState.PUSHING;
        }
    }

    public void setMode(Mode mode) {
        this.mode = mode;
        if (mode == Mode.SHOOTING) {
            veloTimer.reset();
        }
    }

    public void update() {
        // Get the velocity from the motor with the encoder
        currentVelocity = shooter1.getVelocity();

        if(Globals.currentTargetType == Targets.TargetType.HIGHGOAL) {
            this.targetVelocity = Globals.rpmToTicksPerSecond(Globals.highGoalRPM, 1);
        } else {
            this.targetVelocity = Globals.rpmToTicksPerSecond(Globals.powerShotRPM, 1);
        }

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
            case SHOOTING: // shooting speed including pidf
                // flap
                if(Globals.currentTargetType == Targets.TargetType.HIGHGOAL) {
                    flap.setPosition(lutHighgoal.get(distance));
                } else {
                    flap.setPosition(lutHighgoal.get(distance));//lutPowershots.get(distance));
                }

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
