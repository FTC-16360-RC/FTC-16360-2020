package org.firstinspires.ftc.teamcode.lib.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.lib.Globals;
import org.firstinspires.ftc.teamcode.lib.Targets;
import org.firstinspires.ftc.teamcode.lib.datatypes.TUtil;
import org.firstinspires.ftc.teamcode.lib.datatypes.UTuple;
import org.firstinspires.ftc.teamcode.lib.datatypes.util.Instructions;
import org.firstinspires.ftc.teamcode.lib.hardware.Intake;

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
        HardwareMap hardwareMap = Globals.hardwareMap;
        shooter1 = hardwareMap.get(DcMotorEx.class, "shooter1");
        shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");
        feeder = hardwareMap.get(Servo.class, "feeder");
        flap = hardwareMap.get(Servo.class, "flap");
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
        return 0.56 + Math.sqrt(0.0005 * (distance-185)/30);
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

    public TUtil update(TUtil instructions, double currentRuntime) {
        this.currentRuntime = currentRuntime;
        if(feederState == FeederState.PUSHING && (this.currentRuntime-startTime > actuationTime)) {
            feeder.setPosition(feederStartPosition);
            startTime = this.currentRuntime;
            feederState = FeederState.RETRACTING;
        }
        if(feederState == FeederState.RETRACTING && (this.currentRuntime-startTime > actuationTime)) {
            feederState = FeederState.RETRACTED;
        }

        TUtil messages = new TUtil();
        for (UTuple i : instructions.list) {
            switch (i.a_ins) {
                case LOWER_INTAKE_DEBUG:
                    break;
                case SET_SHOOTER_IDLE:
                    setMode(Mode.IDLE);
                    toggleFlap(false);
                    break;
                case SET_SHOOTER_ON:
                    setMode(Mode.SHOOTING);
                    break;
                case RESET_SHOOTER:
                    reset();
                    break;
                case SET_SHOOTER_VELOCITY:
                    setTargetVelocity(i.b_dbl);
                    break;
                case GET_SHOOTER_VELOCITY:
                    if(i.b_adr != null) {
                        messages.add(i.b_adr, Instructions.RETURN_SHOOTER_VELOCITY, getShooterVelocity());
                    }
                    break;
                case SHOOT_ONE:
                    shoot();
                    toggleFlap(true);
                    break;
                case SET_FLAP_POSITION:
                    setFlapPos(i.b_dbl);
                    break;
                case RECIEVE_POSITION:
                    updateFlapPos(i.b_arr);
                    break;
                case ADJUST_FLAP_DEBUG:
                    toggleFlap(false);
                    toggleFlap(true);
                default:
                    break;
            }
        }
        return messages;
    }
}