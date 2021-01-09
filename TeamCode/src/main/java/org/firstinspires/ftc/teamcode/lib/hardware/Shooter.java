package org.firstinspires.ftc.teamcode.lib.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

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

    private Mode mode;

    private FeederState feederState;

    private double currentRuntime;
    private double actuationTime = 0.19;
    private double startTime;

    private final double feederStartPosition = 0.33;
    private final double feederExtendedPosition = 0.5;

    private double targetVelocity;

    public Shooter(HardwareMap hardwaremap) {
        shooter1 = hardwaremap.get(DcMotorEx.class, "shooter1");
        shooter2 = hardwaremap.get(DcMotorEx.class, "shooter2");
        feeder = hardwaremap.get(Servo.class, "feeder");
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

    public TUtil update(double currentRuntime, TUtil instructions) {
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
                    break;
                case SET_SHOOTER_ON:
                    setMode(Mode.SHOOTING);
                    break;
                case RESET_SHOOTER:
                    reset();
                    break;
                case SET_SHOOTER_VELOCITY:
                    setTargetVolicty(i.b_dbl);
                    break;
                case GET_SHOOTER_VELOCITY:
                    if(i.b_adr != null) {
                        messages.add(i.b_adr, Instructions.RETURN_SHOOTER_VELOCITY, getShooterVelocity());
                    }
                    break;
                case SHOOT_ONE:
                    shoot();
                    break;
                case SHOOT_THREE:
                    shoot();
                    shoot();
                    shoot();
                    break;

                default:
                    break;
            }
        }
        return messages;
    }
}