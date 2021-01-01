package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class Shooter {

    // Define 3 states. on, off or reverse
    public enum Mode {
        SHOOTING,
        IDLE,
    }

    private DcMotorEx shooter1;
    private DcMotorEx shooter2;

    private Mode mode;

    private double targetVelocity;

    public Shooter(HardwareMap hardwaremap) {
        shooter1 = hardwaremap.get(DcMotorEx.class, "shooter1");
        shooter2 = hardwaremap.get(DcMotorEx.class, "shooter2");
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

}