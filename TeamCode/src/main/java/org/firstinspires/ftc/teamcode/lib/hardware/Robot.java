package org.firstinspires.ftc.teamcode.lib.hardware;

import com.acmerobotics.roadrunner.drive.Drive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.lib.AutoAim;
import org.firstinspires.ftc.teamcode.lib.Globals;
import org.firstinspires.ftc.teamcode.lib.PoseStorage;

import java.util.List;

public class Robot {
    protected HardwareMap hardwareMap;

    public SampleMecanumDrive drive;
    protected Shooter shooter;
    protected Intake intake;
    protected Transfer transfer;
    protected Wobble wobble;
    protected AutoAim autoAim;

    public enum RobotState {
        INTAKING,
        DRIVING,
        AIMING,
        SHOOTING
    }

    protected static RobotState robotState;

    protected Pose2d poseEstimate = new Pose2d();

    public Robot(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;

        //initialize hardware classes
        drive = new SampleMecanumDrive(hardwareMap);
        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
        transfer = new Transfer(hardwareMap);
        wobble = new Wobble(hardwareMap);

        // initialize aiming class
        autoAim = new AutoAim();

        // set robot state to idle
        robotState = RobotState.DRIVING;

        // set robot pose
        drive.setPoseEstimate(PoseStorage.currentPose);

        // Velocity control per wheel is not necessary outside of motion profiled auto
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //turn on bulk reading
        for (LynxModule module : this.hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }

    public void setRobotState(RobotState desiredRobotState) {
        robotState = desiredRobotState;
        switch(desiredRobotState) {
            case INTAKING:
                intake();
                shooter.setMode(Shooter.Mode.IDLE);
                autoAim.setCurrentMode(AutoAim.Mode.NORMAL_CONTROL);
                break;
            case DRIVING:
                transferIdle();
                shooter.setMode(Shooter.Mode.IDLE);
                autoAim.setCurrentMode(AutoAim.Mode.NORMAL_CONTROL);
                break;
            case AIMING:
                Globals.updateTarget();
                autoAim.setCurrentMode(Globals.currentAimingMode);
                intakeIdle();
                shooter.setMode(Shooter.Mode.SHOOTING);
                break;
            case SHOOTING:
                transferIdle();
                shooter.setMode(Shooter.Mode.SHOOTING);
                break;
        }
    }

    public RobotState getRobotState() {
        return robotState;
    }

    public void update() {
        // update shooter pidf in the background
        shooter.update();

        // very important to make sure nothing breaks
        if(intake.getMode() == Intake.Mode.FORWARD && transfer.getMode() != Transfer.Mode.FORWARD) {
            transfer.setMode(Transfer.Mode.FORWARD);
        }

        if(transfer.getMode() == Transfer.Mode.REVERSE && intake.getMode() != Intake.Mode.REVERSE) {
            intake.setMode(Intake.Mode.REVERSE);
        }

        // We update drive continuously in the background, regardless of state
        drive.update();
        //drive.getLocalizer().update();

        // Read pose
        poseEstimate = drive.getPoseEstimate();

        // Continually write pose to PoseStorage
        PoseStorage.currentPose = poseEstimate;
    }

    public Pose2d getPoseEstimate() {
        return poseEstimate;
    }

    public double getShooterRPM() {
        return shooter.getShooterVelocity();
    }

    public double getTargetRPM() {
        return shooter.getTargetVelocity();
    }

    /*
    ** ROBOT CONTROL METHODS
     */

    // wobble methods
    public void wobbleIntakingPos() {
        wobble.setArmState(Wobble.ArmState.INTAKE);
        wobble.setGripperState(Wobble.GripperState.OPEN);
    }

    public void wobbleOuttakingPos() {
        wobble.setArmState(Wobble.ArmState.OUTTAKE);
        wobble.setGripperState(Wobble.GripperState.CLOSED_TIGHT);
    }

    public void wobbleDroppingPos() {
        wobble.setArmState(Wobble.ArmState.RELEASE);
        wobble.setGripperState(Wobble.GripperState.CLOSED_TIGHT);
    }

    public void wobbleStoringPos() {
        wobble.setArmState(Wobble.ArmState.STORED);
    }

    public void wobbleGrab() {
        wobble.setGripperState(Wobble.GripperState.CLOSED_TIGHT);
    }

    public void wobbleLoosenGrip() {
        wobble.setGripperState(Wobble.GripperState.CLOSED_LOOSE);
    }

    public void wobblegripperOpen() {
        wobble.setGripperState(Wobble.GripperState.OPEN);
    }


    // intake/transfer methods
    public void intake() {
        intake.setMode(Intake.Mode.FORWARD);
        transfer.setMode(Transfer.Mode.FORWARD);
    }

    public void reverseIntake() {
        intake.setMode(Intake.Mode.REVERSE);
    }

    public void reverseTransfer() {
        transfer.setMode(Transfer.Mode.REVERSE);
        intake.setMode(Intake.Mode.REVERSE);
    }

    public void intakeIdle() {
        intake.setMode(Intake.Mode.IDLE);
    }

    public void transferIdle() {
        transfer.setMode(Transfer.Mode.IDLE);
        intake.setMode(Intake.Mode.IDLE);
    }


    // shooter methods
    public void shoot() {
        if(robotState == RobotState.SHOOTING) { //  && AutoAim.getHeadingError() < 5
            shooter.shoot();
        }
    }

    // shooter method for autonomous
    public void forceShoot() {
        shooter.shoot();
    }
}
