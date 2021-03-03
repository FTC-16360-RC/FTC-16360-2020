package org.firstinspires.ftc.teamcode.lib.hardware;

import com.acmerobotics.roadrunner.drive.Drive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.lib.PoseStorage;

import java.util.List;

public class Robot {
    HardwareMap hardwareMap;

    private DcMotorEx shooterMotor1, shooterMotor2, intakeMotor, transferMotor;

    public SampleMecanumDrive drive;
    public Shooter shooter;
    public Intake intake;
    public Transfer transfer;

    public enum RobotState

    public Robot(HardwareMap hardwareMap, boolean autonomous) {
        this.hardwareMap = hardwareMap;

        //initialize hardware classes
        drive = new SampleMecanumDrive(hardwareMap);
        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
        transfer = new Transfer(hardwareMap);


        //turn on bulk reading
        for (LynxModule module : this.hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }

    public void update() {
        // update shooter pidf in the background
        shooter.update();

        // We update drive continuously in the background, regardless of state
        drive.update();

        // Read pose
        Pose2d poseEstimate = drive.getPoseEstimate();

        // Continually write pose to PoseStorage
        PoseStorage.currentPose = poseEstimate;
    }
}
