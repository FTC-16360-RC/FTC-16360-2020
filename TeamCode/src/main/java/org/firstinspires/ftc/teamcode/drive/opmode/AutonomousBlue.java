package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import org.firstinspires.ftc.teamcode.drive.opmode.Shooter;
import com.acmerobotics.roadrunner.control.PIDFController;


import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.lib.PoseStorage;

@Config
@Autonomous(group = "drive")
public class AutonomousBlue extends LinearOpMode {


    private int rings = 0;
    private Shooter shooter;


    @Override
    public void runOpMode() throws InterruptedException {
        shooter = new Shooter(hardwareMap);
        Pose2d startPose = new Pose2d(-62, 23, Math.toRadians(0));
        PoseStorage.currentPose = startPose;
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Vector2d PowerShot1 = new Vector2d(72, 21);
        Vector2d PowerShot2 = new Vector2d(72, 13.5);
        Vector2d PowerShot3 = new Vector2d(72, 6);


        drive.setPoseEstimate(startPose);

        Trajectory trajectory1 = drive.trajectoryBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(0, 20, Math.toRadians(180)))
                .build();
        Trajectory trajectory2 = drive.trajectoryBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(0, 38, Math.toRadians(270)))
                .build();
        Trajectory trajectory3 = drive.trajectoryBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(23.5, 15, Math.toRadians(270)))
                .build();
        Trajectory trajectory4 = drive.trajectoryBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(28, 38, Math.toRadians(270)))
                .build();
        Trajectory trajectory5 = drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(-36, 37), Math.toRadians(180))
                .splineTo(new Vector2d(-36, 47.5), Math.toRadians(180))
                .build();
        Trajectory trajectory6 = drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(-36, 37), Math.toRadians(180))
                .splineTo(new Vector2d(0, 38), Math.toRadians(270))
                .build();

        waitForStart();
        //look for the rings
        Trajectory trajectory7 = drive.trajectoryBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(-10, 21, Math.toRadians(-10)))
                .build();
        drive.followTrajectory(trajectory7);
        shooter.setTargetVolicty(3000);
        shooter.setMode(Shooter.Mode.SHOOTING);
        shooter.setFlapPosition(0.5);
        sleep(1000);
        shooter.shoot();
        sleep(200);
        shooter.reset();
        shooter.setFlapPosition(0);

        Trajectory trajectory8 = drive.trajectoryBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(-10, 13, Math.toRadians(-10)))
                .build();
        drive.followTrajectory(trajectory8);
        shooter.setFlapPosition(0.5);
        sleep(200);
        shooter.shoot();
        sleep(200);
        shooter.reset();
        shooter.setFlapPosition(0);
        Trajectory trajectory9 = drive.trajectoryBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(-10, 6, Math.toRadians(-10)))
                .build();
        drive.followTrajectory(trajectory9);
        shooter.setFlapPosition(0.5);
        sleep(200);
        shooter.shoot();
        sleep(200);
        shooter.reset();
        shooter.setMode(Shooter.Mode.IDLE);

        switch (rings) {
            case (0):
                drive.followTrajectory(trajectory2);//go to the square for the wobble goal
                break;
            case (1):
                drive.followTrajectory(trajectory3);//go to the square for the wobble goal
                break;
            case (4):
                drive.followTrajectory(trajectory4);//go to the square for the wobble goal
                break;
        }
        sleep(4000);

        //wobble goal
        drive.followTrajectory(trajectory5);//go to pick up the second wobble goal
        sleep (4000);
        switch (rings) {
            case (0):
                drive.followTrajectory(trajectory2);//go to the square for the second wobble goal
                break;
            case (1):
                drive.followTrajectory(trajectory3);//go to the square for the second wobble goal
                break;
            case (4):
                drive.followTrajectory(trajectory4);//go to the square for the second wobble goal
                break;
        }
        sleep(4000);
        // wobble goal
        drive.followTrajectory(trajectory6);//go to launch line
    }
}

