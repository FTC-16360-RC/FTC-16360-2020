package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config
@Autonomous(group = "drive")
public class Autonomous_for_blue extends LinearOpMode {


    private int rings = 0;


    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d startPose = new Pose2d(62, 23, Math.toRadians(180));
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startPose);

        Trajectory trajectory1 = drive.trajectoryBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(0, 20, Math.toRadians(180)))
                .build();
        Trajectory trajectory2 = drive.trajectoryBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(0, 38, Math.toRadians(90)))
                .build();
        Trajectory trajectory3 = drive.trajectoryBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(-23.5, 15, Math.toRadians(90)))
                .build();
        Trajectory trajectory4 = drive.trajectoryBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(-28, 38, Math.toRadians(90)))
                .build();
        Trajectory trajectory5 = drive.trajectoryBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(36, 47.5, Math.toRadians(180)))
                .build();
        Trajectory trajectory6 = drive.trajectoryBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(0, 38, Math.toRadians(90)))
                .build();

        waitForStart();
        //look for the rings

        drive.followTrajectory(trajectory1);//go to the line to shoot
        sleep(4000);

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

