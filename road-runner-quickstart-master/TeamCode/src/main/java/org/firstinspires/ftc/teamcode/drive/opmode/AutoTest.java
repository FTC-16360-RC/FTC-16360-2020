package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
public class AutoTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        Trajectory traj = drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(30, 30), 0)
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(traj.end())
                .lineToSplineHeading(new Pose2d(0, 60, Math.toRadians(90)))
                .build();

        Trajectory traj3 = drive.trajectoryBuilder(traj2.end(), true)
                .splineTo(new Vector2d(-30, 30), 135)
                .build();

        Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
                .lineToConstantHeading(new Vector2d(0, 0))
                .build();

        drive.followTrajectory(traj);

        sleep(500);

        drive.followTrajectory(traj2);

        sleep(500);

        drive.followTrajectory(traj3);

        sleep(500);

        drive.followTrajectory(traj4);
    }
}
