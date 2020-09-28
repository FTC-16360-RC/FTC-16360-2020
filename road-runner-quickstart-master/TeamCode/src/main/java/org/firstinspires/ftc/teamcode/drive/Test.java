package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class Test extends LinearOpMode {
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        StandardTrackingWheelLocalizer myLocalizer = new StandardTrackingWheelLocalizer(hardwareMap);
        myLocalizer.setPoseEstimate(new Pose2d(10, 10, Math.toRadians(90)) );

        waitForStart();

        while(opModeIsActive()){
            myLocalizer.update();

            Pose2d myPose = myLocalizer.getPoseEstimate();

            telemetry.addData("x", myPose.getX());
            telemetry.addData("y", myPose.getY());
            telemetry.addData("heading", myPose.getHeading());
        }

        if (isStopRequested()) return;

        Trajectory traj = drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(30, 30), 0)
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(traj.end())
                .lineToSplineHeading(new Pose2d(0, 60, Math.toRadians(90)))
                .build();

        Trajectory traj3 = drive.trajectoryBuilder(traj2.end(), true)
                .splineToSplineHeading(new Pose2d(-30, 30, Math.toRadians(0)), 0)
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
