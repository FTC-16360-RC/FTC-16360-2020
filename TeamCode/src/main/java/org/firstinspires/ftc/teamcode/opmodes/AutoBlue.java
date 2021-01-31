package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmode.Shooter;
import org.firstinspires.ftc.teamcode.lib.Globals;
import org.firstinspires.ftc.teamcode.lib.PoseStorage;
import org.firstinspires.ftc.teamcode.lib.Targets;

@Autonomous(group = "opmodes")
public class AutoBlue extends LinearOpMode {

    // This enum defines our "state"
    // This is defines the possible steps our program will take
    enum State {
        TRAJECTORY_1,   // drive to the shooting spot for power shots
        WAIT_1,         // waiting for shooter
        TURN_1,         // turn to second power shot
        WAIT_2,         // waiting for shooter
        TURN_2,         // turn to third power shot
        WAIT_3,         // waiting for shooter
        TRAJECTORY_2,   // deposit first wobble goal
        WAIT_4,         // waiting to deposit wobble goal
        TRAJECTORY_3,   // pick up second wobble goal
        WAIT_5,         // waiting to pick up wobble
        TRAJECTORY_4,   // deposit second wobble goal
        WAIT_6,         // waiting to deposit second wobble goal
        TRAJECTORY_5,   // go to line
        IDLE,           // Our bot will enter the IDLE state when done
        SHOOT_1,        // shoot the first ring
        WAIT_7,         // wait to shoot the second ring
        SHOOT_2,        // shoot the second ring
        WAIT_8,         // wait to shoot the third ring
        SHOOT_3         // shoot the third ring
    }

    // We define the current state we're on
    // Default to IDLE
    State currentState = State.IDLE;

    // Define our start pose
    Pose2d startPose = new Pose2d(-62, 23, Math.toRadians(0));

    @Override
    public void runOpMode() throws InterruptedException {
        // Variable for rings
        int rings = 0;

        // Initialize the shooter
        Shooter shooter = new Shooter(hardwareMap);

        // Initialize SampleMecanumDrive
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // Set initial pose
        drive.setPoseEstimate(startPose);

        // Save initial pose to PoseStorage
        PoseStorage.currentPose = startPose;

        // first trajectory moves to first power shot
        Trajectory trajectory1 = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(0, 20, Math.toRadians(0+Globals.headingError)))
                .build();

        // trajectory moves to the spot to shoot at the high goal
        Trajectory trajectory6 = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(0, 35, Math.toRadians(0+ Globals.headingError)))
                .build();

        // Time for first shot to be fired
        double waitTime1 = 0.4;
        ElapsedTime waitTimer1 = new ElapsedTime();

        // Time to shoot at high goal
        double waitTime7 = 0.4;
        ElapsedTime waitTimer7 = new ElapsedTime();

        // Time to wait until second shoot
        double waitTime8 = 0.4;
        ElapsedTime waitTimer8 = new ElapsedTime();

        // Time for second shoot
        double waitTime9 = 0.4;
        ElapsedTime waitTimer9 = new ElapsedTime();

        // Time until  third shoot
        double waitTime10 = 0.4;
        ElapsedTime waitTimer10 = new ElapsedTime();

        // Calculate the angle to turn from first to second power shot
        // Obtain the target heading
        double theta1 = Targets.middleBluePowerShot.minus(trajectory1.end().vec()).angle();
        // Calculate the rotation
        double turnAngle1 = Math.toRadians(theta1);

        // Time for second shot to be fired
        double waitTime2 = 0.4;
        ElapsedTime waitTimer2 = new ElapsedTime();

        // Calculate the angle to turn from second to third power shot
        // Obtain the target heading
        double theta2 = Targets.rightBluePowerShot.minus(trajectory1.end().vec()).angle();
        // Calculate the rotation
        double turnAngle2 = Math.toRadians(theta2-theta1);

        // Time for third shot to be fired
        double waitTime3 = 0.4;
        ElapsedTime waitTimer3 = new ElapsedTime();

        // Calculate new start pose
        Pose2d newLastPose = trajectory1.end().plus(new Pose2d(0, 0, turnAngle1 + turnAngle2));

        // Trajectory to deposit first wobble goal with 0 rings
        Trajectory trajectory2_0 = drive.trajectoryBuilder(newLastPose)
                .lineToLinearHeading(new Pose2d(0, 38, Math.toRadians(270)))
                .build();

        // Trajectory to deposit first wobble goal with 1 ring
        Trajectory trajectory2_1 = drive.trajectoryBuilder(newLastPose)
                .lineToLinearHeading(new Pose2d(23.5, 23.5, Math.toRadians(270)))
                .build();

        // Trajectory to deposit first wobble goal with 4 rings
        Trajectory trajectory2_4 = drive.trajectoryBuilder(newLastPose)
                .lineToLinearHeading(new Pose2d(47, 38, Math.toRadians(270)))
                .build();

        // Time for wobble goal to be dropped
        double waitTime4 = 1;
        ElapsedTime waitTimer4 = new ElapsedTime();

        // Trajectory to pick up the second wobble goal for 0 rings
        Trajectory trajectory3_0 = drive.trajectoryBuilder(trajectory2_0.end())
                .splineTo(new Vector2d(-36, 37), Math.toRadians(0))
                .splineTo(new Vector2d(-36, 47.5), Math.toRadians(0))
                .build();

        // Trajectory to pick up the second wobble goal for 1 ring
        Trajectory trajectory3_1 = drive.trajectoryBuilder(trajectory2_1.end())
                .splineTo(new Vector2d(-36, 37), Math.toRadians(0))
                .splineTo(new Vector2d(-36, 47.5), Math.toRadians(0))
                .build();

        // Trajectory to pick up the second wobble goal for 4 rings
        Trajectory trajectory3_4 = drive.trajectoryBuilder(trajectory2_4.end())
                .splineTo(new Vector2d(-36, 37), Math.toRadians(0))
                .splineTo(new Vector2d(-36, 47.5), Math.toRadians(0))
                .build();

        // Time for wobble goal to be picked up
        double waitTime5 = 1;
        ElapsedTime waitTimer5 = new ElapsedTime();

        // Trajectory to deposit second wobble goal with 0 rings
        Trajectory trajectory4_0 = drive.trajectoryBuilder(trajectory3_0.end())
                .lineToLinearHeading(new Pose2d(0, 38, Math.toRadians(180)))
                .build();

        // Trajectory to deposit second wobble goal with 1 ring
        Trajectory trajectory4_1 = drive.trajectoryBuilder(trajectory3_1.end())
                .lineToLinearHeading(new Pose2d(23.5, 15, Math.toRadians(180)))
                .build();

        // Trajectory to deposit second wobble goal with 4 rings
        Trajectory trajectory4_4 = drive.trajectoryBuilder(trajectory3_4.end())
                .lineToLinearHeading(new Pose2d(47, 38, Math.toRadians(180)))
                .build();

        // Time for wobble goal to be dropped
        double waitTime6 = 1;
        ElapsedTime waitTimer6 = new ElapsedTime();

        // Trajectory to the line with 0 rings
        Trajectory trajectory5_0 = drive.trajectoryBuilder(trajectory4_0.end())
                .lineToLinearHeading(new Pose2d(0, 0, Math.toRadians(0)))
                .build();

        // Trajectory to the line with 1 ring
        Trajectory trajectory5_1 = drive.trajectoryBuilder(trajectory4_1.end())
                .lineToLinearHeading(new Pose2d(0, 0, Math.toRadians(0)))
                .build();

        // Trajectory to the line with 4 rings
        Trajectory trajectory5_4 = drive.trajectoryBuilder(trajectory4_4.end())
                .lineToLinearHeading(new Pose2d(0, 0, Math.toRadians(0)))
                .build();

        waitForStart();

        if (isStopRequested()) return;

        // Set the current state to TRAJECTORY_1
        // Then have it follow that trajectory
        // Make sure we're using the async version of the commands
        // Otherwise it will be blocking and pause the program here until the trajectory finishes
        currentState = State.SHOOT_1;
        //drive.followTrajectoryAsync(trajectory1);
        drive.followTrajectoryAsync(trajectory6);
        //shooter.setTargetVolicty(Globals.powerShotRPM);
        shooter.setFlapPosition(0.58);
        shooter.setTargetVolicty(Globals.standardRPM);
        shooter.setMode(Shooter.Mode.SHOOTING);

        while (opModeIsActive() && !isStopRequested()) {
            // The state machine logic

            // We define the flow of the state machine through this switch statement
            switch (currentState) {
                case SHOOT_1:
                    if (!drive.isBusy()) {
                        shooter.shoot();
                        currentState = State.SHOOT_2;
                        waitTimer7.reset();
                        /*shooter.update(getRuntime());
                        if (waitTimer7.seconds() >= waitTime7) {
                            currentState = State.SHOOT_2;
                            waitTimer8.reset();
                        }

                         */
                    }
                    break;
                /*case WAIT_7:
                    shooter.update(getRuntime());
                    if (waitTimer7.seconds() >= waitTime7) {
                        currentState = State.SHOOT_2;
                        waitTimer8.reset();
                    }
                    break;
                 */
                case SHOOT_2:
                    shooter.update(getRuntime());
                    if (waitTimer7.seconds() >= waitTime7) {
                        shooter.shoot();
                        currentState = State.WAIT_8;
                        waitTimer9.reset();
                    }
                    break;
                case WAIT_8:
                    if (waitTimer9.seconds() >= waitTime9) {
                        shooter.update(getRuntime());
                        currentState = State.SHOOT_3;
                        waitTimer10.reset();
                    }
                    break;
                case SHOOT_3:
                    if (waitTimer10.seconds() >= waitTime10) {
                        shooter.shoot();
                        currentState = State.WAIT_3;
                        waitTimer3.reset();
                    }
                    break;

                /*case TRAJECTORY_1:
                    if (!drive.isBusy()) {
                        shooter.shoot();
                        currentState = State.WAIT_1;
                        waitTimer1.reset();
                    }
                    break;
                case WAIT_1:
                    // We update our shooter to reset the feeder
                    shooter.update(getRuntime());
                    if (waitTimer1.seconds() >= waitTime1) {
                        currentState = State.TURN_1;
                        drive.turnAsync(turnAngle1);
                    }
                    break;
                case TURN_1:
                    if (!drive.isBusy()) {
                        shooter.shoot();
                        currentState = State.WAIT_2;
                        waitTimer2.reset();
                    }
                    break;
                case WAIT_2:
                    // We update our shooter to reset the feeder
                    shooter.update(getRuntime());
                    if (waitTimer2.seconds() >= waitTime2) {
                        currentState = State.TURN_2;
                        drive.turnAsync(turnAngle1);
                    }
                    break;
                case TURN_2:
                    if (!drive.isBusy()) {
                        shooter.shoot();
                        currentState = State.WAIT_3;
                        waitTimer3.reset();
                    }
                    break;
                 */
                case WAIT_3:
                    // We update our shooter to reset the feeder
                    shooter.update(getRuntime());
                    if (waitTimer3.seconds() >= waitTime3) {
                        shooter.setMode(Shooter.Mode.IDLE);
                        currentState = State.TRAJECTORY_2;
                        switch(rings) {
                            case 0:
                                drive.followTrajectoryAsync(trajectory2_0);
                                break;
                            case 1:
                                drive.followTrajectoryAsync(trajectory2_1);
                                break;
                            case 4:
                                drive.followTrajectoryAsync(trajectory2_4);
                                break;
                        }
                    }
                    break;
                case TRAJECTORY_2:
                    if (!drive.isBusy()) {
                        currentState = State.WAIT_4;
                        waitTimer4.reset();
                    }
                    break;
                case WAIT_4:
                    if (waitTimer4.seconds() >= waitTime4) {
                        currentState = State.TRAJECTORY_3;
                        switch(rings) {
                            case 0:
                                drive.followTrajectoryAsync(trajectory3_0);
                                break;
                            case 1:
                                drive.followTrajectoryAsync(trajectory3_1);
                                break;
                            case 4:
                                drive.followTrajectoryAsync(trajectory3_4);
                                break;
                        }
                    }
                    break;
                case TRAJECTORY_3:
                    if (!drive.isBusy()) {
                        currentState = State.WAIT_5;
                        waitTimer5.reset();
                    }
                    break;
                case WAIT_5:
                    if (waitTimer5.seconds() >= waitTime5) {
                        currentState = State.TRAJECTORY_4;
                        switch(rings) {
                            case 0:
                                drive.followTrajectoryAsync(trajectory4_0);
                                break;
                            case 1:
                                drive.followTrajectoryAsync(trajectory4_1);
                                break;
                            case 4:
                                drive.followTrajectoryAsync(trajectory4_4);
                                break;
                        }
                    }
                    break;
                case TRAJECTORY_4:
                    if (!drive.isBusy()) {
                        currentState = State.WAIT_6;
                        waitTimer6.reset();
                    }
                    break;
                case WAIT_6:
                    if (waitTimer6.seconds() >= waitTime6) {
                        currentState = State.TRAJECTORY_5;
                        switch(rings) {
                            case 0:
                                drive.followTrajectoryAsync(trajectory5_0);
                                break;
                            case 1:
                                drive.followTrajectoryAsync(trajectory5_1);
                                break;
                            case 4:
                                drive.followTrajectoryAsync(trajectory5_4);
                                break;
                        }
                    }
                    break;
                case TRAJECTORY_5:
                    if (!drive.isBusy()) {
                        currentState = State.IDLE;
                    }
                    break;
                case IDLE:
                    // Do nothing in IDLE
                    // currentState does not change once in IDLE
                    // This concludes the autonomous program
                    break;
            }

            // Anything outside of the switch statement will run independent of the currentState

            // We update drive continuously in the background, regardless of state
            drive.update();

            // Read pose
            Pose2d poseEstimate = drive.getPoseEstimate();

            // Continually write pose to PoseStorage
            PoseStorage.currentPose = poseEstimate;

            // Print pose to telemetry
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
        }
    }
}