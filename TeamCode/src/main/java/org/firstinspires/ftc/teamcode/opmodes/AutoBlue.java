package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmode.Intake;
import org.firstinspires.ftc.teamcode.drive.opmode.Shooter;
import org.firstinspires.ftc.teamcode.drive.opmode.WobbleGoal;
import org.firstinspires.ftc.teamcode.lib.Globals;
import org.firstinspires.ftc.teamcode.lib.PoseStorage;
import org.firstinspires.ftc.teamcode.lib.Targets;
import org.firstinspires.ftc.teamcode.lib.Vision;
import org.firstinspires.ftc.teamcode.drive.opmode.Transfer;
import org.opencv.core.Mat;

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
        WAIT_7,         // wait to shoot
        SHOOT_2,        // shoot the second ring
        SHOOT_3,        // shoot the third ring
        TRAJECTORY_6,   // go to the point to shoot at the high goal
        TRAJECTORY_7,   // take in the last ring
        WAIT_8_1,       // wait to shoot
        WAIT_8_4,       // wait to shoot
        SHOOT_4,        // shoot the forth ring
        SHOOT_5,        // shoot the fifth ring
        SHOOT_6,        // shoot the sixth ring
        TRAJECTORY_9,   // go to shoot 4,5,6
        SHOOT_7,        // shoot the forth ring of the staple
        WAIT_9,         // wait until the ring is through the transfer
        WAIT_10,        // wait until the ring is through the transfer
        WAIT_11         // wait until the ring is through the transfer
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

        Transfer transfer = new Transfer(hardwareMap);

        WobbleGoal wobbleGoal = new WobbleGoal(hardwareMap);

        Vision vision = new Vision(hardwareMap);

        Intake intake = new Intake(hardwareMap);

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
                .splineToLinearHeading(new Pose2d(-15, 19), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(-5, 20), Math.toRadians(330))
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
                .lineToLinearHeading(new Pose2d(25, 23.5, Math.toRadians(270)))
                .build();

        // Trajectory to deposit first wobble goal with 4 rings
        Trajectory trajectory2_4 = drive.trajectoryBuilder(newLastPose)
                .lineToLinearHeading(new Pose2d(47, 38, Math.toRadians(270)))
                .build();

        // Time for wobble goal to be dropped
        double waitTime4 = 1;
        ElapsedTime waitTimer4 = new ElapsedTime();

        // Trajectory to pick up the second wobble goal for 0 rings
        Trajectory trajectory3_0_0 = drive.trajectoryBuilder(trajectory2_0.end())
                .lineToLinearHeading(new Pose2d(-23, 55, Math.toRadians(0)))
                .build();
        Trajectory trajectory3_0_1 = drive.trajectoryBuilder(trajectory3_0_0.end())
                .lineTo(new Vector2d(-38, 40))
                .build();

        // Trajectory to pick up the second wobble goal for 1 ring
        Trajectory trajectory3_1_0 = drive.trajectoryBuilder(trajectory2_1.end())
                .lineToLinearHeading(new Pose2d(-23, 55, Math.toRadians(0)))
                .build();
        Trajectory trajectory3_1_1 = drive.trajectoryBuilder(trajectory3_1_0.end())
                .lineTo(new Vector2d(-38, 40))
                .build();

        // Trajectory to pick up the second wobble goal for 4 rings
        Trajectory trajectory3_4_0 = drive.trajectoryBuilder(trajectory2_4.end())
                .lineToLinearHeading(new Pose2d(-23, 55, Math.toRadians(0)))
                .build();
        Trajectory trajectory3_4_1 = drive.trajectoryBuilder(trajectory3_4_0.end())
                .lineTo(new Vector2d(-38, 40))
                .build();

        // we go to take in the ring if there is only one
        /*Trajectory trajectory7_1_0 = drive.trajectoryBuilder(trajectory3_1_1.end())
                .lineToLinearHeading(new Pose2d(-36, 34, Math.toRadians(0)))
                .build();
         */
        Trajectory trajectory7_1_1 = drive.trajectoryBuilder(trajectory3_1_1.end())
                .splineToLinearHeading(new Pose2d(-5, 20), Math.toRadians(330))
                .build();

        // we go to take in 3 of four rings
        /*Trajectory trajectory7_4_0 = drive.trajectoryBuilder(trajectory3_4_1.end())
                .lineToLinearHeading(new Pose2d(-37.5, 25.5, Math.toRadians(0)))
                .build();
         */
        Trajectory trajectory7_4_1 = drive.trajectoryBuilder(trajectory3_4_1.end())
                .lineTo(new Vector2d(-24, 35))
                .build();

        // time to shoot
        double waitTime11 = 0.4;
        ElapsedTime waitTimer11 = new ElapsedTime();

        // time to shoot
        double waitTime12 = 0.4;
        ElapsedTime waitTimer12 = new ElapsedTime();

        // time to shoot
        double waitTime13 = 0.4;
        ElapsedTime waitTimer13 = new ElapsedTime();

        // time to transfer the ring
        double waitTime16 = 2;
        ElapsedTime waitTimer16 = new ElapsedTime();

        // time to transfer the ring
        double waitTime17 = 2;
        ElapsedTime waitTimer17 = new ElapsedTime();

        // time to transfer the ring
        double waitTime18 = 2;
        ElapsedTime waitTimer18 = new ElapsedTime();

        // Time for wobble goal to be picked up
        double waitTime5 = 1;
        ElapsedTime waitTimer5 = new ElapsedTime();

        // we take in the last ring and go to the point to shoot into the high goal
        Trajectory trajectory8_0 = drive.trajectoryBuilder(trajectory7_4_1.end())
                .lineToLinearHeading(new Pose2d(-20, 34, Math.toRadians(0)))
                .build();
        Trajectory trajectory8_1 = drive.trajectoryBuilder(trajectory8_0.end())
                .splineToLinearHeading(new Pose2d(-5, 20), Math.toRadians(330))
                .build();

        // time to shoot
        double waitTime14 = 0.4;
        ElapsedTime waitTimer14 = new ElapsedTime();

        // time to shoot
        double waitTime15 = 0.4;
        ElapsedTime waitTimer15 = new ElapsedTime();

        // Trajectory to deposit second wobble goal with 0 rings
        Trajectory trajectory4_0 = drive.trajectoryBuilder(trajectory3_0_1.end())
                .lineToLinearHeading(new Pose2d(0, 38, Math.toRadians(270)))
                .build();

        // Trajectory to deposit second wobble goal with 1 ring
        Trajectory trajectory4_1 = drive.trajectoryBuilder(trajectory7_1_1.end())
                .lineToLinearHeading(new Pose2d(23.5, 23.5, Math.toRadians(270)))
                .build();

        // Trajectory to deposit second wobble goal with 4 rings
        Trajectory trajectory4_4 = drive.trajectoryBuilder(trajectory8_1.end())
                .lineToLinearHeading(new Pose2d(47, 38, Math.toRadians(270)))
                .build();

        // Time for wobble goal to be dropped
        double waitTime6 = 1;
        ElapsedTime waitTimer6 = new ElapsedTime();

        // Trajectory to the line with 0 rings
        Trajectory trajectory5_0 = drive.trajectoryBuilder(trajectory4_0.end())
                .lineToLinearHeading(new Pose2d(10, 38, Math.toRadians(270)))
                .build();

        // Trajectory to the line with 1 ring
        Trajectory trajectory5_1 = drive.trajectoryBuilder(trajectory4_1.end())
                .lineToLinearHeading(new Pose2d(10, 23.5, Math.toRadians(270)))
                .build();

        // Trajectory to the line with 4 rings
        Trajectory trajectory5_4 = drive.trajectoryBuilder(trajectory4_4.end())
                .lineToLinearHeading(new Pose2d(10, 38, Math.toRadians(270)))
                .build();




        waitForStart();

        if (isStopRequested()) return;



        // Set the current state to TRAJECTORY_1
        // Then have it follow that trajectory
        // Make sure we're using the async version of the commands
        // Otherwise it will be blocking and pause the program here until the trajectory finishes
        currentState = State.WAIT_7;
        sleep(500);
        rings = 1;
        //rings = vision.getRingAmount();
        //drive.followTrajectoryAsync(trajectory1);
        drive.followTrajectoryAsync(trajectory6);
        //shooter.setTargetVolicty(Globals.powerShotRPM);
        shooter.setFlapPosition(0.35);
        shooter.setTargetVolicty(Globals.standardRPM);
        shooter.setMode(Shooter.Mode.SHOOTING);

        while (opModeIsActive() && !isStopRequested()) {
            // The state machine logic

            // We define the flow of the state machine through this switch statement
            switch (currentState) {
                case WAIT_7:
                    if (!drive.isBusy()) {
                        currentState = State.SHOOT_1;
                        waitTimer7.reset();
                    }
                    break;
                    // shoot the first ring into the high goal
                case SHOOT_1:
                    shooter.update(getRuntime());
                    if (waitTimer7.seconds() >= waitTime7) {
                        shooter.shoot();
                        currentState = State.SHOOT_2;
                        waitTimer8.reset();
                    }
                    break;
                    // shoot the second ring into the high goal
                case SHOOT_2:
                    shooter.update(getRuntime());
                    if (waitTimer8.seconds() >= waitTime8) {
                        shooter.shoot();
                        currentState = State.SHOOT_3;
                        waitTimer9.reset();
                    }
                    break;
                    // shoot the third ring into the high goal
                case SHOOT_3:
                    shooter.update(getRuntime());
                    if (waitTimer9.seconds() >= waitTime9) {
                        shooter.shoot();
                        currentState = State.WAIT_3;
                        waitTimer3.reset();
                    }

                    break;
                case WAIT_3:
                    // We update our shooter to reset the feeder
                    shooter.update(getRuntime());
                    if (waitTimer3.seconds() >= waitTime3) {
                        shooter.setMode(Shooter.Mode.IDLE);
                        currentState = State.TRAJECTORY_2;
                        // we deliver the wobble goal
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
                        wobbleGoal.setMode(WobbleGoal.Mode.NOTHING);
                        currentState = State.WAIT_4;
                        waitTimer4.reset();
                    }
                    break;
                    //we go back to get the second wobble goal
                case WAIT_4:
                    if (waitTimer4.seconds() >= waitTime4) {
                        switch(rings) {
                            case 0:
                                drive.followTrajectory(trajectory3_0_0);
                                drive.followTrajectory(trajectory3_0_1);
                                waitTimer13.reset();
                                wobbleGoal.setMode(WobbleGoal.Mode.LIFTING);
                                currentState = State.WAIT_5;
                                break;
                            case 1:
                                drive.followTrajectory(trajectory3_1_0);
                                drive.followTrajectoryAsync(trajectory3_1_1);
                                wobbleGoal.setMode(WobbleGoal.Mode.LIFTING);
                                currentState = State.WAIT_8_1;
                                break;
                            case 4:
                                drive.followTrajectory(trajectory3_4_0);
                                drive.followTrajectory(trajectory3_4_1);
                                wobbleGoal.setMode(WobbleGoal.Mode.LIFTING);
                                transfer.setMode(Transfer.Mode.NORMAL);
                                intake.setMode(Intake.Mode.NORMAL);
                                sleep(100);
                                currentState = State.WAIT_8_4;
                                break;
                        }
                    }
                    break;
                    // we take in the ring, if there is only one
               case WAIT_8_1:
                    if (!drive.isBusy()){
                        intake.setMode(Intake.Mode.NORMAL);
                        transfer.setMode(Transfer.Mode.NORMAL);
                        //drive.followTrajectory(trajectory7_1_0);
                        drive.followTrajectoryAsync(trajectory7_1_1);
                        currentState = State.WAIT_9;
                        waitTimer16.reset();
                    }
                    break;
                case WAIT_9:
                    if (waitTimer16.seconds() >= waitTime16) {
                        shooter.setFlapPosition(0.4);
                        shooter.setTargetVolicty(Globals.standardRPM);
                        intake.setMode(Intake.Mode.IDLE);
                        transfer.setMode(Transfer.Mode.IDLE);
                        shooter.setFlapPosition(0.52);
                        shooter.setTargetVolicty(Globals.standardRPM);
                        shooter.setMode(Shooter.Mode.SHOOTING);
                        currentState = State.TRAJECTORY_6;
                    }
                    break;
                        // we take in three rings if there are four
                case WAIT_8_4:
                    if (!drive.isBusy()) {
                        transfer.setMode(Transfer.Mode.NORMAL);
                        intake.setMode(Intake.Mode.NORMAL);
                        //drive.followTrajectory(trajectory7_4_0);
                        drive.followTrajectoryAsync(trajectory7_4_1);
                        currentState = State.WAIT_10;
                        waitTimer17.reset();

                    }
                    break;
                case WAIT_10:
                    if (waitTimer17.seconds() >= waitTime17) {
                        intake.setMode(Intake.Mode.IDLE);
                        transfer.setMode(Transfer.Mode.IDLE);
                        currentState = State.TRAJECTORY_7;
                        sleep(100);
                    }
                        case TRAJECTORY_7:
                    if (!drive.isBusy()) {
                        currentState = State.SHOOT_7;
                        shooter.setFlapPosition(0.52);
                        shooter.setTargetVolicty(Globals.standardRPM);
                        shooter.setMode(Shooter.Mode.SHOOTING);
                        waitTimer14.reset();
                    }
                    break;
                    // we shoot one ring into the high goal
                case SHOOT_7:
                    shooter.update(getRuntime());
                    if (waitTimer14.seconds() >= waitTime14) {
                        shooter.shoot();
                        currentState = State.TRAJECTORY_9;
                        waitTimer15.reset();
                    }
                    break;
                    // we take in the last ring
                case TRAJECTORY_9:
                    if (waitTimer15.seconds() >= waitTime15) {
                        transfer.setMode(Transfer.Mode.NORMAL);
                        intake.setMode(Intake.Mode.NORMAL);
                        drive.followTrajectory(trajectory8_0);
                        drive.followTrajectoryAsync(trajectory8_1);
                        currentState = State.WAIT_11;
                        waitTimer18.reset();
                    }
                    break;
                case WAIT_11:
                    if (waitTimer18.seconds() >= waitTime18) {
                        intake.setMode(Intake.Mode.IDLE);
                        transfer.setMode(Transfer.Mode.IDLE);
                        currentState = State.TRAJECTORY_6;
                    }
                        case TRAJECTORY_6:
                    if (!drive.isBusy()) {
                        currentState = State.SHOOT_4;
                        waitTimer10.reset();
                    }
                    break;
                    // we shoot the next ring
                case SHOOT_4:
                    shooter.update(getRuntime());
                    if (waitTimer10.seconds() >= waitTime10) {
                        shooter.shoot();
                        switch (rings) {
                            case 1:
                                currentState = State.TRAJECTORY_3;
                                waitTimer13.reset();
                                break;
                            case 4:
                                currentState = State.SHOOT_5;
                                waitTimer11.reset();
                                break;
                        }
                    }
                    break;
                    // we shoot the next ring
                case SHOOT_5:
                    shooter.update(getRuntime());
                    if (waitTimer11.seconds() >= waitTime11) {
                        shooter.shoot();
                        currentState = State.SHOOT_6;
                        waitTimer12.reset();
                    }
                    break;
                    // we shoot the last ring
                case SHOOT_6:
                    shooter.update(getRuntime());
                    if (waitTimer12.seconds() >= waitTime12) {
                        shooter.shoot();
                        currentState = State.WAIT_5;
                        waitTimer13.reset();
                    }
                /*case TRAJECTORY_3:
                    if (waitTimer13.seconds() >= waitTime13) {
                        currentState = State.WAIT_5;
                        waitTimer5.reset();
                    }

                 */

                    break;
                    // we deliver the second wobble goal
                case WAIT_5:
                    if (waitTimer5.seconds() >= waitTime5) {
                        currentState = State.TRAJECTORY_4;
                        switch(rings) {
                            case 0:
                                drive.followTrajectoryAsync(trajectory4_0);
                                shooter.setMode(Shooter.Mode.IDLE);
                                break;
                            case 1:
                                drive.followTrajectoryAsync(trajectory4_1);
                                shooter.setMode(Shooter.Mode.IDLE);
                                break;
                            case 4:
                                drive.followTrajectoryAsync(trajectory4_4);
                                shooter.setMode(Shooter.Mode.IDLE);
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
                    // we go to the white line
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
            telemetry.addData("rings", rings);
            telemetry.update();
        }
    }
}