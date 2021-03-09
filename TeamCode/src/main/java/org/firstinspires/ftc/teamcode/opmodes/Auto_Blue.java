package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.lib.Globals;
import org.firstinspires.ftc.teamcode.lib.PoseStorage;
import org.firstinspires.ftc.teamcode.lib.Vision;
import org.firstinspires.ftc.teamcode.lib.hardware.Intake;
import org.firstinspires.ftc.teamcode.lib.hardware.Robot;
import org.firstinspires.ftc.teamcode.lib.hardware.Shooter;
import org.firstinspires.ftc.teamcode.lib.hardware.Transfer;
import org.opencv.core.Mat;

@Autonomous(group = "opmodes")
public class Auto_Blue extends LinearOpMode {

    // robot class
    Robot robot;

    // This enum defines our "state"
    // This is defines the possible steps our program will take
    enum State {
        TRAJECTORY_1,     // robot.drive to the shooting spot for power shots
        WAIT_1,           // waiting for shooter
        WAIT_2_0,         // waiting for delivering wobble goal, go back for second wobble goal
        WAIT_2_0_1,       // go back to second wobble goal
        WAIT_2_1,         // waiting for delivering wobble goal, go back for second wobble goal
        WAIT_2_1_1,       // go back to second wobble goal
        WAIT_2_4,         // waiting for delivering wobble goal, go back for second wobble goal
        WAIT_2_4_1,       // go back to second wobble goal
        TRAJECTORY_2_0,   // go to deposit first wobble goal for zero ring
        TRAJECTORY_2_1,   // go to deposit first wobble goal for one ring
        TRAJECTORY_2_4,   // go to deposit first wobble goal for four ring
        TRAJECTORY_3_0,   // go to deliver second wobble goal for zero ring
        WAIT_3_0,         // wait until we are at the right place, let go of wobble goal
        TRAJECTORY_4_0,   // go to the white line to park robot
        WAIT_4,           // make sure we don't move anymore before finishing the program
        TRAJECTORY_3_1,   // go to take in one ring if there is only one and go to shoot it
        WAIT_3_1,         // get ready to shoot ring from staple
        WAIT_4_1,         // wait until we shot
        TRAJECTORY_5_1,   // go to deliver the second wobble goal
        WAIT_5_1,         // let go of wobble goal
        TRAJECTORY_6_1,   // we go to park the robot on the line
        TRAJECTORY_3_4,   // we take in the first two rings of the staple
        IDLE,             // Our bot will enter the IDLE state when done
        SHOOT_1,          // shoot the first ring
        SHOOT_2,          // shoot the second ring
        SHOOT_3,          // shoot the third ring
        WAIT_3_4,         // get ready to shoot
        SHOOT_4_4,        // shoot first ring from the staple
        SHOOT_4_5,        // shoot second ring from the staple
        TRAJECTORY_4_4,   // take in the other rings
        TRAJECTORY_4_4_1, // take in the other rings
        WAIT_4_4,         // get ready to shoot
        SHOOT_4_6,        // shoot the third ring
        SHOOT_4_7,        // shoot the fourth ring
        TRAJECTORY_5_4,   // go to deliver the wobble goal
        WAIT_5_4,         // let go of the wobble goal
        TRAJECTORY_6_4,   // go to park the robot on the line
        SHOOT_4_1,        // we shoot the ring if there is only one on the staple
    }

    // We define the current state we're on
    // Default to IDLE
    State currentState = State.IDLE;

    // Define our start pose
    Pose2d startPose = new Pose2d(-62, 23, Math.toRadians(0));

    private double distance = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        // Variable for rings
        int rings = 0;

        // Initialize the robot clas
        robot = new Robot(hardwareMap);

        // Initialize the shooter
        Vision vision = new Vision(hardwareMap);

        // Set initial pose
        robot.drive.setPoseEstimate(startPose);

        // Save initial pose to PoseStorage
        PoseStorage.currentPose = startPose;

        // set intake servos to hold
        robot.holdIntake();

        // trajectory moves to the spot to shoot at the high goal
        Trajectory trajectory1 = robot.drive.trajectoryBuilder(startPose)
                .splineToLinearHeading(new Pose2d(-15, 19), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(-5, 32), Math.toRadians(0))
                .build();


        // Time to shoot
        double waitTime1 = 0.5;
        ElapsedTime waitTimer1 = new ElapsedTime();

        // Time to get the shooter ready
        double waitTime18 = 1;
        ElapsedTime waitTimer18 = new ElapsedTime();

        // Time until  third shoot
        double waitTime10 = 0.8; //0.4
        ElapsedTime waitTimer10 = new ElapsedTime();

        // Time for third shot to be fired
        double waitTime2 = 0.4;
        ElapsedTime waitTimer2 = new ElapsedTime();

        // Time to let go of wobble goal
        double waitTime6 = 0.5;
        ElapsedTime waitTimer6 = new ElapsedTime();

        // Trajectory to deposit first wobble goal with 0 rings
        Trajectory trajectory2_0 = robot.drive.trajectoryBuilder(trajectory1.end())
                .lineToLinearHeading(new Pose2d(5, 47, Math.toRadians(270)))
                .build();

        // Trajectory to deposit first wobble goal with 1 ring
        Trajectory trajectory2_1 = robot.drive.trajectoryBuilder(trajectory1.end())
                .lineToLinearHeading(new Pose2d(37, 17, Math.toRadians(270)))
                .build();

        // Trajectory to deposit first wobble goal with 4 rings
        Trajectory trajectory2_4 = robot.drive.trajectoryBuilder(trajectory1.end())
                .lineToLinearHeading(new Pose2d(50, 38, Math.toRadians(270)))
                .build();

        // Time for wobble goal to be dropped
        double waitTime3 = 1;
        ElapsedTime waitTimer3 = new ElapsedTime();

        // Trajectory to pick up the second wobble goal for 0 rings
        Trajectory trajectory3_0_0 = robot.drive.trajectoryBuilder(trajectory2_0.end())
                .lineToLinearHeading(new Pose2d(-23, 55, Math.toRadians(0)))
                .build();
        Trajectory trajectory3_0_1 = robot.drive.trajectoryBuilder(trajectory2_0.end())
                .lineToLinearHeading(new Pose2d(-36, 45, Math.toRadians(0)))
                .build();

        // Trajectory to pick up the second wobble goal for 1 ring
        Trajectory trajectory3_1_0 = robot.drive.trajectoryBuilder(trajectory2_1.end())
                .lineToLinearHeading(new Pose2d(-23, 55, Math.toRadians(0)))
                .build();
        Trajectory trajectory3_1_1 = robot.drive.trajectoryBuilder(trajectory3_1_0.end())
                .lineTo(new Vector2d(-42, 43))
                .build();

        // Trajectory to pick up the second wobble goal for 4 rings
        Trajectory trajectory3_4_0 = robot.drive.trajectoryBuilder(trajectory2_4.end())
                .lineToLinearHeading(new Pose2d(-23, 55, Math.toRadians(0)))
                .build();
        Trajectory trajectory3_4_1 = robot.drive.trajectoryBuilder(trajectory3_4_0.end())
                .splineTo(new Vector2d(-42, 43), 1)
                .build();

        // we go to take in the ring if there is only one
        Trajectory trajectory7_1_1 = robot.drive.trajectoryBuilder(trajectory3_1_1.end())
                .splineToLinearHeading(new Pose2d(-5, 32), Math.toRadians(0))
                .build();

        // we go to take in 3 of four rings
        Trajectory trajectory7_4_1 = robot.drive.trajectoryBuilder(trajectory3_4_1.end())
                .lineTo(new Vector2d(-32, 33))
                .build();

        // time to shoot
        double waitTime11 = 0.8; // 0.4
        ElapsedTime waitTimer11 = new ElapsedTime();

        // time to shoot
        double waitTime12 = 0.8; // 0.4
        ElapsedTime waitTimer12 = new ElapsedTime();

        // time to transfer the ring
        double waitTime4 = 2;
        ElapsedTime waitTimer4 = new ElapsedTime();

        // time to transfer the ring
        double waitTime7 = 2;
        ElapsedTime waitTimer7 = new ElapsedTime();

        // Time for wobble goal to be picked up
        double waitTime5 = 1;
        ElapsedTime waitTimer5 = new ElapsedTime();

        // we take in the last ring and go to the point to shoot into the high goal
        Trajectory trajectory8_0 = robot.drive.trajectoryBuilder(trajectory7_4_1.end())
                .lineToLinearHeading(new Pose2d(-20, 34, Math.toRadians(0)))
                .build();
        Trajectory trajectory8_1 = robot.drive.trajectoryBuilder(trajectory8_0.end())
                .splineToLinearHeading(new Pose2d(-5, 42), Math.toRadians(0))
                .build();

        // time to shoot
        double waitTime14 = 0.6;
        ElapsedTime waitTimer14 = new ElapsedTime();

        // time to shoot
        double waitTime15 = 0.4;
        ElapsedTime waitTimer15 = new ElapsedTime();

        // Trajectory to deposit second wobble goal with 0 rings
        Trajectory trajectory4_0 = robot.drive.trajectoryBuilder(trajectory3_0_1.end())
                .lineToLinearHeading(new Pose2d(15, 38, Math.toRadians(270)))
                .build();

        // Trajectory to deposit second wobble goal with 1 ring
        Trajectory trajectory4_1 = robot.drive.trajectoryBuilder(trajectory7_1_1.end())
                .lineToLinearHeading(new Pose2d(23.5, 17, Math.toRadians(270)))
                .build();

        // Trajectory to deposit second wobble goal with 4 rings
        Trajectory trajectory4_4 = robot.drive.trajectoryBuilder(trajectory8_1.end())
                .lineToLinearHeading(new Pose2d(47, 38, Math.toRadians(270)))
                .build();

        // Trajectory to the line with 0 rings
        Trajectory trajectory5_0 = robot.drive.trajectoryBuilder(trajectory4_0.end())
                .lineToLinearHeading(new Pose2d(10, 38, Math.toRadians(270)))
                .build();

        // Trajectory to the line with 1 ring
        Trajectory trajectory5_1 = robot.drive.trajectoryBuilder(trajectory4_1.end())
                .lineToLinearHeading(new Pose2d(10, 23.5, Math.toRadians(270)))
                .build();

        // Trajectory to the line with 4 rings
        Trajectory trajectory5_4 = robot.drive.trajectoryBuilder(trajectory4_4.end())
                .lineToLinearHeading(new Pose2d(10, 38, Math.toRadians(270)))
                .build();




        waitForStart();

        if (isStopRequested()) return;

        // clear cache for bulk reading
        for (LynxModule module : this.hardwareMap.getAll(LynxModule.class)) {
            module.clearBulkCache();
        }

        // Set the current state to TRAJECTORY_1
        // Then have it follow that trajectory
        // Make sure we're using the async version of the commands
        // Otherwise it will be blocking and pause the program here until the trajectory finishes
        currentState = State.TRAJECTORY_1;
        sleep(500);
        rings = vision.getRingAmount();
        //put down the intake
        robot.dropIntake();
        sleep(500);
        robot.drive.followTrajectoryAsync(trajectory1);
        robot.setRobotState(Robot.RobotState.SHOOTING);
        //robot.wobbleStoringPos();

        while (opModeIsActive() && !isStopRequested()) {
            // clear cache for bulk reading
            for (LynxModule module : this.hardwareMap.getAll(LynxModule.class)) {
                module.clearBulkCache();
            }

            // The state machine logic

            // We define the flow of the state machine through this switch statement
            switch (currentState) {
                case TRAJECTORY_1:
                    if (!robot.drive.isBusy()) {
                        currentState = State.SHOOT_1;
                        waitTimer1.reset();
                    }
                    break;
                // shoot the first ring into the high goal
                case SHOOT_1:
                    if (waitTimer1.seconds() >= waitTime1) {
                        robot.forceShoot();
                        currentState = State.SHOOT_2;
                        waitTimer1.reset();
                    }
                    break;
                // shoot the second ring into the high goal
                case SHOOT_2:
                    if (waitTimer1.seconds() >= waitTime1) {
                        robot.forceShoot();
                        currentState = State.SHOOT_3;
                        waitTimer1.reset();
                    }
                    break;
                // shoot the third ring into the high goal
                case SHOOT_3:
                    if (waitTimer1.seconds() >= waitTime1) {
                        robot.forceShoot();
                        currentState = State.WAIT_1;
                        waitTimer2.reset();
                    }

                    break;
                case WAIT_1:
                    // We update our shooter to reset the feeder
                    if (waitTimer2.seconds() >= waitTime2) {
                        robot.setRobotState(Robot.RobotState.DRIVING);
                        robot.wobbleOuttakingPos();
                        // we deliver the wobble goal
                        switch(rings) {
                            case 0:
                                currentState = State.TRAJECTORY_2_0;
                                robot.drive.followTrajectoryAsync(trajectory2_0);
                                break;
                            case 1:
                                currentState = State.TRAJECTORY_2_1;
                                robot.drive.followTrajectoryAsync(trajectory2_1);
                                break;
                            case 4:
                                currentState = State.TRAJECTORY_2_4;
                                robot.drive.followTrajectoryAsync(trajectory2_4);
                                break;
                        }
                    }
                    break;
                case TRAJECTORY_2_0:
                    if (!robot.drive.isBusy()) {
                        robot.wobbleReleasePos();
                        robot.wobblegripperOpen();
                        currentState = State.WAIT_2_0_1;
                        waitTimer3.reset();
                    }
                    break;

                case TRAJECTORY_2_1:
                    if (!robot.drive.isBusy()) {
                        robot.wobbleReleasePos();
                        robot.wobblegripperOpen();
                        currentState = State.WAIT_2_1;
                        waitTimer3.reset();
                    }
                    break;

                case TRAJECTORY_2_4:
                    if (!robot.drive.isBusy()) {
                        robot.wobbleReleasePos();
                        robot.wobblegripperOpen();
                        currentState = State.WAIT_2_4;
                        waitTimer3.reset();
                    }
                    break;
                //we go back to get the second wobble goal
                case WAIT_2_0:
                    if (waitTimer3.seconds() >= waitTime3) {
                        robot.wobbleIntakingPos();
                        robot.drive.followTrajectoryAsync(trajectory3_0_0);
                        currentState = State.WAIT_2_0_1;

                    }
                    break;
                case WAIT_2_0_1:
                    if (waitTimer3.seconds() >= waitTime3){ //!robot.drive.isBusy()) {
                        robot.drive.followTrajectoryAsync(trajectory3_0_1);
                        currentState = State.TRAJECTORY_3_0;
                }
                    break;
                case TRAJECTORY_3_0:
                    if (!robot.drive.isBusy()) {
                        robot.wobbleGrab();
                        robot.wobbleStoringPos();
                        currentState = State.WAIT_3_0;
                        waitTimer6.reset();
                    }
                    break;
                case WAIT_3_0:
                    if (waitTimer6.seconds() >= waitTime6) {
                        //let go of wobble goal
                        robot.drive.followTrajectoryAsync(trajectory4_0);
                        currentState = State.TRAJECTORY_4_0;
                    }
                    break;
                case TRAJECTORY_4_0:
                    if (!robot.drive.isBusy()) {
                        robot.wobbleReleasePos();
                        robot.wobblegripperOpen();
                        robot.drive.followTrajectoryAsync(trajectory5_0);
                        currentState = State.WAIT_4;
                    }
                    break;
                case WAIT_2_1:
                    if (waitTimer3.seconds() >= waitTime3) {
                        robot.drive.followTrajectoryAsync(trajectory3_1_0);
                        currentState = State.WAIT_2_1_1;
                    }
                    break;
                case WAIT_2_1_1:
                    if (!robot.drive.isBusy()){
                        robot.drive.followTrajectoryAsync(trajectory3_1_1);
                        robot.wobbleIntakingPos();
                        currentState = State.TRAJECTORY_3_1;
                    }
                    break;
                case TRAJECTORY_3_1:
                    if (!robot.drive.isBusy()){
                        robot.wobbleGrab();
                        robot.wobbleStoringPos();
                        robot.setRobotState(Robot.RobotState.INTAKING);
                        robot.drive.followTrajectoryAsync(trajectory7_1_1);
                        currentState = State.WAIT_3_1;
                        waitTimer4.reset();
                    }
                    break;
                case WAIT_3_1:
                    if (waitTimer4.seconds() >= waitTime4) {
                        robot.setRobotState(Robot.RobotState.SHOOTING);
                        waitTimer18.reset();
                        currentState = State.SHOOT_4_1;
                    }
                    break;
                case SHOOT_4_1:
                    if (waitTimer18.seconds() >= waitTime18){
                        robot.forceShoot();
                        waitTimer1.reset();
                        currentState = State.WAIT_4_1;
                    }
                    break;
                case WAIT_4_1:
                    if (waitTimer1.seconds() >= waitTime1) {
                        robot.setRobotState(Robot.RobotState.DRIVING);
                        currentState = State.TRAJECTORY_5_1;
                        waitTimer5.reset();
                    }
                    break;
                case TRAJECTORY_5_1:
                    if (waitTimer5.seconds() >= waitTime5) {
                    currentState = State.WAIT_5_1;
                    robot.drive.followTrajectoryAsync(trajectory4_1);
                    }
                    break;
                case WAIT_5_1:
                    if (!robot.drive.isBusy()) {
                        robot.wobbleReleasePos();
                        robot.wobblegripperOpen();
                        currentState = State.TRAJECTORY_6_1;
                        waitTimer6.reset();
                    }
                    break;
                case TRAJECTORY_6_1:
                    if (waitTimer6.seconds() >= waitTime6) {
                        currentState = State.WAIT_4;
                        robot.setRobotState(Robot.RobotState.DRIVING);
                        robot.drive.followTrajectoryAsync(trajectory5_1);
                    }
                    break;
                case WAIT_2_4:
                    if (waitTimer3.seconds() >= waitTime3){
                        robot.drive.followTrajectoryAsync(trajectory3_4_0);
                        currentState = State.WAIT_2_4_1;
                        }
                    break;
                case WAIT_2_4_1:
                    if (!robot.drive.isBusy()){
                        robot.drive.followTrajectoryAsync(trajectory3_4_1);
                        robot.wobbleIntakingPos();
                        sleep(100);
                        currentState = State.TRAJECTORY_3_4;
                    }
                    break;
                // we take in two rings if there are four
                case TRAJECTORY_3_4:
                    if (!robot.drive.isBusy()) {
                        robot.wobbleGrab();
                        robot.wobbleStoringPos();
                        robot.intake();
                        robot.drive.followTrajectoryAsync(trajectory7_4_1);
                        currentState = State.WAIT_3_4;
                        waitTimer7.reset();
                    }
                    break;
                case WAIT_3_4:
                    if (waitTimer7.seconds() >= waitTime7) {
                        robot.transferIdle();
                        currentState = State.SHOOT_4_4;
                        sleep(100);
                    }
                case SHOOT_4_4:
                    if (!robot.drive.isBusy()) {
                        currentState = State.SHOOT_4_5;
                        robot.setRobotState(Robot.RobotState.SHOOTING);
                        waitTimer14.reset();
                    }
                    break;
                // we shoot one ring into the high goal
                case SHOOT_4_5:
                    if (waitTimer14.seconds() >= waitTime14) {
                        robot.forceShoot();
                        currentState = State.TRAJECTORY_4_4;
                        waitTimer15.reset();
                    }
                    break;
                // we take in the last ring
                case TRAJECTORY_4_4:
                    if (waitTimer15.seconds() >= waitTime15) {
                        robot.intake();
                        robot.drive.followTrajectoryAsync(trajectory8_0);
                        currentState = State.TRAJECTORY_4_4_1;
                    }
                    break;
                case TRAJECTORY_4_4_1:
                    if (!robot.drive.isBusy()){
                        robot.drive.followTrajectoryAsync(trajectory8_1);
                        currentState = State.WAIT_4_4;
                    }
                    break;
                case WAIT_4_4:
                    if (!robot.drive.isBusy()) {
                        robot.transferIdle();
                        currentState = State.SHOOT_4_6;
                        waitTimer10.reset();
                    }
                // we shoot the next ring
                case SHOOT_4_6:
                    if (waitTimer10.seconds() >= waitTime10) {
                        robot.forceShoot();
                        currentState = State.SHOOT_4_7;
                        waitTimer11.reset();
                    }
                    break;
                // we shoot the next ring
                case SHOOT_4_7:
                    if (waitTimer11.seconds() >= waitTime11) {
                        robot.forceShoot();
                        currentState = State.TRAJECTORY_5_4;
                        waitTimer12.reset();
                    }
                    break;
                // we deliver the second wobble goal
                case TRAJECTORY_5_4:
                    if (waitTimer12.seconds() >= waitTime12) {
                        currentState = State.WAIT_5_4;
                        robot.drive.followTrajectoryAsync(trajectory4_4);
                    }
                    break;
                case WAIT_5_4:
                    if (!robot.drive.isBusy()) {
                        robot.wobbleReleasePos();
                        robot.wobblegripperOpen();
                        currentState = State.TRAJECTORY_6_4;
                        waitTimer6.reset();
                    }
                    break;
                // we go to the white line
                case TRAJECTORY_6_4:
                    if (waitTimer6.seconds() >= waitTime6) {
                        currentState = State.WAIT_4;
                        robot.setRobotState(Robot.RobotState.DRIVING);
                        robot.drive.followTrajectoryAsync(trajectory5_4);
                    }
                    break;
                case WAIT_4:
                    if (!robot.drive.isBusy()) {
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

            // We update robot continuously in the background, regardless of state
            robot.update();

            // Read pose
            Pose2d poseEstimate = robot.drive.getPoseEstimate();

            // Continually write pose to PoseStorage
            PoseStorage.currentPose = poseEstimate;

            distance = Globals.currentTarget.minus(poseEstimate.vec()).norm();

            // Print pose to telemetry
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("rings", rings);
            telemetry.addData("state", currentState);
            telemetry.update();
        }
    }
}