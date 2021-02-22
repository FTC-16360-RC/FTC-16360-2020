package org.firstinspires.ftc.teamcode.newLib;

import com.arcrobotics.ftclib.command.OdometrySubsystem;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.Motor.Encoder;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.DashboardUtil;

/**
 * This opmode demonstrates how one would implement "align to point behavior" in teleop. You specify
 * a desired vector (x/y coordinate) via`targetPosition`. In the `ALIGN_TO_POINT` mode, the bot will
 * switch into field centric control and independently control its heading to align itself with the
 * specified `targetPosition`.
 * <p>
 * Press `a` to switch into alignment mode and `b` to switch back into standard teleop driving mode.
 * <p>
 * Note: We don't call drive.update() here because it has its own field drawing functions. We don't
 * want that to interfere with our graph so we just directly update localizer instead
 *
 */

public class AlignToPointARC {

    HardwareMap hardwareMap;
    Telemetry telemetry;
    Gamepad gamepad1;
    Gamepad gamepad2;

    SampleMecanumDrive drive;

    static final double TRACKWIDTH = 13.7;
    static final double TICKS_TO_INCHES = 15.3;
    public static final double CENTER_WHEEL_OFFSET = -2.1;
    public static final double WHEEL_DIAMETER = 2.0;
    public static final double TICKS_PER_REV = 8192;
    public static final double DISTANCE_PER_PULSE = Math.PI * WHEEL_DIAMETER / TICKS_PER_REV;

    private MotorEx frontLeft, frontRight, backLeft, backRight;
    private MecanumDrive driveTrain;
    private Motor intakeLeft, intakeRight, liftLeft, liftRight;
    private Encoder leftOdometer, rightOdometer, centerOdometer;
    private HolonomicOdometry holonomicOdometry;
    private OdometrySubsystem odometry;

    //public static double DRAWING_TARGET_RADIUS = 2;

    // Define 2 states, driver control or alignment control
    enum Mode {
        NORMAL_CONTROL,
        ALIGN_TO_POINT
    }

    int errorX = 0;
    int errorY = 0;

    private Mode currentMode = Mode.NORMAL_CONTROL;

    // Declare a PIDF Controller to regulate heading
    // Use the same gains as SampleMecanumDrive's heading controller
    private PIDFController headingController = new PIDFController(8,0,0,0);

    // Declare a target vector you'd like your bot to align with
    // Can be any x/y coordinate of your choosing
    private Vector2d targetPosition = new Vector2d(138, 18);

    private void resetOrientation() {
        drive.getLocalizer().setPoseEstimate(new Pose2d(0, 0, 0));
        errorX = 0;
        errorY = 0;
    }

    public void setCurrentMode(Mode mode) {
        currentMode = mode;
    }

    public AlignToPointARC() {
        //odometry shenanigains (https://github.com/FTCLib/FTCLib/blob/v1.1.2/examples/src/main/java/com/example/ftclibexamples/DeadWheelsSample.java)
        frontLeft = new MotorEx(hardwareMap, "front_left");
        frontRight = new MotorEx(hardwareMap, "front_right");
        backLeft = new MotorEx(hardwareMap, "back_left");
        backRight = new MotorEx(hardwareMap, "back_right");
        driveTrain = new MecanumDrive(frontLeft, frontRight, backLeft, backRight);

        leftOdometer = frontLeft.encoder.setDistancePerPulse(DISTANCE_PER_PULSE);
        rightOdometer = frontRight.encoder.setDistancePerPulse(DISTANCE_PER_PULSE);
        centerOdometer = backLeft.encoder.setDistancePerPulse(DISTANCE_PER_PULSE);

        this.hardwareMap = Comms.hardwareMap;
        this.telemetry = Comms.telemetry;
        this.gamepad1 = Comms.gamepad1;
        this.gamepad2 = Comms.gamepad2;

        holonomicOdometry = new HolonomicOdometry(
                leftOdometer::getDistance,
                rightOdometer::getDistance,
                centerOdometer::getDistance,
                TRACKWIDTH, CENTER_WHEEL_OFFSET
        );
        odometry = new OdometrySubsystem(holonomicOdometry);


        targetPosition = Targets.currentTarget;

        // Initialize SampleMecanumDrive
        drive = new SampleMecanumDrive(hardwareMap);

        // We want to turn off velocity control for teleop
        // Velocity control per wheel is not necessary outside of motion profiled auto
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        resetOrientation();

        // Set input bounds for the heading controller
        // Automatically handles overflow
        //headingController.setInputBounds(-Math.PI, Math.PI);
    }

    public void update() {

        // Read pose
        Pose2d poseEstimate = odometry.getPose();

        // Declare a drive direction
        // Pose representing desired x, y, and angular velocity
        Pose2d driveDirection = new Pose2d();

        switch (currentMode) {
            case NORMAL_CONTROL:
                // Switch into alignment mode if `a` is pressed
                // Standard teleop control
                // Convert gamepad input into desired pose velocity
                driveDirection = new Pose2d(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x,
                        -gamepad1.right_stick_x
                );
                break;
            case ALIGN_TO_POINT:
                targetPosition = new Vector2d(Targets.currentTarget.getX() + errorX, Targets.currentTarget.getY() + errorY);
                // Switch back into normal driver control mode if `b` is pressed
                // Create a vector from the gamepad x/y inputs which is the field relative movement
                // Then, rotate that vector by the inverse of that heading for field centric control
                Vector2d fieldFrameInput = new Vector2d(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x
                );
                Vector2d robotFrameInput = fieldFrameInput.rotated(-poseEstimate.getHeading());

                // Difference between the target vector and the bot's position
                Vector2d difference = targetPosition.minus(poseEstimate.vec());
                // Obtain the target angle for feedback and derivative for feedforward
                double theta = difference.angle();

                // Not technically omega because its power. This is the derivative of atan2
                double thetaFF = -fieldFrameInput.rotated(-Math.PI / 2).dot(difference) / (difference.norm() * difference.norm());

                // Set the target heading for the heading controller to our desired angle
                headingController.setTargetPosition(theta);

                // Set desired angular velocity to the heading controller output + angular
                // velocity feedforward
                double headingInput = (headingController.update(poseEstimate.getHeading())
                        * DriveConstants.kV + thetaFF)
                        * DriveConstants.TRACK_WIDTH;

                // Combine the field centric x/y velocity with our derived angular velocity
                driveDirection = new Pose2d(
                        robotFrameInput,
                        headingInput
                );
        }

        //get position
        Comms.position = driveDirection;
        // Print pose to telemetry
        /*telemetry.addData("current Mode : ", currentMode.toString());
        telemetry.addData("current Target : ", Targets.currentTargetName);
        telemetry.addData("x", poseEstimate.getX());
        telemetry.addData("y", poseEstimate.getY());
        telemetry.addData("x error", errorX);
        telemetry.addData("y error", errorY);
        telemetry.addData("heading", poseEstimate.getHeading());*/
    }
}