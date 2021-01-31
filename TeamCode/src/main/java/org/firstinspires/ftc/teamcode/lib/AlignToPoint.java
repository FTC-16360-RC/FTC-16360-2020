package org.firstinspires.ftc.teamcode.lib;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.drive.Drive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.lib.datatypes.TUtil;
import org.firstinspires.ftc.teamcode.lib.datatypes.UTuple;
import org.firstinspires.ftc.teamcode.lib.datatypes.util.Adresses;
import org.firstinspires.ftc.teamcode.lib.datatypes.util.Instructions;
import org.firstinspires.ftc.teamcode.util.DashboardUtil;

import java.util.ArrayList;

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

public class AlignToPoint {

    HardwareMap hardwareMap;
    Telemetry telemetry;
    Gamepad gamepad1;
    Gamepad gamepad2;

    SampleMecanumDrive drive;

    boolean a;
    boolean b;

    public static double DRAWING_TARGET_RADIUS = 2;

    // Define 2 states, driver control or alignment control
    enum Mode {
        NORMAL_CONTROL,
        ALIGN_TO_POINT
    }

    int errorX = 0;
    int errorY = 0;
    boolean lastState = false;

    private Mode currentMode = Mode.NORMAL_CONTROL;

    // Declare a PIDF Controller to regulate heading
    // Use the same gains as SampleMecanumDrive's heading controller
    private PIDFController headingController = new PIDFController(SampleMecanumDrive.HEADING_PID);

    // Declare a target vector you'd like your bot to align with
    // Can be any x/y coordinate of your choosing
    private Vector2d targetPosition = new Vector2d(-138, -18);

    private void resetOrientation() {
        drive.getLocalizer().setPoseEstimate(new Pose2d(0, 0, 0));
        errorX = 0;
        errorY = 0;
    }


    public AlignToPoint(HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2) {

        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;

        targetPosition = Targets.currentTarget;

        // Initialize SampleMecanumDrive
        drive = new SampleMecanumDrive(hardwareMap);

        // We want to turn off velocity control for teleop
        // Velocity control per wheel is not necessary outside of motion profiled auto
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        resetOrientation();

        // Set input bounds for the heading controller
        // Automatically handles overflow
        headingController.setInputBounds(-Math.PI, Math.PI);
    }

    public TUtil update(TUtil instructions) {

        a = false;
        b = false;

        for (UTuple i : instructions.list) {
            switch (i.a_ins) {
                case RESET_ORIENTATION:
                    resetOrientation();
                    break;
                case STD_DPAD_LEFT:
                    errorY += 5;
                    break;
                case STD_DPAD_RIGHT:
                    errorY -= 5;
                    break;
                case STD_A:
                    a = true;
                    break;
                case STD_B:
                    b = true;
                    break;
                default:
                    telemetry.addLine("oh  no oh no oh no something went wrong oh no");
                    break;
            }
        }

        // Read pose
        Pose2d poseEstimate = drive.getLocalizer().getPoseEstimate();

        // Declare a drive direction
        // Pose representing desired x, y, and angular velocity
        Pose2d driveDirection = new Pose2d();

        telemetry.addData("mode", currentMode);

        // Declare telemetry packet for dashboard field drawing
        TelemetryPacket packet = new TelemetryPacket();
        Canvas fieldOverlay = packet.fieldOverlay();

        switch (currentMode) {
            case NORMAL_CONTROL:
                // Switch into alignment mode if `a` is pressed
                if (a) {
                    currentMode = Mode.ALIGN_TO_POINT;
                }

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
                if (b) {
                    currentMode = Mode.NORMAL_CONTROL;
                }

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

                // Draw the target on the field
                fieldOverlay.setStroke("#dd2c00");
                fieldOverlay.strokeCircle(targetPosition.getX(), targetPosition.getY(), DRAWING_TARGET_RADIUS);

                // Draw lines to target
                fieldOverlay.setStroke("#b89eff");
                fieldOverlay.strokeLine(targetPosition.getX(), targetPosition.getY(), poseEstimate.getX(), poseEstimate.getY());
                fieldOverlay.setStroke("#ffce7a");
                fieldOverlay.strokeLine(targetPosition.getX(), targetPosition.getY(), targetPosition.getX(), poseEstimate.getY());
                fieldOverlay.strokeLine(targetPosition.getX(), poseEstimate.getY(), poseEstimate.getX(), poseEstimate.getY());
                break;
        }

        // Draw bot on canvas
        fieldOverlay.setStroke("#3F51B5");
        DashboardUtil.drawRobot(fieldOverlay, poseEstimate);

        drive.setWeightedDrivePower(driveDirection);

        // Update the heading controller with our current heading
        headingController.update(poseEstimate.getHeading());

        // Update he localizer
        drive.getLocalizer().update();

        // Send telemetry packet off to dashboard
        FtcDashboard.getInstance().sendTelemetryPacket(packet);

        // Print pose to telemetry
        telemetry.addData("current Target : ", Targets.currentTargetName);
        telemetry.addData("x", poseEstimate.getX());
        telemetry.addData("y", poseEstimate.getY());
        telemetry.addData("x error", errorX);
        telemetry.addData("y error", errorY);
        telemetry.addData("heading", poseEstimate.getHeading());
        telemetry.update();

        TUtil messages = new TUtil();

        if (a || b) {
            if (currentMode == Mode.ALIGN_TO_POINT) {
                messages.add(Adresses.SHOOTER, Instructions.SET_SHOOTER_ON);
                messages.add(Adresses.INTAKE, Instructions.SET_INTAKE_IDLE);
            }
            if (currentMode == Mode.NORMAL_CONTROL) {
                messages.add(Adresses.SHOOTER, Instructions.SET_SHOOTER_IDLE);
                messages.add(Adresses.INTAKE, Instructions.SET_INTAKE_ON);
                messages.add(Adresses.TRANSFER, Instructions.SET_TRANSFER_ON);
            }
        }
        double[] position = {poseEstimate.getX(), poseEstimate.getY()};
        messages.add(Adresses.SHOOTER, Instructions.RECIEVE_POSITION, position);

        return messages;
    }
}