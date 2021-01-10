package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.opmode.Intake;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmode.Shooter;
import org.firstinspires.ftc.teamcode.drive.opmode.Transfer;
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
 */
@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(group = "advanced")
public class Tele extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    public static double DRAWING_TARGET_RADIUS = 2;

    // Define 2 states, driver control or alignment control
    enum Mode {
        NORMAL_CONTROL,
        ALIGN_TO_POINT
    }

    int targetX = 72;
    int targetY = 35;  //37, 21, 13.5, 6
    int errorX = 0;
    int errorY = 0;
    final double headingError = Math.toRadians(-10);//-10
    boolean lastState = false;

    private Mode currentMode = Mode.NORMAL_CONTROL;

    private Intake intake;

    private Transfer transfer;

    private Shooter shooter;

    double flapPosition;

    // Declare a PIDF Controller to regulate heading
    // Use the same gains as SampleMecanumDrive's heading controller
    private PIDFController headingController = new PIDFController(new PIDCoefficients(12, 0, 0));

    // Declare a target vector you'd like your bot to align with
    // Can be any x/y coordinate of your choosing
    private Vector2d targetPosition = new Vector2d(targetX, targetY);

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize SampleMecanumDrive
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        intake = new Intake(hardwareMap);
        transfer = new Transfer(hardwareMap);
        shooter = new Shooter(hardwareMap);

        // We want to turn off velocity control for teleop
        // Velocity control per wheel is not necessary outside of motion profiled auto
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Retrieve our pose from the PoseStorage.currentPose static field
        // See AutoTransferPose.java for further details
        drive.getLocalizer().setPoseEstimate(new Pose2d(0, 0, 0));

        // Set input bounds for the heading controller
        // Automatically handles overflow
        headingController.setInputBounds(-Math.PI, Math.PI);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            double currentRuntime = getRuntime();
            if(gamepad2.a) {
                intake.setMode(Intake.Mode.NORMAL);
                transfer.setMode(Transfer.Mode.NORMAL);
            }
            if(gamepad2.b) {
                intake.setMode(Intake.Mode.IDLE);
                transfer.setMode(Transfer.Mode.IDLE);
            }
            if(gamepad2.x) {
                intake.setMode(Intake.Mode.REVERSE);
                transfer.setMode(Transfer.Mode.REVERSE);
            }
            if(gamepad2.right_bumper || gamepad1.a) {
                shooter.setFlapPosition(flapPosition);
                shooter.setTargetVolicty(5000);
                shooter.setMode(Shooter.Mode.SHOOTING);
                intake.setMode(Intake.Mode.IDLE);
            }
            if(gamepad2.left_bumper || gamepad1.b) {
                shooter.setFlapPosition(0);
                shooter.setMode(Shooter.Mode.IDLE);
                intake.setMode(Intake.Mode.NORMAL);
                transfer.setMode(Transfer.Mode.NORMAL);
            }
            shooter.update(currentRuntime);
            if(gamepad2.right_trigger != 0) {
                shooter.shoot();
            }
            if(!gamepad1.dpad_right && !gamepad1.dpad_left && !gamepad2.dpad_up && !gamepad2.dpad_down) {
                lastState = false;
            }
            if(gamepad1.dpad_right && !lastState) {
                errorY -= 5;
                lastState = true;
            } else if(gamepad1.dpad_left && !lastState) {
                errorY += 5;
                lastState = true;
            }
            if (gamepad2.dpad_up && !lastState) {
                flapPosition += 0.1;
                if(flapPosition > 1) {
                    flapPosition = 1;
                }
                shooter.setFlapPosition(flapPosition);
                lastState = true;
            }
            if (gamepad2.dpad_down && !lastState) {
                flapPosition -= 0.1;
                if(flapPosition < 0) {
                    flapPosition = 0;
                }
                shooter.setFlapPosition(flapPosition);
                lastState = true;
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
                    if (gamepad1.a) {
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
                    targetPosition = new Vector2d(targetX+errorX, targetY+errorY);
                    // Switch back into normal driver control mode if `b` is pressed
                    if (gamepad1.b) {
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
                    headingController.setTargetPosition(theta+headingError);

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

            if(gamepad1.x) {
                drive.getLocalizer().setPoseEstimate(new Pose2d(0, 0, 0));
                errorX = 0;
                errorY = 0;
            }

            drive.setWeightedDrivePower(driveDirection);

            // Update the heading controller with our current heading
            headingController.update(poseEstimate.getHeading());

            // Update he localizer
            drive.getLocalizer().update();

            // Send telemetry packet off to dashboard
            FtcDashboard.getInstance().sendTelemetryPacket(packet);

            // Print pose to telemetry
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("flap", shooter.getFlapPosition()+"%");
            telemetry.addData("x error", errorX);
            telemetry.addData("y error", errorY);
            telemetry.update();
        }
    }
}