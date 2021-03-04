package org.firstinspires.ftc.teamcode.lib;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.lib.hardware.Intake;
import org.firstinspires.ftc.teamcode.lib.hardware.Robot;
import org.firstinspires.ftc.teamcode.lib.hardware.Transfer;

public class RobotTele extends Robot {
    Controller controller1, controller2;

    public RobotTele(HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2) {
        super(hardwareMap);

        // initialise controllers
        controller1 = new Controller(gamepad1);
        controller2 = new Controller(gamepad2);
    }

    public void update() {
        // We update drive continuously in the background, regardless of state
        drive.update();

        // update controllers
        controller1.update();
        controller2.update();

        // update controls according to button states
        updateControls();

        // update shooter pidf in the background
        shooter.update();

        // very important to make sure nothing breaks
        if(intake.getMode() == Intake.Mode.FORWARD && transfer.getMode() != Transfer.Mode.FORWARD) {
            transfer.setMode(Transfer.Mode.FORWARD);
        }

        if(transfer.getMode() == Transfer.Mode.REVERSE && intake.getMode() != Intake.Mode.REVERSE) {
            intake.setMode(Intake.Mode.REVERSE);
        }

        // Read pose
        Pose2d poseEstimate = drive.getPoseEstimate();

        // Continually write pose to PoseStorage
        PoseStorage.currentPose = poseEstimate;
    }

    private void updateControls() {
        // controller 1
        if(controller1.getaButton() == Controller.ButtonState.ON_PRESS) // set to aiming mode
            setRobotState(RobotState.AIMING);

        if(controller1.getbButton() == Controller.ButtonState.ON_PRESS) // set to intaking mode
            setRobotState(RobotState.INTAKING);

        if(controller1.getRightTrigger() == Controller.ButtonState.PRESSED) { // shoot
            setRobotState(RobotState.SHOOTING);
            shoot();
        }


        // controller 2
        if(controller2.getdPadUp() == Controller.ButtonState.ON_PRESS) // increase distance by subtracting 2 inches to x pose
            drive.setPoseEstimate(drive.getPoseEstimate().plus(new Pose2d(-2, 0, 0)));

        if(controller2.getdPadDown() == Controller.ButtonState.ON_PRESS) // increase distance by adding 2 inches to x pose
            drive.setPoseEstimate(drive.getPoseEstimate().plus(new Pose2d(2, 0, 0)));

        if(controller2.getdPadLeft() == Controller.ButtonState.ON_PRESS) // correct heading by adding 1 degree
            drive.setPoseEstimate(drive.getPoseEstimate().plus(new Pose2d(0, 0, Math.toRadians(1))));

        if(controller2.getdPadRight() == Controller.ButtonState.ON_PRESS) // correct heading by subtracting 1 degree
            drive.setPoseEstimate(drive.getPoseEstimate().plus(new Pose2d(0, 0, Math.toRadians(-1))));

    }
}
