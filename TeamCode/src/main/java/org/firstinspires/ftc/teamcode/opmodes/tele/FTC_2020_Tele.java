package org.firstinspires.ftc.teamcode.opmodes.tele;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.control.Capstone;
import org.firstinspires.ftc.teamcode.control.DriveTele;
import org.firstinspires.ftc.teamcode.control.Extension;
import org.firstinspires.ftc.teamcode.control.FoundationPuller;
import org.firstinspires.ftc.teamcode.control.Gripper;
import org.firstinspires.ftc.teamcode.control.Intake;
import org.firstinspires.ftc.teamcode.control.Lift;
import org.firstinspires.ftc.teamcode.lib.Controller;


@TeleOp(name="FTC 2020 Tele", group="Iterative Opmode")
//@Disabled
public class FTC_2020_Tele extends OpMode {
    private Controller controller1;
    private Controller controller2;

    private ElapsedTime runtime = new ElapsedTime();

    private double maxDriveSpeed = 1;  //variable to control max speed
    private double maxTurnSpeed = 1;   //variable to control max turning speed

    private boolean aiming = false;
    private boolean lastDpapdUp = false;

    //the intake object
    private Intake intake;

    //mecanum drive object
    private DriveTele driveTele;

    //the IMU
    //private IMU imu;

    //the gripper
    private Gripper gripper;

    //the lift
    private Lift lift;

    //the extension
    private Extension extension;

    //the servos to pull the foundation
    private FoundationPuller foundationPuller;

    //the capstone mechanism
    private Capstone capstone;

    @Override
    public void init() {
        //initialize the intake
        intake = new Intake(hardwareMap);

        //initialize the controllers
        controller1 = new Controller(gamepad1);
        controller2 = new Controller(gamepad2);

        //initialize the IMU
        //imu = new IMU(hardwareMap);
        //imu.setPID(0.08, 0, 0.05);
        //imu.setPID(0, 0, 0);

        //initialise the gripper
        gripper = new Gripper(hardwareMap);

        //initialise the lift
        lift = new Lift(hardwareMap);

        //initialise the extension
        //extension = new Extension(hardwareMap);

        //initialise the servos of the foundation puller
        foundationPuller = new FoundationPuller(hardwareMap);
        foundationPuller.setServoState(FoundationPuller.ServoState.OPEN);

        //initialise the capstone servos
        capstone = new Capstone(hardwareMap);

        //initialize mecanum drive
        driveTele = new DriveTele(hardwareMap, DcMotor.RunMode.RUN_WITHOUT_ENCODER, DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        //imu.setTargetHeading(imu.getHeading());
        runtime.reset();
        //initialise the extension
        extension = new Extension(hardwareMap);
        extension.forceRetract();
    }

    @Override
    public void loop() {
        //update controllers
        controller1.update();
        controller2.update();

        //calculate current speed
        double speed = Math.hypot(Math.abs(controller1.getLeftJoystickXValue()), Math.abs(controller1.getLeftJoystickYValue()));

        //intake control
        if(gamepad1.right_bumper) {
            intake.setIntakeMode(Intake.IntakeMode.INTAKE);
            if(gripper.getClawMode() == Gripper.ClawMode.CLOSED)
            {
                gripper.open();
            }
            if(lift.getLiftState() == Lift.LiftState.RETRACTED) {
                lift.setLiftState(Lift.LiftState.RETRACTED);
            }
        }

        if(gamepad1.left_bumper) {
            intake.setIntakeMode(Intake.IntakeMode.OUTTAKE);
        }

        if(gamepad1.a) {
            intake.setIntakeMode(Intake.IntakeMode.REST);
        }

        if(controller1.getbButton() == Controller.ButtonState.ON_PRESS && lift.getLiftState() == Lift.LiftState.RETRACTED
                && gripper.getClawMode() == Gripper.ClawMode.CLOSED) {
            lift.setLiftState(Lift.LiftState.CAPSTONE);
            capstone.deploy();
        }

        if(gamepad1.x)
        {
            capstone.close_grabber();
        }

        if(gamepad1.y) {
            lift.resetOffset();
        }

        if(controller1.getRightTrigger() == Controller.ButtonState.ON_PRESS)
        {
            if (foundationPuller.getServoState() == FoundationPuller.ServoState.PREPARED || lift.getLiftState() != Lift.LiftState.RETRACTED)
            {
                foundationPuller.setServoState(FoundationPuller.ServoState.CLOSED);
                maxDriveSpeed = 0.6;
                maxTurnSpeed = 0.6;
            } else {
                foundationPuller.setServoState(FoundationPuller.ServoState.PREPARED);
            }
        }

        if(controller1.getLeftTrigger() == Controller.ButtonState.PRESSED)
        {
            foundationPuller.setGrabberStateRight(FoundationPuller.GrabberState.OPEN);
            foundationPuller.setGrabberStateLeft(FoundationPuller.GrabberState.OPEN);
            foundationPuller.setServoState(FoundationPuller.ServoState.OPEN);
            maxDriveSpeed = 1;
            maxTurnSpeed = 1;
        }

        if(controller1.getdPadUp() == Controller.ButtonState.ON_PRESS)
        {
            lift.changeOffset(1);
        }

        if(controller1.getdPadDown() == Controller.ButtonState.ON_PRESS)
        {
            lift.changeOffset(-1);
        }

        if(gamepad1.dpad_up) {
            foundationPuller.setServoState(FoundationPuller.ServoState.HOLD_STONE);
            foundationPuller.setGrabberStateRight(FoundationPuller.GrabberState.CLOSED);
            foundationPuller.setGrabberStateLeft(FoundationPuller.GrabberState.CLOSED);
        }

        if(gamepad1.dpad_right) {
            foundationPuller.setServoState(FoundationPuller.ServoState.AUTO_MOVE);
            foundationPuller.setGrabberStateRight(FoundationPuller.GrabberState.PREPARED);
            foundationPuller.setGrabberStateLeft(FoundationPuller.GrabberState.PREPARED);
        }

        if(gamepad1.dpad_down) {
            foundationPuller.setGrabberStateRight(FoundationPuller.GrabberState.CLOSED);
            foundationPuller.setGrabberStateLeft(FoundationPuller.GrabberState.CLOSED);
        }

        if(gamepad1.dpad_left) {
            foundationPuller.setServoState(FoundationPuller.ServoState.DEPOSIT);
            foundationPuller.setGrabberStateRight(FoundationPuller.GrabberState.PREPARED);
            foundationPuller.setGrabberStateLeft(FoundationPuller.GrabberState.PREPARED);
        }

        if(gamepad2.left_bumper && foundationPuller.getServoState() != FoundationPuller.ServoState.CLOSED)
        {
            intake.resetBlockIntaked();
            gripper.open();
            lift.unsetAim();
            if(extension.getExtensionState() == Extension.ExtensionState.EXTENDED && lift.getLevel() != 0)
            {
                maxDriveSpeed = 1;
                maxTurnSpeed = 1;
                extension.retract();
                lift.setLiftState(Lift.LiftState.EXTENDED);
                lift.setLiftState(Lift.LiftState.RETRACTED);
            }
        }

        if(gamepad2.right_bumper || intake.getBlockIntaked(gripper.getClawMode() == Gripper.ClawMode.OPENED))
        {
            intake.resetBlockIntaked();
            gripper.close();
        }

        if(intake.getIntakeMode() != Intake.IntakeMode.REST && gripper.getClawMode() == Gripper.ClawMode.CLOSED)
        {
            intake.setIntakeMode(Intake.IntakeMode.REST);
        }

        if(gamepad2.dpad_up) {
            if (!lastDpapdUp)
            {
                lift.setLevel(1);
            }
            lastDpapdUp = true;
        } else {
            lastDpapdUp = false;
        }

        if(controller2.getdPadDown() == Controller.ButtonState.ON_PRESS)
        {
            lift.setLevel(-1);
        }

        if(gamepad2.y)
        {
            lift.setLevel(-7);
        }

        if(controller2.getRightTrigger() == Controller.ButtonState.ON_PRESS)
        {
            lift.setLiftState(Lift.LiftState.EXTENDED);
            extension.extend();
            maxDriveSpeed = 0.8;
            maxTurnSpeed = 0.6;
        }

        if(controller2.getRightTrigger() == Controller.ButtonState.ON_RELEASE && lift.getLiftState() == Lift.LiftState.AIMING)
        {
            lift.setLiftState(Lift.LiftState.EXTENDED);
            maxDriveSpeed = 0.8;
            maxTurnSpeed = 0.6;
        }

        if(controller2.getRightTrigger() == Controller.ButtonState.PRESSED && lift.getLiftState() == Lift.LiftState.EXTENDED) {
            if (!aiming) {
                lift.setLiftState(Lift.LiftState.AIMING);
                aiming = true;
                maxDriveSpeed = 0.6;
                maxTurnSpeed = 0.6;
            }
        } else {
            aiming = false;
        }

        if(controller2.getLeftTrigger() == Controller.ButtonState.ON_PRESS)
        {
            if(extension.getExtensionState() == Extension.ExtensionState.EXTENDED){
                lift.setLiftState(Lift.LiftState.EXTENDED);
                extension.retract();
            }
            lift.setLiftState(Lift.LiftState.RETRACTED);
            maxDriveSpeed = 1;
            maxTurnSpeed = 1;
        }

        if(controller2.getxButton() == Controller.ButtonState.ON_PRESS)
        {
            intake.resetBlockIntaked();
            if(lift.getLiftState() == Lift.LiftState.RETRACTED)
            {
                lift.setLiftState(Lift.LiftState.RETRACTED);
                extension.push();
                gripper.open();
                intake.setIntakeMode(Intake.IntakeMode.OUTTAKE);
            } else {
                extension.extend();
            }
        } else if(extension.getExtensionState() == Extension.ExtensionState.PUSHING && !gamepad2.x){
            extension.forceRetract();
            intake.setIntakeMode(Intake.IntakeMode.REST);
            gripper.open();
        }

        if(controller2.getaButton() == Controller.ButtonState.ON_PRESS)
        {
            extension.forceRetract();
        }

        if(controller2.getLeftJoystickButton() == Controller.ButtonState.PRESSED)
        {
            lift.resetMinimum();
        }

        //control mecanum drive
        driveTele.setPower(controller1.getLeftJoystickYValue(), controller1.getLeftJoystickXValue()
                , controller1.getRightJoystickXValue(), maxDriveSpeed, maxTurnSpeed);

        lift.update(gamepad2.left_stick_y, gamepad2.right_trigger, extension.getExtensionState() == Extension.ExtensionState.RETRACTED, gripper.getClawMode() == Gripper.ClawMode.CLOSED);

        //update mecanum drive
        driveTele.updatePower();

        //update the runtime
        double currentRuntime = getRuntime();

        //update the gripper
        gripper.update(currentRuntime);

        //update the capstone mechanism
        capstone.update(currentRuntime, true);

        //update extension state
        extension.update(currentRuntime, gripper.getClawMode() == Gripper.ClawMode.OPENED, lift.getLiftState() == Lift.LiftState.EXTENDED);

        //update intake
        intake.update(currentRuntime);

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("drive speed",Double.valueOf(speed).toString());
        //telemetry.addData("heading", Double.valueOf(imu.getHeading()).toString());
        telemetry.addData("lift level", lift.getLevel());
        telemetry.update();
    }
}
