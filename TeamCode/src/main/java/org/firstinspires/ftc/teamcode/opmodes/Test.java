package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lib.AutoAim;
import org.firstinspires.ftc.teamcode.lib.Controller;
import org.firstinspires.ftc.teamcode.lib.Globals;
import org.firstinspires.ftc.teamcode.lib.PoseStorage;
import org.firstinspires.ftc.teamcode.lib.RobotTele;
<<<<<<< HEAD
import org.firstinspires.ftc.teamcode.lib.hardware.Intake;
import org.firstinspires.ftc.teamcode.lib.hardware.Robot;
import org.firstinspires.ftc.teamcode.lib.hardware.Shooter;
=======
>>>>>>> 2948485b147e05bef036f548d516ece2fb69bec9


@TeleOp(group = "advanced")
public class Test extends LinearOpMode {
    private RobotTele robot;
    private Controller controller;
    private Servo servo;

    private ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode() throws InterruptedException {

<<<<<<< HEAD
        DcMotor shooter1 = hardwareMap.get(DcMotor.class, "shooter1");

=======
        // initialize robot
        robot = new RobotTele(hardwareMap, gamepad1, gamepad2);

        servo = hardwareMap.get(Servo.class, "flap");
        servo.setPosition(0.6);
        controller = new Controller(gamepad1);

        waitForStart();
>>>>>>> 2948485b147e05bef036f548d516ece2fb69bec9

        if (isStopRequested()) return;

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
<<<<<<< HEAD

            shooter1.setPower(0.1);sleep(1050);
            shooter1.setPower(0.2);sleep(1050);
            shooter1.setPower(0.3);sleep(1050);
            shooter1.setPower(0.4);sleep(1050);
            shooter1.setPower(0.5);sleep(1050);
            shooter1.setPower(0.6);sleep(1050);
            shooter1.setPower(0.7);sleep(1050);
            shooter1.setPower(0.8);sleep(1050);
            shooter1.setPower(0.9);sleep(1050);
            shooter1.setPower(1);sleep(100550);
=======
            // clear cache for bulk reading
            for (LynxModule module : this.hardwareMap.getAll(LynxModule.class)) {
                module.clearBulkCache();
            }

            Globals.autonomous = true;
            controller.update();
            robot.update();

            if(controller.getdPadUp() == Controller.ButtonState.ON_PRESS) {
                servo.setPosition(servo.getPosition() + 0.01);
            }
            if(controller.getdPadDown() == Controller.ButtonState.ON_PRESS) {
                servo.setPosition(servo.getPosition() - 0.01);
            }
            if(controller.getaButton() == Controller.ButtonState.ON_PRESS) {
                robot.shoot();
            }

            telemetry.addData("pos", servo.getPosition());
            telemetry.update();
>>>>>>> 2948485b147e05bef036f548d516ece2fb69bec9




            telemetry.addData("done", "done");
            telemetry.update();
        }
    }
}
