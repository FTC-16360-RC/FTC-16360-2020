package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.lib.AutoAim;
import org.firstinspires.ftc.teamcode.lib.Globals;
import org.firstinspires.ftc.teamcode.lib.PoseStorage;
import org.firstinspires.ftc.teamcode.lib.RobotTele;
import org.firstinspires.ftc.teamcode.lib.hardware.Shooter;


@TeleOp(group = "advanced")
public class Test extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {

        Servo ringArm;
        DcMotor lift;
        double servoPos = 0.5;
        boolean lastState = false;

        waitForStart();

        ringArm = hardwareMap.get(Servo.class, "ringArm");
        Shooter shooter = new Shooter(hardwareMap);
        lift = hardwareMap.get(DcMotor.class, "transfer");

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {

            if(gamepad1.dpad_up && !lastState) {
                servoPos += 0.01;
                lastState = true;
            }
            if(gamepad1.dpad_down && !lastState) {
                servoPos -= 0.01;
                lastState = true;
            }
            if(!gamepad1.dpad_up && !gamepad1.dpad_down) {
                lastState = false;
            }
            if(gamepad1.a) {
             shooter.shoot();
            }
            if(gamepad1.b) {
                shooter.setMode(Shooter.Mode.IDLE);
            }
            if(gamepad1.x) {
                shooter.setMode(Shooter.Mode.SHOOTING);
            }
            


            ringArm.setPosition(servoPos);
            telemetry.addData("w", Math.round(servoPos * 100) / 100);
            telemetry.addData("why", lastState);
            telemetry.update();

        }
    }
}
