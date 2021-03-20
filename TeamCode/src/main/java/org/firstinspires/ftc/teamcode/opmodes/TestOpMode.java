package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.lib.AutoAim;
import org.firstinspires.ftc.teamcode.lib.Globals;
import org.firstinspires.ftc.teamcode.lib.PoseStorage;
import org.firstinspires.ftc.teamcode.lib.RobotTele;
import org.firstinspires.ftc.teamcode.lib.hardware.Shooter;
import org.opencv.core.Mat;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(group = "advanced")
public class TestOpMode extends LinearOpMode {

    double startTime;

    public static double norm(double inValue, double min, double max) {
        return (inValue - min)/(max - min);
    }

    @Override
    public void runOpMode() throws InterruptedException {

        DcMotorEx m1 = hardwareMap.get(DcMotorEx.class, "shooter1");
        DcMotorEx m2 = hardwareMap.get(DcMotorEx.class, "shooter2");
        int d = 15000;
        double pw = 1.5;
        boolean lastState = false;
        boolean accelerating = false;
        long startTime = 0;
        long time = 0;
        double speed = 0;

        double s0 = 0.08;

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            time = System.currentTimeMillis() - startTime;

            if (gamepad1.a) {
                m1.setPower(0);
                m2.setPower(0);
                accelerating = false;
            }
            if (gamepad1.b) {
                accelerating = true;
                startTime = System.currentTimeMillis();
            }

            if (accelerating && false) {
                if (time < 500) {
                    m1.setPower(s0);
                    m2.setPower(s0);
                    telemetry.addLine("phase1");
                }
                if (time > 500 && time < 3000) {
                    speed = s0 + (time-500) / 5000;
                    m1.setPower(speed);
                    m2.setPower(speed);
                    telemetry.addLine("phase2");
                    telemetry.addData("time", time);
                }
                if (time > 3000) {
                    m1.setPower(s0 + (3000-500) / 5000 + (time - 3000) / 1000);
                    telemetry.addLine("phase3");
                }

            }

            if (accelerating) {
                m1.setVelocity(time/24);
                m2.setVelocity(time/24);

            }

            if (gamepad1.dpad_up && !lastState) {
                d += 1000;
                telemetry.addData("delay", d);
                telemetry.addData("pw", pw);
                telemetry.update();
                lastState = true;
            }
            if (gamepad1.dpad_down && !lastState) {
                d -= 1000;
                telemetry.addData("delay", d);
                telemetry.addData("pw", pw);
                telemetry.update();
                lastState = true;
            }

            if (gamepad1.dpad_right && !lastState) {
                pw += 0.1;
                telemetry.addData("delay", d);
                telemetry.addData("pw", pw);
                telemetry.update();
                lastState = true;
            }
            if (gamepad1.dpad_left && !lastState) {
                pw -= 0.1;
                telemetry.addData("delay", d);
                telemetry.addData("pw", pw);
                telemetry.update();
                lastState = true;
            }

            telemetry.update();

            if(!gamepad1.dpad_up && !gamepad1.dpad_down && !gamepad1.dpad_left && !gamepad1.dpad_right) {
                lastState = false;
            }
        }
    }
}
