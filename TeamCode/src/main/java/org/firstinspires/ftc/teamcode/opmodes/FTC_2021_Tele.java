package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.lib.RobotTele;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(group = "advanced")
public class FTC_2021_Tele extends LinearOpMode {
    private RobotTele robot;

    private ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode() throws InterruptedException {

        // initialize robot
        robot = new RobotTele(hardwareMap, gamepad1, gamepad2);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            // clear cache for bulk reading
            for (LynxModule module : this.hardwareMap.getAll(LynxModule.class)) {
                module.clearBulkCache();
            }

            // update robot and all it's elements
            robot.update();

            // get pose Estimtae for telemetry
            Pose2d poseEstimate = robot.getPoseEstimate();

            // Print pose to telemetry
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("robot state", robot.getRobotState());
            telemetry.addData("rpm", robot.getShooterRPM());
            telemetry.update();
        }
    }
}
