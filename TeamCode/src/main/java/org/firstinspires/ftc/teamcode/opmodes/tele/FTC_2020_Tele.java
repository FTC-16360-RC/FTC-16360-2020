package org.firstinspires.ftc.teamcode.opmodes.tele;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.lib.Shooting;
import org.firstinspires.ftc.teamcode.lib.Vector;
import org.firstinspires.ftc.teamcode.lib.Drivemode;


@TeleOp(name="FTC 2020 Tele", group="Iterative Opmode")
//@Disabled
public class FTC_2020_Tele extends OpMode {

    Drivemode drivemode = new Drivemode(gamepad1);
    Shooting shooting = new Shooting("blue");
    StandardTrackingWheelLocalizer myLocalizer = new StandardTrackingWheelLocalizer(hardwareMap);

    @Override
    public void init() {
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        //drivemode.startTracking();
        myLocalizer.setPoseEstimate(new Pose2d(10, 10, Math.toRadians(90)));
    }

    @Override
    public void loop() {
        myLocalizer.update();
        //drivemode.loop();
        if (gamepad1.a) {
            shooting.shootAllStatic(hardwareMap, myLocalizer.getPoseEstimate());
        }
    }
}
