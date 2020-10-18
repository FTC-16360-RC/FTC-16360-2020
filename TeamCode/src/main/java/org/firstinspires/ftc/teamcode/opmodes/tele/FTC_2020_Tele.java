package org.firstinspires.ftc.teamcode.opmodes.tele;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.lib.Shooter;


@TeleOp(name="FTC 2020 Tele", group="Iterative Opmode")
//@Disabled
public class FTC_2020_Tele extends OpMode {

    private Shooter shooter = new Shooter(hardwareMap);
    Pose2d currentPosition = new Pose2d();

    StandardTrackingWheelLocalizer myLocalizer = new StandardTrackingWheelLocalizer(hardwareMap);
    public double d = 0;

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
        currentPosition = myLocalizer.getPoseEstimate();
        //drivemode.loop();
        if (gamepad1.a) {
            shooter.shootAllStatic(currentPosition);
        }
    }
}
