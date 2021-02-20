package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.lib.Globals;
import org.firstinspires.ftc.teamcode.lib.Robot;

@TeleOp(name="2021 Tele Red", group="Iterative Opmode")
//@Disabled
public class FTC_2021_Tele_Red extends OpMode {

    Robot robot;

    @Override
    public void init() {
        robot = new Robot(telemetry, hardwareMap, gamepad1, gamepad2);
        robot.init();
        Globals.team = Globals.Teams.RED;
        Globals.hardwareMap = hardwareMap;
    }

    @Override
    public void init_loop() {
        robot.init_loop();
    }

    @Override
    public void start() {
        robot.start();
    }

    @Override
    public void loop() {
        robot.loop(getRuntime());
    }
}
