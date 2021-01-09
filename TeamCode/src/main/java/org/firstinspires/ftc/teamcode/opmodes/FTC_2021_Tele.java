package org.firstinspires.ftc.teamcode.opmodes;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.lib.AlignToPoint;
import org.firstinspires.ftc.teamcode.lib.G;
import org.firstinspires.ftc.teamcode.lib.Keybindings;
import org.firstinspires.ftc.teamcode.lib.Robot;
import org.firstinspires.ftc.teamcode.lib.datatypes.TUtil;
import org.firstinspires.ftc.teamcode.lib.datatypes.UTuple;
import org.firstinspires.ftc.teamcode.lib.datatypes.util.Adresses;
import org.firstinspires.ftc.teamcode.lib.hardware.Intake;
import org.firstinspires.ftc.teamcode.lib.hardware.Shooter;

import java.util.ArrayList;


@TeleOp(name="FTC 2020 Tele", group="Iterative Opmode")
//@Disabled
public class FTC_2021_Tele extends OpMode {

    Robot robot;

    @Override
    public void init() {
        robot = new Robot(telemetry, hardwareMap, gamepad1, gamepad2);
        robot.init();
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
        robot.loop();
    }
}
