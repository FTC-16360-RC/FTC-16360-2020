package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.newLib.Comms;
import org.firstinspires.ftc.teamcode.newLib.Inputs;
import org.firstinspires.ftc.teamcode.newLib.Robot;

@TeleOp(name="2021 Tele Red", group="Iterative Opmode")
//@Disabled
public class FTC_2021_Tele_Red extends OpMode {

    Robot robot;
    Inputs inputs;

    @Override
    public void init() {
        Comms.hardwareMap = hardwareMap;
        Comms.telemetry = telemetry;
        Comms.gamepad1 = gamepad1;
        Comms.gamepad2 = gamepad2;
        Comms.team = Comms.Team.RED;

        Comms.reset();
        robot = new Robot();
        inputs = new Inputs();
    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {
        inputs.update();
        robot.update(getRuntime());
    }
}