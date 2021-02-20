package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.newLib.Comms;
import org.firstinspires.ftc.teamcode.newLib.Inputs;
import org.firstinspires.ftc.teamcode.newLib.Robot;
import org.firstinspires.ftc.teamcode.newLib.hardware.Intake;

@TeleOp(name="$$€ⓅιV££", group="Iterative Opmode")
//@Disabled
public class Tele_Test extends OpMode {

    Intake intake;

    @Override
    public void init() {
        Comms.hardwareMap = hardwareMap;
        Comms.telemetry = telemetry;
        Comms.gamepad1 = gamepad1;
        Comms.gamepad2 = gamepad2;
        Comms.team = Comms.Team.BLU;

        Comms.reset();
        intake = new Intake();
    }

    @Override
    public void init_loop() {
        intake.lowerIntake();
    }

    @Override
    public void start() {
    }

    @Override
    public void loop() {
        intake.resetIntake();
    }
}
