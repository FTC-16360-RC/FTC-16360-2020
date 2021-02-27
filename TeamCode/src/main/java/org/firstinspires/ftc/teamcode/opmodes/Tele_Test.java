package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.newLib.Comms;
import org.firstinspires.ftc.teamcode.newLib.Inputs;
import org.firstinspires.ftc.teamcode.newLib.Robot;
import org.firstinspires.ftc.teamcode.newLib.hardware.Intake;

@TeleOp(name="$$€ⓅιV££", group="Iterative Opmode")
//@Disabled
public class Tele_Test extends OpMode {

    Intake intake;
    Servo servo1;
    Servo servo2;

    @Override
    public void init() {
        Comms.hardwareMap = hardwareMap;
        Comms.telemetry = telemetry;
        Comms.gamepad1 = gamepad1;
        Comms.gamepad2 = gamepad2;
        Comms.team = Comms.Team.BLU;

        Comms.reset();
        servo1 = Comms.hardwareMap.get(Servo.class, "feeder");
    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {
        if(gamepad1.a) {
            servo1.setPosition(1);
        }
        if(gamepad1.b) {
            servo1.setPosition(0);
        }

    }
}
