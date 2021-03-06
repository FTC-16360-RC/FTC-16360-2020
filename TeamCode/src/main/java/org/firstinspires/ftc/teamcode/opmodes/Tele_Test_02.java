package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.newLib.Comms;
import org.firstinspires.ftc.teamcode.newLib.hardware.Intake;

@TeleOp(name="Harry Potter and the Colombia Disaster", group="Iterative Opmode")
//@Disabled
public class Tele_Test_02 extends OpMode {

    Intake intake;
    Servo servo1;
    Servo servo2;
    double servopos = 0;
    int servonum = 0;
    boolean lastState = true;

    @Override
    public void init() {
        Comms.hardwareMap = hardwareMap;
        Comms.telemetry = telemetry;
        Comms.gamepad1 = gamepad1;
        Comms.gamepad2 = gamepad2;
        Comms.team = Comms.Team.BLU;

        Comms.reset();
        intake = new Intake();
        servo1 = Comms.hardwareMap.get(Servo.class, "intake1");
        servo2 = Comms.hardwareMap.get(Servo.class, "intake2");
    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {
        if (gamepad1.a && lastState) {
            intake.resetIntake();
            lastState = false;
        }
        if (gamepad1.b && lastState) {
            intake.lowerIntake();
            lastState = false;
        }
        if (!gamepad1.a && !gamepad1.b) {
            lastState = true;
        }
    }
}
