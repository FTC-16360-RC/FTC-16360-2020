package org.firstinspires.ftc.teamcode.opmodes.tele;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lib.Controller;

@TeleOp(name="Turret", group="Iterative Opmode")
public class Turret extends OpMode {

    private Controller controller1;
    DcMotor motorA;
    DcMotor motorB;
    Servo servo;
    double v;
    double spin;
    int ticks_since;

    @Override
    public void init() {
        ticks_since = 0;
        v = 0;
        spin = 1;
        controller1 = new Controller(gamepad1);
        servo = hardwareMap.get(Servo.class, "piu");
        motorA = hardwareMap.get(DcMotor.class, "br");
        motorB = hardwareMap.get(DcMotor.class, "fr");
        //motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
    }

    @Override
    public void loop() {

        if(gamepad1.dpad_left) {
            servo.setPosition(0.25);
        }

        if(gamepad1.dpad_right || gamepad1.y) {
            servo.setPosition(0.75);
        }

        if (ticks_since > 2) {
            if (gamepad1.dpad_up) {
                spin += 0.05;
            }
            if (gamepad1.dpad_down) {
                spin -= 0.05;
            }
        }

        if(gamepad1.dpad_up || gamepad1.dpad_down)
        {
            ticks_since = 0;
        }
        if(gamepad1.b){
            v = 0;
            servo.setPosition(0.25);
        }
        if(gamepad1.x){
            v = 1;
        }
        if(spin > 1) {
            spin = 1;
        }

        ticks_since++;

        motorA.setPower(v);
        motorB.setPower(-v * spin);

        telemetry.addData("spin = ", Double.valueOf(Math.round(spin*100)).toString());
        telemetry.addData("rotations = ", Double.valueOf(Math.round(motorA.getCurrentPosition()/(28*3.7))).toString());
        telemetry.addData("ticks = ", Double.valueOf(ticks_since).toString());
        telemetry.update();
    }

    @Override
    public void stop() {
        motorA.setPower(0);
    }
}
