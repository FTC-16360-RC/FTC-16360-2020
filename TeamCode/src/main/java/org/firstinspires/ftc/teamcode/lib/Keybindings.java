package org.firstinspires.ftc.teamcode.lib;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lib.datatypes.TUtil;
import org.firstinspires.ftc.teamcode.lib.datatypes.UTuple;
import org.firstinspires.ftc.teamcode.lib.datatypes.util.Adresses;
import org.firstinspires.ftc.teamcode.lib.datatypes.util.Instructions;

import java.util.ArrayList;
/*
    controls:
    rotation: left Joystick
    driving: right joystick
 */

public class Keybindings {

    Boolean manual = false;
    Telemetry telemetry;

    Controller controller1;
    Controller controller2;

    Gamepad gamepad1;
    Gamepad gamepad2;

    //debug variables
    Boolean intake_on = false;
    Boolean lift_on = false;
    Boolean shooter_on = false;


    public Keybindings(Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry) {
        controller1 = new Controller(gamepad1);
        controller2 = new Controller(gamepad2);

        this.telemetry = telemetry;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
    }
    public Keybindings(Gamepad gamepad1) {
        controller1 = new Controller(gamepad1);
        this.gamepad1 = gamepad1;
    }

    public TUtil update(TUtil instructions) {
        TUtil messages = new TUtil();

        controller1.update();
        controller2.update();

        //reset Odometry orientation
        if (gamepad1.b) {
            messages.add(Adresses.ALIGN_TO_POINT, Instructions.RESET_ORIENTATION);
        }

        //shooting
        if (gamepad1.a) {
            messages.add(Adresses.SHOOTER, Instructions.SHOOT_THREE);
        }
        if (gamepad1.x) {
            messages.add(Adresses.SHOOTER, Instructions.SHOOT_ONE);
        }

        //switch between targets
        if (gamepad2.dpad_up) {
            messages.add(Adresses.SHOOTER, Instructions.NEXT_TARGET);
        }
        if (gamepad2.dpad_down) {
            messages.add(Adresses.SHOOTER, Instructions.PREVIOUS_TARGET);
        }

        //debugging mode
        if (controller2.getLeftBumper() == Controller.ButtonState.ON_PRESS) {
            manual = !manual;
        }
        if (gamepad2.b) {
            telemetry.addData("Press left bumper (gp2) to toggle debug mode. It is currently", manual);
        }
        if (manual) {
            if (gamepad2.x) {
                if (lift_on) {
                    messages.add(Adresses.INTAKE, Instructions.DISABLE_LIFT_DEBUG);
                    lift_on = !lift_on;
                }
                else {
                    messages.add(Adresses.INTAKE, Instructions.ENABLE_LIFT_DEBUG);
                    lift_on = !lift_on;
                }
            }
            if (gamepad2.a) {
                if (intake_on) {
                    messages.add(Adresses.INTAKE, Instructions.DISABLE_INTAKE_DEBUG);
                    intake_on = !intake_on;
                }
                else {
                    messages.add(Adresses.INTAKE, Instructions.ENABLE_INTAKE_DEBUG);
                    intake_on = !intake_on;
                }
            }
            if (gamepad2.y) {
                if (shooter_on) {
                    messages.add(Adresses.SHOOTER, Instructions.DISABLE_SHOOTER);
                    shooter_on = !shooter_on;
                }
                else {
                    messages.add(Adresses.SHOOTER, Instructions.ENABLE_SHOOTER);
                    shooter_on = !shooter_on;
                }
            }
            if (gamepad2.dpad_down) {
                messages.add(Adresses.INTAKE, Instructions.LOWER_INTAKE_DEBUG);
            }
            if (gamepad2.dpad_left) {
                messages.add(Adresses.SHOOTER, Instructions.ADJUST_FLAP_DEBUG);
            }
            if (gamepad2.dpad_up) {
                messages.add(Adresses.SHOOTER, Instructions.FEED_RING_DEBUG);
            }
            if (gamepad2.b) {
                telemetry.addLine("y to toggle shooter");
                telemetry.addLine("x to toggle lift");
                telemetry.addLine("a to toggle intake");
                telemetry.addLine("dpad up to feed single ring");
                telemetry.addLine("dpad left to adjust flap");
                telemetry.addLine("dpad down to lower intake");
            }

        }
        telemetry.log();

        return messages;
    }
}

