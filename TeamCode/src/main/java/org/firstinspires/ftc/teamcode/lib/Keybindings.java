package org.firstinspires.ftc.teamcode.lib;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lib.datatypes.UTuple;

import java.util.ArrayList;
/*
    controls:
    rotation: left Joystick
    driving: right joystick
 */

public class Keybindings {

    G g = new G();
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

    public ArrayList<UTuple> update(ArrayList<UTuple> instructions) {
        ArrayList<UTuple> output = new ArrayList<>();

        controller1.update();
        controller2.update();

        //reset Odometry orientation
        if (gamepad1.b) {
            output.add(new UTuple(G.a.ALIGN_TO_POINT, G.i.RESET_ORIENTATION));
        }

        //shooting
        if (gamepad1.a) {
            output.add(new UTuple(G.a.SHOOTER, G.i.SHOOT_THREE));
        }
        if (gamepad1.x) {
            output.add(new UTuple(G.a.SHOOTER, G.i.SHOOT_ONE));
        }

        //switch between targets
        if (gamepad2.dpad_up) {
            output.add(new UTuple(G.a.SHOOTER, G.i.NEXT_TARGET));
        }
        if (gamepad2.dpad_down) {
            output.add(new UTuple(G.a.SHOOTER, G.i.PREVIOUS_TARGET));
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
                    output.add(new UTuple(G.a.INTAKE, G.i.DISABLE_LIFT_DEBUG));
                    lift_on = !lift_on;
                }
                else {
                    output.add(new UTuple(G.a.INTAKE, G.i.ENABLE_LIFT_DEBUG));
                    lift_on = !lift_on;
                }
            }
            if (gamepad2.a) {
                if (intake_on) {
                    output.add(new UTuple(G.a.INTAKE, G.i.DISABLE_INTAKE_DEBUG));
                    intake_on = !intake_on;
                }
                else {
                    output.add(new UTuple(G.a.INTAKE, G.i.ENABLE_INTAKE_DEBUG));
                    intake_on = !intake_on;
                }
            }
            if (gamepad2.y) {
                if (shooter_on) {
                    output.add(new UTuple(G.a.SHOOTER, G.i.DISABLE_SHOOTER));
                    shooter_on = !shooter_on;
                }
                else {
                    output.add(new UTuple(G.a.SHOOTER, G.i.ENABLE_SHOOTER));
                    shooter_on = !shooter_on;
                }
            }
            if (gamepad2.dpad_down) {
                output.add(new UTuple(G.a.INTAKE, G.i.LOWER_INTAKE_DEBUG));
            }
            if (gamepad2.dpad_left) {
                output.add(new UTuple(G.a.SHOOTER, G.i.ADJUST_FLAP_DEBUG));
            }
            if (gamepad2.dpad_up) {
                output.add(new UTuple(G.a.SHOOTER, G.i.FEED_RING_DEBUG));
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

        return output;
    }
}

