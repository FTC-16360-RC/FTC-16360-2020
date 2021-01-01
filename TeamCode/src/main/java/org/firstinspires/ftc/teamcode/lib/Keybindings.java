package org.firstinspires.ftc.teamcode.lib;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.lib.datatypes.INTuple;
import org.firstinspires.ftc.teamcode.lib.datatypes.Twople;

import java.util.ArrayList;
/*
    controls:
    rotation: left Joystick
    driving: right joystick
 */

public class Keybindings {

    G g = new G();

    Controller controller1;
    Controller controller2;

    Gamepad gamepad1;
    Gamepad gamepad2;


    public Keybindings(Gamepad gamepad1, Gamepad gamepad2) {
        controller1 = new Controller(gamepad1);
        controller2 = new Controller(gamepad2);

        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
    }
    public Keybindings(Gamepad gamepad1) {
        controller1 = new Controller(gamepad1);
        this.gamepad1 = gamepad1;
    }

    public ArrayList<Twople> update(ArrayList<INTuple> instructions) {
        ArrayList<Twople> output = new ArrayList<>();

        controller1.update();
        controller2.update();

        if (gamepad1.x) {
            output.add(new Twople(G.a.ALIGN_TO_POINT, G.i.RESET_ORIENTATION));
        }

        //shooting
        if (gamepad1.a) {
            output.add(new Twople(G.a.SHOOTER, G.i.SHOOT_THREE));
        }
        if (gamepad1.x) {
            output.add(new Twople(G.a.SHOOTER, G.i.SHOOT_ONE));
        }

        //switch between targets
        if (gamepad2.dpad_up) {
            output.add(new Twople(G.a.SHOOTER, G.i.NEXT_TARGET));
        }
        if (gamepad2.dpad_down) {
            output.add(new Twople(G.a.SHOOTER, G.i.PREVIOUS_TARGET));
        }

        return output;
    }
}

