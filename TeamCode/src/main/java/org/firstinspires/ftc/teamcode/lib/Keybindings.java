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

    Controller controller1;
    Controller controller2;


    public Keybindings(Gamepad gamepad1, Gamepad gamepad2) {
        controller1 = new Controller(gamepad1);
        controller2 = new Controller(gamepad2);
    }
    public Keybindings(Gamepad gamepad1) {
        controller1 = new Controller(gamepad1);
    }

    public ArrayList<Twople> update(ArrayList<INTuple> instructions) {
        ArrayList<Twople> output = new ArrayList<Twople>();

        controller1.update();
        controller2.update();

        //double speed = Math.hypot(Math.abs(controller1.getLeftJoystickXValue()), Math.abs(controller1.getLeftJoystickYValue()));

        //direction
        double x = controller1.getLeftJoystickXValue();
        double y = controller1.getLeftJoystickYValue();
        double angle = controller1.getRightJoystickXValue();
        output.add(new Twople("drivetrain", new INTuple("setSpeed", new double[]{x, y, angle})));

        //switch between targets
        if (controller2.getdPadLeft() == Controller.ButtonState.ON_PRESS) {
            output.add(new Twople("shooter", new INTuple("nextTarget")));
        }
        if (controller2.getdPadRight() == Controller.ButtonState.ON_PRESS) {
            output.add(new Twople("shooter", new INTuple("previousTarget")));
        }

        return output;
    }
}
