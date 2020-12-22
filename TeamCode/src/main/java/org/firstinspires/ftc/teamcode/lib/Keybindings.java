package org.firstinspires.ftc.teamcode.lib;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.lib.datatypes.INTuple;
import org.firstinspires.ftc.teamcode.lib.datatypes.Twople;
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

    public Twople[] update(INTuple[] instructions) {
        Twople[] output = {};
        controller1.update();
        controller2.update();

        if (controller1.getaButton() == Controller.ButtonState.PRESSED) {
            output[output.length+1] = new Twople("shooter", new INTuple("shoot"));
        }
        //update controllers

        //calculate current speed
        double speed = Math.hypot(Math.abs(controller1.getLeftJoystickXValue()), Math.abs(controller1.getLeftJoystickYValue()));

        //intake control
        if(controller1.right_bumper) {
        }

        if(controller1.left_bumper) {
        }

        if(controller1.a) {
        }

        if(controller1.getbButton() == Controller.ButtonState.ON_PRESS) {
        }

        if(controller1.x) {
        }

        if(controller1.y) {
        }

        if(controller1.getRightTrigger() == Controller.ButtonState.ON_PRESS) {
            maxDriveSpeed = 0.25;
            maxTurnSpeed = 0.25;
        }

        if(controller1.getLeftTrigger() == Controller.ButtonState.PRESSED) {
            maxDriveSpeed = 1;
            maxTurnSpeed = 1;
        }

        if(controller1.getdPadUp() == Controller.ButtonState.ON_PRESS) {
        }

        if(controller1.getdPadDown() == Controller.ButtonState.ON_PRESS) {
        }

        if(controller1.dpad_up) {
        }

        if(controller1.dpad_right) {
        }

        if(controller1.dpad_down) {
        }

        if(controller1.dpad_left) {
        }

        if(controller2.left_bumper) {
        }

        if(controller2.right_bumper) {
        }

        if(controller2.dpad_up) {
        }

        if(controller2.getdPadDown() == Controller.ButtonState.ON_PRESS) {
        }

        if(controller2.y) {
        }

        if(controller2.getRightTrigger() == Controller.ButtonState.ON_PRESS) {
        }

        if(controller2.getRightTrigger() == Controller.ButtonState.ON_RELEASE) {
        }

        if(controller2.getRightTrigger() == Controller.ButtonState.PRESSED) {
        }

        if(controller2.getLeftTrigger() == Controller.ButtonState.ON_PRESS) {
        }

        if(controller2.getxButton() == Controller.ButtonState.ON_PRESS) {
        }

        if(controller2.getaButton() == Controller.ButtonState.ON_PRESS) {
        }

        if(controller2.getLeftJoystickButton() == Controller.ButtonState.PRESSED) {
        }
        return output;
    }
}
