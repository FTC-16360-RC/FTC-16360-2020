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

    int mode = 0;
    Telemetry telemetry;

    Controller controller1;
    Controller controller2;

    Gamepad gamepad1;
    Gamepad gamepad2;

    //debug variables
    Boolean intake_on = true;
    Boolean lift_on = true;
    Boolean shooter_on = false;
    double servoPos = 0.6;


    public Keybindings(Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2) {
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
        if (gamepad1.dpad_up) {
            messages.add(Adresses.ALIGN_TO_POINT, Instructions.RESET_ORIENTATION);
        }

        //Reverse intake / transfer
        if (gamepad1.x) {
            messages.add(Adresses.TRANSFER, Instructions.SET_TRANSFER_REVERSE);
            messages.add(Adresses.INTAKE, Instructions.SET_INTAKE_REVERSE);
        }
        if (controller1.getxButton() == Controller.ButtonState.ON_RELEASE) {
            if (lift_on) {
                messages.add(Adresses.TRANSFER, Instructions.SET_TRANSFER_ON);
            } else {
                messages.add(Adresses.TRANSFER, Instructions.SET_TRANSFER_IDLE);
                messages.add(Adresses.INTAKE, Instructions.SET_INTAKE_IDLE);
                intake_on = false;
            }
            if (intake_on) {
                messages.add(Adresses.INTAKE, Instructions.SET_INTAKE_ON);
            } else {
                messages.add(Adresses.INTAKE, Instructions.SET_INTAKE_IDLE);
            }
        }

        //toggle intake and transfer
        if (controller1.getyButton() == Controller.ButtonState.ON_RELEASE) {
            if (lift_on && intake_on) {
                messages.add(Adresses.INTAKE, Instructions.SET_INTAKE_IDLE);
                messages.add(Adresses.TRANSFER, Instructions.SET_TRANSFER_IDLE);
                lift_on = false;
                intake_on = false;
            } else {
                messages.add(Adresses.INTAKE, Instructions.SET_INTAKE_ON);
                messages.add(Adresses.TRANSFER, Instructions.SET_TRANSFER_ON);
                lift_on = true;
                intake_on = true;
            }
        }

        //drivemodes
        if (controller1.getaButton() == Controller.ButtonState.ON_PRESS) {
            messages.add(Adresses.ALIGN_TO_POINT, Instructions.STD_A);
            intake_on = false;
        }
        if (controller1.getbButton() == Controller.ButtonState.ON_PRESS) {
            messages.add(Adresses.ALIGN_TO_POINT, Instructions.STD_B);
            intake_on = true;
            lift_on = true;
        }

        //shooting
        if (gamepad1.right_trigger != 0) {
            messages.add(Adresses.SHOOTER, Instructions.SHOOT_ONE);
        }

        //AlignToPoint trimming
        if (controller1.getdPadLeft() == Controller.ButtonState.ON_PRESS) {
            messages.add(Adresses.ALIGN_TO_POINT, Instructions.STD_DPAD_LEFT);
        }
        if (controller1.getdPadRight() == Controller.ButtonState.ON_PRESS) {
            messages.add(Adresses.ALIGN_TO_POINT, Instructions.STD_DPAD_RIGHT);
        }

        //
        //Gamepad 2
        //

        //switch between modes
        if (gamepad2.left_bumper){
            mode = 0;
            messages.add(Adresses.ALIGN_TO_POINT, Instructions.MODE_SIMPLE);
        }
        if (gamepad2.right_bumper) {
            mode = 1;
            messages.add(Adresses.ALIGN_TO_POINT, Instructions.MODE_DEBUG);
        }
        if (gamepad2.left_stick_button) {
            mode = 2;
            messages.add(Adresses.ALIGN_TO_POINT, Instructions.MODE_DEBUG);
        }

        if (mode == 0) {
            //switch between targets
            if (gamepad2.dpad_up) {
                Targets.nextTarget();
            }
            if (gamepad2.dpad_down) {
                Targets.previousTarget();
            }

            //AlignToPoint trimming
            if (controller2.getdPadLeft() == Controller.ButtonState.ON_PRESS) {
                messages.add(Adresses.ALIGN_TO_POINT, Instructions.STD_DPAD_LEFT);
            }
            if (controller2.getdPadRight() == Controller.ButtonState.ON_PRESS) {
                messages.add(Adresses.ALIGN_TO_POINT, Instructions.STD_DPAD_RIGHT);
            }
        }
        if (mode == 1) {

            if (controller2.getdPadUp() == Controller.ButtonState.ON_PRESS) {
                if (shooter_on) {
                    messages.add(Adresses.SHOOTER, Instructions.SET_SHOOTER_IDLE);
                    shooter_on = !shooter_on;
                } else {
                    messages.add(Adresses.SHOOTER, Instructions.SET_SHOOTER_ON);
                    shooter_on = !shooter_on;
                }
            }

            if (gamepad2.dpad_down) {
                messages.add(Adresses.INTAKE, Instructions.LOWER_INTAKE_DEBUG);
            }
            if (gamepad2.dpad_left) {
                messages.add(Adresses.SHOOTER, Instructions.ADJUST_FLAP_DEBUG);
            }
            if (gamepad2.dpad_right) {
                messages.add(Adresses.SHOOTER, Instructions.FEED_RING_DEBUG);
            }
        }
        if (mode == 2) {
            if (controller2.getdPadUp() == Controller.ButtonState.ON_PRESS) {
                servoPos -= 0.01;
                messages.add(Adresses.SHOOTER, Instructions.SET_FLAP_POSITION, servoPos);
            }
            if (controller2.getdPadDown() == Controller.ButtonState.ON_PRESS) {
                servoPos += 0.01;
                messages.add(Adresses.SHOOTER, Instructions.SET_FLAP_POSITION, servoPos);
            }
            if (gamepad2.left_trigger > 0) {
                messages.add(Adresses.SHOOTER, Instructions.ADJUST_FLAP_DEBUG, servoPos);
            }
            //toggle shooter
            if (controller2.getdPadLeft() == Controller.ButtonState.ON_PRESS) {
                if (shooter_on) {
                    messages.add(Adresses.SHOOTER, Instructions.SET_SHOOTER_IDLE);
                    shooter_on = !shooter_on;
                } else {
                    messages.add(Adresses.SHOOTER, Instructions.SET_SHOOTER_ON);
                    shooter_on = !shooter_on;
                }
            }
            telemetry.addData("ServoPos: ", servoPos);
        }
        //Shooting
        if (controller2.getRightTrigger() == Controller.ButtonState.PRESSED) {
            messages.add(Adresses.SHOOTER, Instructions.SHOOT_ONE);
        }

        //Transfer on / off
        if (controller2.getxButton() == Controller.ButtonState.ON_PRESS) {
            if (lift_on) {
                messages.add(Adresses.TRANSFER, Instructions.SET_TRANSFER_IDLE);
                messages.add(Adresses.INTAKE, Instructions.SET_INTAKE_IDLE);
                intake_on = false;
                lift_on = !lift_on;
            } else {
                messages.add(Adresses.TRANSFER, Instructions.SET_TRANSFER_ON);
                lift_on = !lift_on;
            }
        }
        //Transfer reverse
        if (gamepad2.y) {
            messages.add(Adresses.TRANSFER, Instructions.SET_TRANSFER_REVERSE);
            messages.add(Adresses.INTAKE, Instructions.SET_INTAKE_REVERSE);
        }
        if (controller2.getyButton() == Controller.ButtonState.ON_RELEASE) {
            if (lift_on) {
                messages.add(Adresses.TRANSFER, Instructions.SET_TRANSFER_ON);
            } else {
                messages.add(Adresses.TRANSFER, Instructions.SET_TRANSFER_IDLE);
                messages.add(Adresses.INTAKE, Instructions.SET_INTAKE_IDLE);
                intake_on = false;
            }
            if (intake_on) {
                messages.add(Adresses.INTAKE, Instructions.SET_INTAKE_ON);
            } else {
                messages.add(Adresses.INTAKE, Instructions.SET_INTAKE_IDLE);
            }
        }

        //Intake on / off
        if (controller2.getaButton() == Controller.ButtonState.ON_PRESS) {
            if (intake_on) {
                messages.add(Adresses.INTAKE, Instructions.SET_INTAKE_IDLE);
                intake_on = !intake_on;
            } else {
                messages.add(Adresses.INTAKE, Instructions.SET_INTAKE_ON);
                messages.add(Adresses.TRANSFER, Instructions.SET_TRANSFER_ON);
                lift_on = true;
                intake_on = !intake_on;
            }
        }
        //Reverse Intake
        if (gamepad2.b) {
            messages.add(Adresses.INTAKE, Instructions.SET_INTAKE_REVERSE);
        }
        if (controller2.getbButton() == Controller.ButtonState.ON_RELEASE) {
            if (intake_on) {
                messages.add(Adresses.INTAKE, Instructions.SET_INTAKE_ON);
                messages.add(Adresses.TRANSFER, Instructions.SET_TRANSFER_ON);
                lift_on = true;
            } else {
                messages.add(Adresses.INTAKE, Instructions.SET_INTAKE_IDLE);
            }
        }

        telemetry.addData("Mode:", mode);
        return messages;
    }
}

