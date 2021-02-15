package org.firstinspires.ftc.teamcode.newLib;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.lib.Controller;


public class Inputs {

    int mode = 0;
    Gamepad gamepad1;
    Gamepad gamepad2;
    Controller controller1;
    Controller controller2;

    public Inputs() {
        gamepad1 = Comms.gamepad1;
        gamepad2 = Comms.gamepad2;
        controller1 = new Controller(Comms.gamepad1);
        controller2 = new Controller(Comms.gamepad2);
    }


    public void update() {
        Comms.tasks.clear();

        controller1.update();
        controller2.update();

        //drivemodes
        if (controller1.getaButton() == Controller.ButtonState.ON_PRESS && Comms.driveMode != Comms.DriveMode.MODIFIED_GOAL_CENTRIC) {
            if (Comms.driveMode == Comms.DriveMode.MODIFIED_ROBOT_CENTRIC) {
                Comms.driveMode = Comms.DriveMode.MODIFIED_GOAL_CENTRIC;
            } else {
                Comms.driveMode = Comms.DriveMode.GOAL_CENTRIC;
            }
        }
        if (controller1.getbButton() == Controller.ButtonState.ON_PRESS && Comms.driveMode != Comms.DriveMode.MODIFIED_ROBOT_CENTRIC) {
            if (Comms.driveMode == Comms.DriveMode.MODIFIED_GOAL_CENTRIC) {
                Comms.driveMode = Comms.DriveMode.MODIFIED_ROBOT_CENTRIC;
            } else {
                Comms.driveMode = Comms.DriveMode.ROBOT_CENTRIC;
            }
        }

        //shooting
        if (gamepad1.right_trigger != 0) {
            Comms.tasks.add(Comms.Tasks.SHOOT);
        }

        //Reverse intake / transfer
        if (controller1.getxButton() == Controller.ButtonState.ON_PRESS) {
            Comms.tasks.add(Comms.Tasks.REVERSE_TRANSFER);
        }
        if (controller1.getxButton() == Controller.ButtonState.ON_RELEASE) {
            Comms.tasks.add(Comms.Tasks.RESET_TRANSFER);
            Comms.tasks.add(Comms.Tasks.RESET_INTAKE);
        }


        //reset Odometry orientation
        if (gamepad1.dpad_up) {
            messages.add(Adresses.ALIGN_TO_POINT, Instructions.RESET_ORIENTATION);
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
            Comms.tasks.add(Comms.Tasks.SHOOT);
        }

        //Transfer on / off
        if (controller2.getxButton() == Controller.ButtonState.ON_PRESS) {
            Comms.tasks.add(Comms.Tasks.TOGGLE_TRANSFER);
        }
        //Transfer reverse
        if (controller1.getxButton() == Controller.ButtonState.ON_PRESS) {
            Comms.tasks.add(Comms.Tasks.REVERSE_TRANSFER);
        }
        if (controller1.getxButton() == Controller.ButtonState.ON_RELEASE) {
            Comms.tasks.add(Comms.Tasks.RESET_TRANSFER);
        }

        //Intake on / off
        if (controller2.getaButton() == Controller.ButtonState.ON_PRESS) {
            Comms.tasks.add(Comms.Tasks.TOGGLE_INTAKE);
        }

        //Intake reverse
        if (controller1.getxButton() == Controller.ButtonState.ON_PRESS) {
            Comms.tasks.add(Comms.Tasks.REVERSE_INTAKE);
        }
        if (controller1.getxButton() == Controller.ButtonState.ON_RELEASE) {
            Comms.tasks.add(Comms.Tasks.RESET_INTAKE);
        }
    }
}

