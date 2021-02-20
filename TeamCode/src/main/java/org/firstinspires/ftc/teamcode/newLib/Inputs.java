package org.firstinspires.ftc.teamcode.newLib;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class Inputs {

    int mode = 0;
    Gamepad gamepad1;
    Gamepad gamepad2;
    Controller controller1;
    Controller controller2;
    Telemetry telemetry;

    public Inputs() {
        gamepad1 = Comms.gamepad1;
        gamepad2 = Comms.gamepad2;
        controller1 = new Controller(Comms.gamepad1);
        controller2 = new Controller(Comms.gamepad2);
        telemetry = Comms.telemetry;
    }


    public void update() {
        Comms.tasks.clear();

        controller1.update();
        controller2.update();

        //drivemodes
        if (controller1.getaButton() == Controller.ButtonState.ON_PRESS) {
            Comms.tasks.add(Comms.Tasks.SET_GOAL_CENTRIC);
        }
        if (controller1.getbButton() == Controller.ButtonState.ON_PRESS) {
            Comms.tasks.add(Comms.Tasks.SET_ROBOT_CENTRIC);
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

        if (controller1.getyButton() == Controller.ButtonState.ON_PRESS && controller1.getxButton() != Controller.ButtonState.PRESSED) {
            Comms.tasks.add(Comms.Tasks.DISABLE_INTAKE);
            Comms.tasks.add(Comms.Tasks.DISABLE_TRANSFER);
        }


        //reset Odometry orientation
        if (gamepad1.dpad_up) {
            //messages.add(Adresses.ALIGN_TO_POINT, Instructions.RESET_ORIENTATION);
        }

        //AlignToPoint trimming
        if (controller1.getdPadLeft() == Controller.ButtonState.ON_PRESS) {
            //messages.add(Adresses.ALIGN_TO_POINT, Instructions.STD_DPAD_LEFT);
        }
        if (controller1.getdPadRight() == Controller.ButtonState.ON_PRESS) {
            //messages.add(Adresses.ALIGN_TO_POINT, Instructions.STD_DPAD_RIGHT);
        }

        //switch between targets
        if (controller1.getdPadUp() == Controller.ButtonState.ON_PRESS) {
            Targets.nextTarget();
        }
        if (controller1.getdPadDown() == Controller.ButtonState.ON_PRESS) {
            Targets.previousTarget();
        }

        //
        //Gamepad 2
        //

        //switch between modes
        if (gamepad2.left_bumper){
            mode = 0;
        }
        if (gamepad2.right_bumper) {
            mode = 1;
        }
        if (gamepad2.left_stick_button) {
            mode = 2;
        }

        if (mode == 0) {
            //switch between targets
            if (controller2.getdPadUp() == Controller.ButtonState.ON_PRESS) {
                Targets.nextTarget();
            }
            if (controller2.getdPadDown() == Controller.ButtonState.ON_PRESS) {
                Targets.previousTarget();
            }

            //AlignToPoint trimming
            if (controller2.getdPadLeft() == Controller.ButtonState.ON_PRESS) {
                //messages.add(Adresses.ALIGN_TO_POINT, Instructions.STD_DPAD_LEFT);
            }
            if (controller2.getdPadRight() == Controller.ButtonState.ON_PRESS) {
                //messages.add(Adresses.ALIGN_TO_POINT, Instructions.STD_DPAD_RIGHT);
            }
        }
        if (mode == 1) {

            if (controller2.getdPadUp() == Controller.ButtonState.ON_PRESS) {
                /*if (shooter_on) {
                    messages.add(Adresses.SHOOTER, Instructions.SET_SHOOTER_IDLE);
                    shooter_on = !shooter_on;
                } else {
                    messages.add(Adresses.SHOOTER, Instructions.SET_SHOOTER_ON);
                    shooter_on = !shooter_on;
                }*/
            }

            if (gamepad2.dpad_down) {
                //messages.add(Adresses.INTAKE, Instructions.LOWER_INTAKE_DEBUG);
            }
            if (gamepad2.dpad_left) {
                //messages.add(Adresses.SHOOTER, Instructions.ADJUST_FLAP_DEBUG);
            }
            if (gamepad2.dpad_right) {
                //messages.add(Adresses.SHOOTER, Instructions.FEED_RING_DEBUG);
            }
        }
        if (mode == 2) {
            if (controller2.getdPadUp() == Controller.ButtonState.ON_PRESS) {
                /*servoPos -= 0.01;
                messages.add(Adresses.SHOOTER, Instructions.SET_FLAP_POSITION, servoPos);
            }
            if (controller2.getdPadDown() == Controller.ButtonState.ON_PRESS) {
                servoPos += 0.01;
                messages.add(Adresses.SHOOTER, Instructions.SET_FLAP_POSITION, servoPos);
            }
            if (gamepad2.left_trigger > 0) {
                messages.add(Adresses.SHOOTER, Instructions.ADJUST_FLAP_DEBUG, servoPos);
                 */
            }
            //toggle shooter
            if (controller2.getdPadLeft() == Controller.ButtonState.ON_PRESS) {
                //Comms.tasks.add(Comms.Tasks.TOGGLE_SHOOTER);
            }
        }

        //toggle shooter
        if (controller2.getdPadLeft() == Controller.ButtonState.ON_PRESS) {
            Comms.tasks.add(Comms.Tasks.TOGGLE_SHOOTER);
        }

        //Shooting
        if (controller2.getRightTrigger() == Controller.ButtonState.PRESSED) {
            Comms.tasks.add(Comms.Tasks.SHOOT);
        }

        //Transfer on / off
        if (controller2.getyButton() == Controller.ButtonState.ON_PRESS) {
            Comms.tasks.add(Comms.Tasks.TOGGLE_TRANSFER);
            Comms.tasks.add(Comms.Tasks.DISABLE_INTAKE);
        }
        //Transfer reverse
        if (controller2.getxButton() == Controller.ButtonState.ON_PRESS) {
            Comms.tasks.add(Comms.Tasks.REVERSE_TRANSFER);
        }
        if (controller2.getxButton() == Controller.ButtonState.ON_RELEASE) {
            Comms.tasks.add(Comms.Tasks.RESET_TRANSFER);
            Comms.tasks.add(Comms.Tasks.RESET_INTAKE);
        }

        //Intake on / off
        if (controller2.getbButton() == Controller.ButtonState.ON_PRESS) {
            Comms.tasks.add(Comms.Tasks.TOGGLE_INTAKE);
            Comms.tasks.add(Comms.Tasks.DISABLE_TRANSFER);
        }

        //Intake reverse
        if (controller2.getaButton() == Controller.ButtonState.ON_PRESS) {
            Comms.tasks.add(Comms.Tasks.REVERSE_INTAKE);
        }
        if (controller2.getaButton() == Controller.ButtonState.ON_RELEASE) {
            Comms.tasks.add(Comms.Tasks.RESET_INTAKE);
        }
    }
}


