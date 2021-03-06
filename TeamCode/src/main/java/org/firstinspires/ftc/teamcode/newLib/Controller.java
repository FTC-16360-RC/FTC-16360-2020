package org.firstinspires.ftc.teamcode.newLib;

import com.qualcomm.robotcore.hardware.Gamepad;

public class Controller {

    /*
    //  DECLARATION
    */

    Gamepad gamepad = null;

    public enum ButtonState {
        PRESSED,    // Button state when button is held down.
        RELEASED,   // Button state when button is not pushed up.
        ON_PRESS,   // Button state when button is being pressed down.
        ON_RELEASE  // Button state when button is being released up.
    }

    private double       joystickDeadZone = 0.05;
    private ButtonState  leftJoystickXButton = ButtonState.RELEASED;
    public boolean       leftJoystickX = false;
    private float        leftJoystickXValue = (float) 0.0;
    private ButtonState  leftJoystickYButton = ButtonState.RELEASED;
    private boolean      leftJoystickY = false;
    private float        leftJoystickYValue = (float) 0.0;
    private ButtonState  leftJoystickButton = ButtonState.RELEASED;
    private boolean      leftJoystick = false;

    private ButtonState  rightJoystickXButton = ButtonState.RELEASED;
    private boolean      rightJoystickX = false;
    private float        rightJoystickXValue = (float) 0.0;
    private ButtonState  rightJoystickYButton = ButtonState.RELEASED;
    private boolean      rightJoystickY = false;
    private float        rightJoystickYValue = (float) 0.0;
    private ButtonState  rightJoystickButton = ButtonState.RELEASED;
    private boolean      rightJoystick = false;

    private ButtonState  dPadUpButton = ButtonState.RELEASED;
    public boolean       dPadUp = false;
    private ButtonState  dPadDownButton= ButtonState.RELEASED;
    public boolean       dPadDown = false;
    private ButtonState  dPadLeftButton = ButtonState.RELEASED;
    public boolean       dPadLeft = false;
    private ButtonState  dPadRightButton= ButtonState.RELEASED;
    public boolean       dPadRight = false;

    private ButtonState  aButton = ButtonState.RELEASED;
    public boolean       a = false;
    private ButtonState  bButton = ButtonState.RELEASED;
    public boolean       b = false;
    private ButtonState  xButton = ButtonState.RELEASED;
    public boolean       x = false;
    private ButtonState  yButton = ButtonState.RELEASED;
    public boolean       y = false;

    private ButtonState  leftBumperButton = ButtonState.RELEASED;
    public boolean       leftBumper = false;
    private ButtonState  rightBumperButton = ButtonState.RELEASED;
    public boolean       rightBumper = false;

    private double       triggerDeadZone = 0.05;
    private ButtonState  leftTriggerButton = ButtonState.RELEASED;
    public boolean       leftTrigger = false;
    private float        leftTriggerValue = (float) 0.0;
    private ButtonState  rightTriggerButton = ButtonState.RELEASED;
    public boolean       rightTrigger = false;
    private float        rightTriggerValue = (float) 0.0;

    private ButtonState  backButton = ButtonState.RELEASED;
    public boolean       back = false;
    private ButtonState  guideButton = ButtonState.RELEASED;
    public boolean       guide = false;
    private ButtonState  startButton = ButtonState.RELEASED;
    public boolean       start = false;

    /*
    //  METHODS
    */

    //constructor without deadzones
    public Controller(Gamepad _gamepad){
        gamepad = _gamepad;
    }

    //constructor with deadzones
    public Controller(Gamepad _gamepad, double _joystickDeadZone, double _triggerDeadZone){
        gamepad = _gamepad;
        joystickDeadZone = _joystickDeadZone;
        triggerDeadZone = _triggerDeadZone;
    }

    //gets gamepad values and updates variables
    public void update () {
        // Left Joystick X
        if (Math.abs(gamepad.left_stick_x) > joystickDeadZone) {
            leftJoystickXValue = gamepad.left_stick_x;
            if (leftJoystickXButton == ButtonState.RELEASED) {
                leftJoystickXButton = ButtonState.ON_PRESS;
            } else if (leftJoystickXButton == ButtonState.ON_PRESS) {
                leftJoystickXButton = ButtonState.PRESSED;
            }
        } else {
            leftJoystickXValue = (float) 0.0;
            if (leftJoystickXButton == ButtonState.PRESSED) {
                leftJoystickXButton = ButtonState.ON_RELEASE;
            } else if (leftJoystickXButton == ButtonState.ON_RELEASE) {
                leftJoystickXButton = ButtonState.RELEASED;
            }
        }

        // Left Joystick Y
        if (Math.abs(gamepad.left_stick_y) > joystickDeadZone) {
            leftJoystickYValue = gamepad.left_stick_y;
            if (leftJoystickYButton == ButtonState.RELEASED) {
                leftJoystickYButton = ButtonState.ON_PRESS;
            } else if (leftJoystickYButton == ButtonState.ON_PRESS) {
                leftJoystickYButton = ButtonState.PRESSED;
            }
        } else {
            leftJoystickYValue = (float) 0.0;
            if (leftJoystickYButton == ButtonState.PRESSED) {
                leftJoystickYButton = ButtonState.ON_RELEASE;
            } else if (leftJoystickYButton == ButtonState.ON_RELEASE) {
                leftJoystickYButton = ButtonState.RELEASED;
            }
        }

        // Left Joystick Button
        if (gamepad.left_stick_button) {
            if (leftJoystickButton == ButtonState.RELEASED) {
                leftJoystickButton = ButtonState.ON_PRESS;
            } else if (leftJoystickButton == ButtonState.ON_PRESS) {
                leftJoystickButton = ButtonState.PRESSED;
            }
        } else {
            if (leftJoystickButton == ButtonState.PRESSED) {
                leftJoystickButton = ButtonState.ON_RELEASE;
            } else if (leftJoystickButton == ButtonState.ON_RELEASE) {
                leftJoystickButton = ButtonState.RELEASED;
            }
        }

        // Right Joystick X
        if (Math.abs(gamepad.right_stick_x) > joystickDeadZone) {
            rightJoystickXValue = gamepad.right_stick_x;
            if (rightJoystickXButton == ButtonState.RELEASED) {
                rightJoystickXButton = ButtonState.ON_PRESS;
            } else if (rightJoystickXButton == ButtonState.ON_PRESS) {
                rightJoystickXButton = ButtonState.PRESSED;
            }
        } else {
            rightJoystickXValue = (float) 0.0;
            if (rightJoystickXButton == ButtonState.PRESSED) {
                rightJoystickXButton = ButtonState.ON_RELEASE;
            } else if (rightJoystickXButton == ButtonState.ON_RELEASE) {
                rightJoystickXButton = ButtonState.RELEASED;
            }
        }

        // Right Joystick Y
        if (Math.abs(gamepad.right_stick_y) > joystickDeadZone) {
            rightJoystickYValue = gamepad.right_stick_y;
            if (rightJoystickYButton == ButtonState.RELEASED) {
                rightJoystickYButton = ButtonState.ON_PRESS;
            } else if (rightJoystickYButton == ButtonState.ON_PRESS) {
                rightJoystickYButton = ButtonState.PRESSED;
            }
        } else {
            rightJoystickYValue = (float) 0.0;
            if (rightJoystickYButton == ButtonState.PRESSED) {
                rightJoystickYButton = ButtonState.ON_RELEASE;
            } else if (rightJoystickYButton == ButtonState.ON_RELEASE) {
                rightJoystickYButton = ButtonState.RELEASED;
            }
        }

        // Right Joystick Button
        if (gamepad.right_stick_button) {
            if (rightJoystickButton == ButtonState.RELEASED) {
                rightJoystickButton = ButtonState.ON_PRESS;
            } else if (rightJoystickButton == ButtonState.ON_PRESS) {
                rightJoystickButton = ButtonState.PRESSED;
            }
        } else {
            if (rightJoystickButton == ButtonState.PRESSED) {
                rightJoystickButton = ButtonState.ON_RELEASE;
            } else if (rightJoystickButton == ButtonState.ON_RELEASE) {
                rightJoystickButton = ButtonState.RELEASED;
            }
        }

        // DPad Up
        if (gamepad.dpad_up) {
            if (dPadUpButton == ButtonState.RELEASED) {
               dPadUpButton= ButtonState.ON_PRESS;
            } else if (dPadUpButton == ButtonState.ON_PRESS) {
               dPadUpButton= ButtonState.PRESSED;
            }
        } else {
            if (dPadUpButton == ButtonState.PRESSED) {
               dPadUpButton= ButtonState.ON_RELEASE;
            } else if (dPadUpButton == ButtonState.ON_RELEASE) {
               dPadUpButton= ButtonState.RELEASED;
            }
        }

        // DPad Down
        if (gamepad.dpad_down) {
            if (dPadDownButton== ButtonState.RELEASED) {
                dPadDownButton= ButtonState.ON_PRESS;
            } else if (dPadDownButton== ButtonState.ON_PRESS) {
                dPadDownButton= ButtonState.PRESSED;
            }
        } else {
            if (dPadDownButton== ButtonState.PRESSED) {
                dPadDownButton= ButtonState.ON_RELEASE;
            } else if (dPadDownButton== ButtonState.ON_RELEASE) {
                dPadDownButton= ButtonState.RELEASED;
            }
        }

        // DPad Left
        if (gamepad.dpad_left) {
            if (dPadLeftButton == ButtonState.RELEASED) {
                dPadLeftButton = ButtonState.ON_PRESS;
            } else if (dPadLeftButton == ButtonState.ON_PRESS) {
                dPadLeftButton = ButtonState.PRESSED;
            }
        } else {
            if (dPadLeftButton == ButtonState.PRESSED) {
                dPadLeftButton = ButtonState.ON_RELEASE;
            } else if (dPadLeftButton == ButtonState.ON_RELEASE) {
                dPadLeftButton = ButtonState.RELEASED;
            }
        }

        // DPad Right
        if (gamepad.dpad_right) {
            if (dPadRightButton== ButtonState.RELEASED) {
                dPadRightButton= ButtonState.ON_PRESS;
            } else if (dPadRightButton== ButtonState.ON_PRESS) {
                dPadRightButton= ButtonState.PRESSED;
            }
        } else {
            if (dPadRightButton== ButtonState.PRESSED) {
                dPadRightButton= ButtonState.ON_RELEASE;
            } else if (dPadRightButton== ButtonState.ON_RELEASE) {
                dPadRightButton= ButtonState.RELEASED;
            }
        }

        // A
        if (gamepad.a) {
            if (aButton == ButtonState.RELEASED) {
                aButton = ButtonState.ON_PRESS;
            } else if (aButton == ButtonState.ON_PRESS) {
                aButton = ButtonState.PRESSED;
            }
        } else {
            if (aButton == ButtonState.PRESSED) {
                aButton = ButtonState.ON_RELEASE;
            } else if (aButton == ButtonState.ON_RELEASE) {
                aButton = ButtonState.RELEASED;
            }
        }

        // B
        if (gamepad.b) {
            if (bButton == ButtonState.RELEASED) {
                bButton = ButtonState.ON_PRESS;
            } else if (bButton == ButtonState.ON_PRESS) {
                bButton = ButtonState.PRESSED;
            }
        } else {
            if (bButton == ButtonState.PRESSED) {
                bButton = ButtonState.ON_RELEASE;
            } else if (bButton == ButtonState.ON_RELEASE) {
                bButton = ButtonState.RELEASED;
            }
        }

        // X
        if (gamepad.x) {
            if (xButton == ButtonState.RELEASED) {
                xButton = ButtonState.ON_PRESS;
            } else if (xButton == ButtonState.ON_PRESS) {
                xButton = ButtonState.PRESSED;
            }
        } else {
            if (xButton == ButtonState.PRESSED) {
                xButton = ButtonState.ON_RELEASE;
            } else if (xButton == ButtonState.ON_RELEASE) {
                xButton = ButtonState.RELEASED;
            }
        }

        // Y
        if (gamepad.y) {
            if (yButton == ButtonState.RELEASED) {
                yButton = ButtonState.ON_PRESS;
            } else if (yButton == ButtonState.ON_PRESS) {
                yButton = ButtonState.PRESSED;
            }
        } else {
            if (yButton == ButtonState.PRESSED) {
                yButton = ButtonState.ON_RELEASE;
            } else if (yButton == ButtonState.ON_RELEASE) {
                yButton = ButtonState.RELEASED;
            }
        }

        // Left Bumper
        if (gamepad.left_bumper) {
            if (leftBumperButton == ButtonState.RELEASED) {
                leftBumperButton = ButtonState.ON_PRESS;
            } else if (leftBumperButton == ButtonState.ON_PRESS) {
                leftBumperButton = ButtonState.PRESSED;
            }
        } else {
            if (leftBumperButton == ButtonState.PRESSED) {
                leftBumperButton = ButtonState.ON_RELEASE;
            } else if (leftBumperButton == ButtonState.ON_RELEASE) {
                leftBumperButton = ButtonState.RELEASED;
            }
        }

        // Right Bumper
        if (gamepad.right_bumper) {
            if (rightBumperButton == ButtonState.RELEASED) {
                rightBumperButton = ButtonState.ON_PRESS;
            } else if (rightBumperButton == ButtonState.ON_PRESS) {
                rightBumperButton = ButtonState.PRESSED;
            }
        } else {
            if (rightBumperButton == ButtonState.PRESSED) {
                rightBumperButton = ButtonState.ON_RELEASE;
            } else if (rightBumperButton == ButtonState.ON_RELEASE) {
                rightBumperButton = ButtonState.RELEASED;
            }
        }

        // Left Trigger
        if (gamepad.left_trigger > triggerDeadZone) {
            leftTriggerValue = gamepad.left_trigger;
            if (leftTriggerButton == ButtonState.RELEASED) {
                leftTriggerButton = ButtonState.ON_PRESS;
            } else if (leftTriggerButton == ButtonState.ON_PRESS) {
                leftTriggerButton = ButtonState.PRESSED;
            }
        } else {
            leftTriggerValue = (float) 0.0;
            if (leftTriggerButton == ButtonState.PRESSED) {
                leftTriggerButton = ButtonState.ON_RELEASE;
            } else if (leftTriggerButton == ButtonState.ON_RELEASE) {
                leftTriggerButton = ButtonState.RELEASED;
            }
        }

        // Left Trigger
        if (gamepad.right_trigger > triggerDeadZone) {
            rightTriggerValue = gamepad.left_trigger;
            if (rightTriggerButton == ButtonState.RELEASED) {
                rightTriggerButton = ButtonState.ON_PRESS;
            } else if (rightTriggerButton == ButtonState.ON_PRESS) {
                rightTriggerButton = ButtonState.PRESSED;
            }
        } else {
            rightTriggerValue = (float) 0.0;
            if (rightTriggerButton == ButtonState.PRESSED) {
                rightTriggerButton = ButtonState.ON_RELEASE;
            } else if (rightTriggerButton == ButtonState.ON_RELEASE) {
                rightTriggerButton = ButtonState.RELEASED;
            }
        }

        // Back
        if (gamepad.back) {
            if (backButton == ButtonState.RELEASED) {
                backButton = ButtonState.ON_PRESS;
            } else if (backButton == ButtonState.ON_PRESS) {
                backButton = ButtonState.PRESSED;
            }
        } else {
            if (backButton == ButtonState.PRESSED) {
                backButton = ButtonState.ON_RELEASE;
            } else if (backButton == ButtonState.ON_RELEASE) {
                backButton = ButtonState.RELEASED;
            }
        }

        // Guide
        if (gamepad.guide) {
            if (guideButton == ButtonState.RELEASED) {
                guideButton = ButtonState.ON_PRESS;
            } else if (guideButton == ButtonState.ON_PRESS) {
                guideButton = ButtonState.PRESSED;
            }
        } else {
            if (guideButton == ButtonState.PRESSED) {
                guideButton = ButtonState.ON_RELEASE;
            } else if (guideButton == ButtonState.ON_RELEASE) {
                guideButton = ButtonState.RELEASED;
            }
        }

        // Start
        if (gamepad.start) {
            if (startButton == ButtonState.RELEASED) {
                startButton = ButtonState.ON_PRESS;
            } else if (startButton == ButtonState.ON_PRESS) {
                startButton = ButtonState.PRESSED;
            }
        } else {
            if (startButton == ButtonState.PRESSED) {
                startButton = ButtonState.ON_RELEASE;
            } else if (startButton == ButtonState.ON_RELEASE) {
                startButton = ButtonState.RELEASED;
            }
        }

        //a bool
        if (aButton == ButtonState.PRESSED) {
            a = true;
        } else {
            a = false;
        }
        //b bool
        if (bButton == ButtonState.PRESSED) {
            b = true;
        } else {
            b = false;
        }
        //x bool
        if (xButton == ButtonState.PRESSED) {
            x = true;
        } else {
            x = false;
        }
        //y bool
        if (yButton == ButtonState.PRESSED) {
            y = true;
        } else {
            y = false;
        }
        //bumperLeft bool
        if (leftBumperButton == ButtonState.PRESSED) {
            leftBumper = true;
        } else {
            leftBumper = false;
        }
        //leftBumper bool
        if (leftBumperButton == ButtonState.PRESSED) {
            leftBumper = true;
        } else {
            leftBumper = false;
        }
        //rightBumper bool
        if (rightBumperButton == ButtonState.PRESSED) {
            rightBumper = true;
        } else {
            rightBumper = false;
        }
        //dpadUp bool
        if (dPadUpButton == ButtonState.PRESSED) {
           dPadUp = true;
        } else {
           dPadUp = false;
        }
        //dPadLeft bool
        if (dPadLeftButton == ButtonState.PRESSED) {
            dPadLeft = true;
        } else {
            dPadLeft = false;
        }
        //dPadRight bool
        if (dPadRightButton == ButtonState.PRESSED) {
            dPadRight = true;
        } else {
            dPadRight = false;
        }
        //dPadDown bool
        if (dPadDownButton == ButtonState.PRESSED) {
            dPadDown = true;
        } else {
            dPadDown = false;
        }
        //LJoystickX bool
        if (leftJoystickXButton == ButtonState.PRESSED) {
            leftJoystickX = true;
        } else {
            leftJoystickX = false;
        }
        //LJoystickY bool
        if (leftJoystickYButton == ButtonState.PRESSED) {
            leftJoystickY = true;
        } else {
            leftJoystickY = false;
        }
        //LJoystick bool
        if (leftJoystickButton == ButtonState.PRESSED) {
            leftJoystick = true;
        } else {
            leftJoystick = false;
        }
        //RJoystickX bool
        if (rightJoystickXButton == ButtonState.PRESSED) {
            rightJoystickX = true;
        } else {
            rightJoystickX = false;
        }
        //RJoystickY bool
        if (rightJoystickYButton == ButtonState.PRESSED) {
            rightJoystickY = true;
        } else {
            rightJoystickY = false;
        }
        //RJoystick bool
        if (rightJoystickButton == ButtonState.PRESSED) {
            rightJoystick = true;
        } else {
            rightJoystick = false;
        }
        //Ltrigger bool
        if (leftTriggerButton == ButtonState.PRESSED) {
            leftTrigger = true;
        } else {
            leftTrigger = false;
        }
        //Rtrigger bool
        if (rightTriggerButton == ButtonState.PRESSED) {
            rightTrigger = true;
        } else {
            rightTrigger = false;
        }
        //back bool
        if (backButton == ButtonState.PRESSED) {
            back = true;
        } else {
            back = false;
        }
        //guide bool
        if (guideButton == ButtonState.PRESSED) {
            guide = true;
        } else {
            guide = false;
        }
        //start bool
        if (startButton == ButtonState.PRESSED) {
            start = true;
        } else {
            start = false;
        }

    }

    /*
     ** GETTERS AND SETTERS
     */

    public ButtonState getLeftJoystickX() {
        return leftJoystickXButton;
    }

    public float getLeftJoystickXValue() {
        return leftJoystickXValue;
    }

    public ButtonState getLeftJoystickY() {
        return leftJoystickYButton;
    }

    public float getLeftJoystickYValue() {
        return leftJoystickYValue;
    }

    public ButtonState getLeftJoystickButton() {
        return leftJoystickButton;
    }

    public ButtonState getRightJoystickX() {
        return rightJoystickXButton;
    }

    public float getRightJoystickXValue() {
        return rightJoystickXValue;
    }

    public ButtonState getRightJoystickY() {
        return rightJoystickYButton;
    }

    public float getRightJoystickYValue() {
        return rightJoystickYValue;
    }

    public ButtonState getRightJoystickButton() {
        return rightJoystickButton;
    }

    public ButtonState getdPadUp() {
        return dPadUpButton;
    }

    public ButtonState getdPadDown() {
        return dPadDownButton;
    }

    public ButtonState getdPadLeft() {
        return dPadLeftButton;
    }

    public ButtonState getdPadRight() {
        return dPadRightButton;
    }

    public ButtonState getaButton() {
        return aButton;
    }

    public ButtonState getbButton() {
        return bButton;
    }

    public ButtonState getxButton() {
        return xButton;
    }

    public ButtonState getyButton() {
        return yButton;
    }

    public ButtonState getLeftBumper() {
        return leftBumperButton;
    }

    public ButtonState getRightBumper() {
        return rightBumperButton;
    }

    public ButtonState getLeftTrigger() {
        return leftTriggerButton;
    }

    public float getLeftTriggerValue() {
        return leftTriggerValue;
    }

    public ButtonState getRightTrigger() {
        return rightTriggerButton;
    }

    public float getRightTriggerValue() {
        return rightTriggerValue;
    }

    public ButtonState getBackButton() {
        return backButton;
    }

    public ButtonState getGuideButton() {
        return guideButton;
    }

    public ButtonState getStartButton() {
        return startButton;
    }

    public double getJoystickDeadZone() {
        return joystickDeadZone;
    }

    public void setJoystickDeadZone(double _joystickDeadZone) {
        joystickDeadZone = _joystickDeadZone;
    }

    public double getTriggerDeadZone() {
        return triggerDeadZone;
    }

    public void setTriggerDeadZone(double _triggerDeadZone) {
        triggerDeadZone = _triggerDeadZone;
    }
}