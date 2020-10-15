package org.firstinspires.ftc.teamcode.lib;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.opmodes.tele.FTC_2020_Tele;

public class Drivemode {
    Globals g = new Globals();
    Controller controller;
    Shooting shooting = new Shooting("blue");       //classes

    String mode = "ROBOT";                                  //Idk

    double currentRotation = 0;
    double expectedRotation = 0;
    Vector currentHeading = new Vector();
    Vector lastPos = new Vector();
    Vector currentPos = new Vector();                       //transform

    Vector motorPower = new Vector();           // #   #   x   y        ^
    Vector rotationPower = new Vector();        // |   |   |   |        ^
    Vector translationPowerX = new Vector();    // # - #   z - w        ^
    Vector translationPowerY = new Vector();
    Vector translationPower = new Vector();
    double rotationFactor = 0;                              //Motor

    double error;

    public Drivemode(Gamepad gamepad) {
        controller = new Controller(gamepad);
    }

    public void startTracking() {
        //roadrunner.goto(currentHeading * tuned constant depending on speed, shooting.calculateAngleAlpha)
        mode = "GOAL";
    }

    public void stopTracking() {
        mode = "ROBOT";
    }

    public void loop() {
        lastPos = currentPos;
        //currentPos = roadrunner.getPos();
        currentHeading = currentPos.subtract(lastPos);

        if (mode == "GOAL") {
            //currentRotation = roadrunner.getRotation();
            expectedRotation = shooting.currentTarget.getCoordinates().subtract(currentPos).angle();
            error = currentRotation - expectedRotation;

            if (error > g.e) {
                rotationPower = new Vector(-1.0, 1.0, -1.0, 1.0);
            }
            else {
                rotationPower = new Vector(1.0, -1.0, 1.0, -1.0);
            }

            translationPowerX = new Vector(1,-1,-1,1).multiply(controller.getLeftJoystickXValue());
            translationPowerY = new Vector(1,1,1,1).multiply(controller.getLeftJoystickYValue());
            translationPower = translationPowerX.add(translationPowerY);
            motorPower = rotationPower.multiply(rotationFactor).add(translationPower.multiply(1-rotationFactor));       //f * v1 + (1-f) * v2
        }
    }
}