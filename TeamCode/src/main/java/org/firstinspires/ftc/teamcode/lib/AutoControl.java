package org.firstinspires.ftc.teamcode.lib;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.control.Capstone;
import org.firstinspires.ftc.teamcode.control.DriveAuto;
import org.firstinspires.ftc.teamcode.control.Extension;
import org.firstinspires.ftc.teamcode.control.FoundationPuller;
import org.firstinspires.ftc.teamcode.control.Gripper;
import org.firstinspires.ftc.teamcode.control.Intake;
import org.firstinspires.ftc.teamcode.control.Lift;

public class AutoControl {
    private DriveAuto driveAuto;

    private boolean ready = false;

    private int maxStages;
    private int currentStage;
    private boolean stageCompleted;

    private boolean red = false;
    private int skystonePosition;

    private double currentRuntime = 0;

    public enum AutoMode
    {
        TEST,
        FOUNDATION_RED,
        FOUNDATION_BLUE,
        FOUNDATION_BLUE_CLOSE,
        FOUNDATION_RED_CLOSE,
        SKYSTONES_BLUE,
        SKYSTONES_RED,
        DOUBLE_SKYSTONES_BLUE,
        DOUBLE_SKYSTONES_RED,
        PARK
    }

    private AutoMode autoMode;

    private FoundationPuller foundationPuller;

    private Extension extension;

    private SkystoneDetector skystoneDetector;

    private Intake intake;

    private Gripper gripper;

    private Lift lift;

    private Capstone capstone;

    public AutoControl(HardwareMap hardwareMap, boolean allianceRed)
    {
        driveAuto = new DriveAuto(hardwareMap, DcMotor.RunMode.RUN_USING_ENCODER, DcMotor.ZeroPowerBehavior.BRAKE);
        foundationPuller = new FoundationPuller(hardwareMap);
        extension = new Extension(hardwareMap);
        skystoneDetector = new SkystoneDetector(hardwareMap, allianceRed);
        intake = new Intake(hardwareMap);
        gripper = new Gripper(hardwareMap);
        lift = new Lift(hardwareMap);
        capstone = new Capstone(hardwareMap);
    }

    private void checkStateCompletion()
    {
        if(driveAuto.getCompleted())
        {
            stageCompleted = true;
            driveAuto.resetOrder();
        }
    }

    private void initialiseTest()
    {
        maxStages = 7;
        currentStage = 0;
    }

    private void initialiseFoundationBlue()
    {
        maxStages = 13;
        currentStage = 0;
    }

    private void initialiseFoundationRed()
    {
        maxStages = 13;
        currentStage = 0;
    }

    private void initialiseFoundationRedClose()
    {
        maxStages = 14;
        currentStage = 0;
    }

    private void initialiseFoundationBlueClose()
    {
        maxStages = 14;
        currentStage = 0;
    }

    private void initialisePark()
    {
        maxStages = 2;
        currentStage = 0;
    }

    private void initialiseSkystonesBlue()
    {
        skystoneDetector.init();
        maxStages = 21;
        currentStage = 0;
    }

    private void initialiseSkystonesRed()
    {
        skystoneDetector.init();
        maxStages = 21;
        currentStage = 0;
    }

    private void initialiseDoubleSkystones()
    {
        skystoneDetector.init();
        maxStages = 21;
        currentStage = 0;
    }

    //the test auto program
    private void runTest()
    {
        switch(currentStage)
        {
            case 0:
                if(ready) {
                    extension.forceRetract();
                    stageCompleted = true;
                }
                break;
            case 1:
                driveAuto.setMaxDriveSpeed(0.6);
                driveAuto.accelerate(DriveAuto.DriveDirection.FORWARD, 100);
                checkStateCompletion();
                break;
            case 2:
                //driveAuto.setAccelerate(false);
                driveAuto.brake(DriveAuto.DriveDirection.FORWARD, 100);
                checkStateCompletion();
                break;
            case 3:
                driveAuto.setMaxDriveSpeed(1);
                driveAuto.drive(DriveAuto.DriveDirection.BACKWARD, 100);
                break;
                /*
            case 4:
                driveAuto.drive(DriveAuto.DriveDirection.BACKWARD, 40);
                checkStateCompletion();
                break;
            case 5:
                driveAuto.turn(DriveAuto.TurnDirection.RIGHT, 270);
                checkStateCompletion();
                break;
            case 6:
                driveAuto.drive(DriveAuto.DriveDirection.RIGHT, 20);
                checkStateCompletion();
                break;
            case 7:
                driveAuto.drive(DriveAuto.DriveDirection.FORWARD, 40);
                checkStateCompletion();
                break;
                */
        }
    }

    //the test auto program
    private void runFoundationBlue()
    {
        switch(currentStage)
        {
            case 0:
                if(ready) {
                    stageCompleted = true;
                }
                break;
            case 1:
                driveAuto.drive(DriveAuto.DriveDirection.LEFT, 40);
                checkStateCompletion();
                break;
            case 2:
                driveAuto.drive(DriveAuto.DriveDirection.FORWARD, 70);
                checkStateCompletion();
                break;
            case 3:
                foundationPuller.setServoState(FoundationPuller.ServoState.CLOSED);
                extension.forceRetract();
                stageCompleted = true;
                break;
            case 4:
                driveAuto.drive(DriveAuto.DriveDirection.FORWARD, 15);
                checkStateCompletion();
                break;
            case 5:
                driveAuto.turn(DriveAuto.TurnDirection.LEFT, 18);
                checkStateCompletion();
                break;
            case 6:
                driveAuto.drive(DriveAuto.DriveDirection.BACKWARD, 55);
                checkStateCompletion();
                break;
            case 7:
                driveAuto.turn(DriveAuto.TurnDirection.LEFT, 72);
                checkStateCompletion();
                break;
            case 8:
                foundationPuller.setServoState(FoundationPuller.ServoState.OPEN);
                stageCompleted = true;
                break;
            case 9:
                driveAuto.drive(DriveAuto.DriveDirection.FORWARD, 30);
                checkStateCompletion();
                break;
            case 10:
                driveAuto.drive(DriveAuto.DriveDirection.RIGHT, 15);
                checkStateCompletion();
                break;
            case 11:
                driveAuto.drive(DriveAuto.DriveDirection.BACKWARD, 20);
                checkStateCompletion();
                break;
            case 12:
                driveAuto.turn(DriveAuto.TurnDirection.LEFT, 180);
                checkStateCompletion();
                break;
            case 13:
                driveAuto.drive(DriveAuto.DriveDirection.FORWARD, 80);
                checkStateCompletion();
                break;
        }
    }

    //the test auto program
    private void runFoundationRed()
    {
        switch(currentStage)
        {
            case 0:
                if(ready) {
                    stageCompleted = true;
                }
                break;
            case 1:
                driveAuto.drive(DriveAuto.DriveDirection.RIGHT, 40);
                checkStateCompletion();
                break;
            case 2:
                driveAuto.drive(DriveAuto.DriveDirection.FORWARD, 70);
                checkStateCompletion();
                break;
            case 3:
                foundationPuller.setServoState(FoundationPuller.ServoState.CLOSED);
                extension.forceRetract();
                stageCompleted = true;
                break;
            case 4:
                driveAuto.drive(DriveAuto.DriveDirection.FORWARD, 15);
                checkStateCompletion();
                break;
            case 5:
                driveAuto.turn(DriveAuto.TurnDirection.RIGHT, 18);
                checkStateCompletion();
                break;
            case 6:
                driveAuto.drive(DriveAuto.DriveDirection.BACKWARD, 55);
                checkStateCompletion();
                break;
            case 7:
                driveAuto.turn(DriveAuto.TurnDirection.RIGHT, 72);
                checkStateCompletion();
                break;
            case 8:
                foundationPuller.setServoState(FoundationPuller.ServoState.OPEN);
                stageCompleted = true;
                break;
            case 9:
                driveAuto.drive(DriveAuto.DriveDirection.FORWARD, 30);
                checkStateCompletion();
                break;
            case 10:
                driveAuto.drive(DriveAuto.DriveDirection.LEFT, 15);
                checkStateCompletion();
                break;
            case 11:
                driveAuto.drive(DriveAuto.DriveDirection.BACKWARD, 20);
                checkStateCompletion();
                break;
            case 12:
                driveAuto.turn(DriveAuto.TurnDirection.RIGHT, 180);
                checkStateCompletion();
                break;
            case 13:
                driveAuto.drive(DriveAuto.DriveDirection.FORWARD, 80);
                checkStateCompletion();
                break;
        }
    }

    //the test auto program
    private void runFoundationBlueClose() {
        switch (currentStage) {
            case 0:
                if (ready) {
                    stageCompleted = true;
                }
                break;
            case 1:
                driveAuto.drive(DriveAuto.DriveDirection.LEFT, 40);
                checkStateCompletion();
                break;
            case 2:
                driveAuto.drive(DriveAuto.DriveDirection.FORWARD, 70);
                checkStateCompletion();
                break;
            case 3:
                foundationPuller.setServoState(FoundationPuller.ServoState.CLOSED);
                extension.forceRetract();
                stageCompleted = true;
                break;
            case 4:
                driveAuto.drive(DriveAuto.DriveDirection.FORWARD, 15);
                checkStateCompletion();
                break;
            case 5:
                driveAuto.turn(DriveAuto.TurnDirection.LEFT, 18);
                checkStateCompletion();
                break;
            case 6:
                driveAuto.drive(DriveAuto.DriveDirection.BACKWARD, 55);
                checkStateCompletion();
                break;
            case 7:
                driveAuto.turn(DriveAuto.TurnDirection.LEFT, 72);
                checkStateCompletion();
                break;
            case 8:
                foundationPuller.setServoState(FoundationPuller.ServoState.OPEN);
                stageCompleted = true;
                break;
            case 9:
                driveAuto.drive(DriveAuto.DriveDirection.FORWARD, 30);
                checkStateCompletion();
                break;
            case 10:
                driveAuto.drive(DriveAuto.DriveDirection.LEFT, 40);
                checkStateCompletion();
                break;
            case 11:
                driveAuto.drive(DriveAuto.DriveDirection.BACKWARD, 20);
                checkStateCompletion();
                break;
            case 12:
                driveAuto.turn(DriveAuto.TurnDirection.LEFT, 180);
                checkStateCompletion();
                break;
            case 13:
                driveAuto.drive(DriveAuto.DriveDirection.RIGHT, 25);
                checkStateCompletion();
                break;
            case 14:
                driveAuto.drive(DriveAuto.DriveDirection.FORWARD, 80);
                checkStateCompletion();
                break;
        }
    }

    //the test auto program
    private void runFoundationRedClose()
    {
        switch(currentStage)
        {
            case 0:
                if(ready) {
                    stageCompleted = true;
                }
                break;
            case 1:
                driveAuto.drive(DriveAuto.DriveDirection.RIGHT, 40);
                checkStateCompletion();
                break;
            case 2:
                driveAuto.drive(DriveAuto.DriveDirection.FORWARD, 70);
                checkStateCompletion();
                break;
            case 3:
                foundationPuller.setServoState(FoundationPuller.ServoState.CLOSED);
                extension.forceRetract();
                stageCompleted = true;
                break;
            case 4:
                driveAuto.drive(DriveAuto.DriveDirection.FORWARD, 15);
                checkStateCompletion();
                break;
            case 5:
                driveAuto.turn(DriveAuto.TurnDirection.RIGHT, 18);
                checkStateCompletion();
                break;
            case 6:
                driveAuto.drive(DriveAuto.DriveDirection.BACKWARD, 55);
                checkStateCompletion();
                break;
            case 7:
                driveAuto.turn(DriveAuto.TurnDirection.RIGHT, 72);
                checkStateCompletion();
                break;
            case 8:
                foundationPuller.setServoState(FoundationPuller.ServoState.OPEN);
                stageCompleted = true;
                break;
            case 9:
                driveAuto.drive(DriveAuto.DriveDirection.FORWARD, 30);
                checkStateCompletion();
                break;
            case 10:
                driveAuto.drive(DriveAuto.DriveDirection.RIGHT, 40);
                checkStateCompletion();
                break;
            case 11:
                driveAuto.drive(DriveAuto.DriveDirection.BACKWARD, 20);
                checkStateCompletion();
                break;
            case 12:
                driveAuto.turn(DriveAuto.TurnDirection.RIGHT, 180);
                checkStateCompletion();
                break;
            case 13:
                driveAuto.drive(DriveAuto.DriveDirection.LEFT, 25);
                checkStateCompletion();
                break;
            case 14:
                driveAuto.drive(DriveAuto.DriveDirection.FORWARD, 80);
                checkStateCompletion();
                break;
        }
    }

    //the test auto program
    private void runSkystonesBlue() {
        switch (currentStage) {
            case 0:
                if (ready) {
                    driveAuto.setMaxDriveSpeed(0.7);
                    skystonePosition = skystoneDetector.getPosition();
                    extension.forceRetract();
                    stageCompleted = true;
                }
                break;
            case 1:
                driveAuto.drive(DriveAuto.DriveDirection.LEFT, 20);
                checkStateCompletion();
                break;
            case 2:
                if (skystonePosition == 0) {
                    driveAuto.drive(DriveAuto.DriveDirection.FORWARD, 7);
                } else if(skystonePosition == 1) {
                    driveAuto.drive(DriveAuto.DriveDirection.FORWARD, 23);
                } else {
                    driveAuto.drive(DriveAuto.DriveDirection.FORWARD, 45);
                }
                checkStateCompletion();
                break;
            case 3:
                driveAuto.turn(DriveAuto.TurnDirection.LEFT, 90);
                checkStateCompletion();
                break;
            case 4:
                driveAuto.setMaxDriveSpeed(0.5);
                foundationPuller.setServoState(FoundationPuller.ServoState.AUTO_MOVE);
                foundationPuller.setGrabberStateLeft(FoundationPuller.GrabberState.PREPARED);
                driveAuto.drive(DriveAuto.DriveDirection.FORWARD, 38);
                checkStateCompletion();
                break;
            case 5:
                foundationPuller.setGrabberStateLeft(FoundationPuller.GrabberState.CLOSED);
                driveAuto.drive(DriveAuto.DriveDirection.FORWARD, 5);
                checkStateCompletion();
                break;
            case 6:
                driveAuto.setMaxDriveSpeed(0.8);
                foundationPuller.setServoState(FoundationPuller.ServoState.HOLD_STONE);
                driveAuto.drive(DriveAuto.DriveDirection.BACKWARD, 8);
                checkStateCompletion();
                break;
            case 7:
                driveAuto.drive(DriveAuto.DriveDirection.LEFT, 165 + 20 * skystonePosition);
                checkStateCompletion();
                break;
            case 8:
                foundationPuller.setServoState(FoundationPuller.ServoState.DEPOSIT);
                driveAuto.drive(DriveAuto.DriveDirection.FORWARD, 22);
                checkStateCompletion();
                break;
            case 9:
                foundationPuller.setGrabberStateLeft(FoundationPuller.GrabberState.OPEN);
                driveAuto.drive(DriveAuto.DriveDirection.BACKWARD, 22);
                checkStateCompletion();
                break;
            case 10:
                foundationPuller.setServoState(FoundationPuller.ServoState.AUTO_MOVE);
                driveAuto.drive(DriveAuto.DriveDirection.RIGHT, 192 + 20 * skystonePosition);
                checkStateCompletion();
                break;
            case 11:
                driveAuto.setMaxDriveSpeed(0.5);
                foundationPuller.setGrabberStateRight(FoundationPuller.GrabberState.PREPARED);
                driveAuto.drive(DriveAuto.DriveDirection.FORWARD, 10);
                checkStateCompletion();
                break;
            case 12:
                foundationPuller.setGrabberStateRight(FoundationPuller.GrabberState.CLOSED);
                driveAuto.drive(DriveAuto.DriveDirection.FORWARD, 5);
                checkStateCompletion();
                break;
            case 13:
                driveAuto.setMaxDriveSpeed(0.8);
                foundationPuller.setServoState(FoundationPuller.ServoState.HOLD_STONE);
                driveAuto.drive(DriveAuto.DriveDirection.BACKWARD, 15);
                checkStateCompletion();
                break;
            case 14:
                driveAuto.drive(DriveAuto.DriveDirection.LEFT, 175 + 20 * skystonePosition);
                checkStateCompletion();
                break;
            case 15:
                foundationPuller.setServoState(FoundationPuller.ServoState.DEPOSIT);
                foundationPuller.setGrabberStateRight(FoundationPuller.GrabberState.OPEN);
                driveAuto.drive(DriveAuto.DriveDirection.LEFT, 60);
                checkStateCompletion();
                break;
            case 16:
                driveAuto.drive(DriveAuto.DriveDirection.FORWARD, 30);
                checkStateCompletion();
                break;
            case 17:
                foundationPuller.setServoState(FoundationPuller.ServoState.CLOSED);
                driveAuto.drive(DriveAuto.DriveDirection.FORWARD, 10);
                checkStateCompletion();
                break;
            case 18:
                driveAuto.specialTurn(DriveAuto.TurnDirection.LEFT);
                checkStateCompletion();
                break;
            case 19:
                foundationPuller.setServoState(FoundationPuller.ServoState.OPEN);
                driveAuto.drive(DriveAuto.DriveDirection.FORWARD, 45);
                checkStateCompletion();
                break;
            case 20:
                driveAuto.turn(DriveAuto.TurnDirection.LEFT, 16);
                checkStateCompletion();
                break;
            case 21:
                driveAuto.drive(DriveAuto.DriveDirection.BACKWARD, 95);
                checkStateCompletion();
                break;
        }
    }

        //the test auto program
        private void runSkystonesRed()
        {
            switch (currentStage) {
                case 0:
                    if (ready) {
                        driveAuto.setMaxDriveSpeed(0.8);
                        skystonePosition = skystoneDetector.getPosition();
                        extension.forceRetract();
                        stageCompleted = true;
                        skystonePosition = Math.abs(skystonePosition-2);
                    }
                    break;
                case 1:
                    driveAuto.drive(DriveAuto.DriveDirection.LEFT, 20);
                    checkStateCompletion();
                    break;
                case 2:
                    if (skystonePosition == 0) {
                        driveAuto.drive(DriveAuto.DriveDirection.FORWARD, 15);
                    } else if(skystonePosition == 1) {
                        driveAuto.drive(DriveAuto.DriveDirection.BACKWARD, 4);
                    } else {
                        driveAuto.drive(DriveAuto.DriveDirection.BACKWARD, 20);
                    }
                    checkStateCompletion();
                    break;
                case 3:
                    driveAuto.turn(DriveAuto.TurnDirection.LEFT, 90);
                    checkStateCompletion();
                    break;
                case 4:
                    driveAuto.setMaxDriveSpeed(0.4);
                    foundationPuller.setServoState(FoundationPuller.ServoState.AUTO_MOVE);
                    foundationPuller.setGrabberStateRight(FoundationPuller.GrabberState.PREPARED);
                    driveAuto.drive(DriveAuto.DriveDirection.FORWARD, 38);
                    checkStateCompletion();
                    break;
                case 5:
                    foundationPuller.setGrabberStateRight(FoundationPuller.GrabberState.CLOSED);
                    driveAuto.drive(DriveAuto.DriveDirection.FORWARD, 5);
                    checkStateCompletion();
                    break;
                case 6:
                    driveAuto.setMaxDriveSpeed(0.8);
                    foundationPuller.setServoState(FoundationPuller.ServoState.HOLD_STONE);
                    driveAuto.drive(DriveAuto.DriveDirection.BACKWARD, 20);
                    checkStateCompletion();
                    break;
                case 7:
                    driveAuto.drive(DriveAuto.DriveDirection.RIGHT, 165 + 20 * skystonePosition);
                    checkStateCompletion();
                    break;
                case 8:
                    foundationPuller.setServoState(FoundationPuller.ServoState.DEPOSIT);
                    driveAuto.drive(DriveAuto.DriveDirection.FORWARD, 10);
                    checkStateCompletion();
                    break;
                case 9:
                    foundationPuller.setGrabberStateRight(FoundationPuller.GrabberState.OPEN);
                    driveAuto.drive(DriveAuto.DriveDirection.BACKWARD, 10);
                    checkStateCompletion();
                    break;
                case 10:
                    foundationPuller.setServoState(FoundationPuller.ServoState.AUTO_MOVE);
                    driveAuto.drive(DriveAuto.DriveDirection.LEFT, 185 + 20 * skystonePosition);
                    checkStateCompletion();
                    break;
                case 11:
                    driveAuto.setMaxDriveSpeed(0.5);
                    foundationPuller.setGrabberStateLeft(FoundationPuller.GrabberState.PREPARED);
                    driveAuto.drive(DriveAuto.DriveDirection.FORWARD, 22);
                    checkStateCompletion();
                    break;
                case 12:
                    foundationPuller.setGrabberStateLeft(FoundationPuller.GrabberState.CLOSED);
                    driveAuto.drive(DriveAuto.DriveDirection.FORWARD, 5);
                    checkStateCompletion();
                    break;
                case 13:
                    driveAuto.setMaxDriveSpeed(0.8);
                    foundationPuller.setServoState(FoundationPuller.ServoState.HOLD_STONE);
                    driveAuto.drive(DriveAuto.DriveDirection.BACKWARD, 22);
                    checkStateCompletion();
                    break;
                case 14:
                    driveAuto.drive(DriveAuto.DriveDirection.RIGHT, 170 + 20 * skystonePosition);
                    checkStateCompletion();
                    break;
                case 15:
                    foundationPuller.setServoState(FoundationPuller.ServoState.DEPOSIT);
                    foundationPuller.setGrabberStateLeft(FoundationPuller.GrabberState.OPEN);
                    driveAuto.drive(DriveAuto.DriveDirection.RIGHT, 60);
                    checkStateCompletion();
                    break;
                case 16:
                    driveAuto.drive(DriveAuto.DriveDirection.FORWARD, 18);
                    checkStateCompletion();
                    break;
                case 17:
                    foundationPuller.setServoState(FoundationPuller.ServoState.CLOSED);
                    driveAuto.drive(DriveAuto.DriveDirection.FORWARD, 10);
                    checkStateCompletion();
                    break;
                case 18:
                    driveAuto.specialTurn(DriveAuto.TurnDirection.RIGHT);
                    checkStateCompletion();
                    break;
                case 19:
                    foundationPuller.setServoState(FoundationPuller.ServoState.OPEN);
                    driveAuto.drive(DriveAuto.DriveDirection.FORWARD, 38);
                    checkStateCompletion();
                    break;
                case 20:
                    driveAuto.turn(DriveAuto.TurnDirection.RIGHT, 10);
                    checkStateCompletion();
                    break;
                case 21:
                    driveAuto.drive(DriveAuto.DriveDirection.BACKWARD, 95);
                    checkStateCompletion();
                    break;
            }
        }
    /*switch(currentStage)
        {
            case 0:
                if(ready) {
                    skystonePosition = skystoneDetector.getPosition();
                    extension.forceRetract();
                    stageCompleted = true;
                }
                break;
            case 1:
                driveAuto.drive(DriveAuto.DriveDirection.LEFT, 62);
                checkStateCompletion();
                break;
            case 2:
                foundationPuller.setServoState(FoundationPuller.ServoState.LEFT);
                if(skystonePosition == 0)
                {
                    driveAuto.drive(DriveAuto.DriveDirection.BACKWARD, 5);
                } else {
                    driveAuto.drive(DriveAuto.DriveDirection.FORWARD, 20*skystonePosition-5);
                }
                checkStateCompletion();
                break;
            case 3:
                driveAuto.drive(DriveAuto.DriveDirection.LEFT, 40);
                checkStateCompletion();
                break;
            case 4:
                foundationPuller.setServoState(FoundationPuller.ServoState.OPEN);
                intake.setIntakeMode(Intake.IntakeMode.INTAKE);
                driveAuto.drive(DriveAuto.DriveDirection.FORWARD, 30);
                checkStateCompletion();
                break;
            case 5:
                driveAuto.drive(DriveAuto.DriveDirection.RIGHT, 40);
                checkStateCompletion();
                break;
            case 6:
                driveAuto.turn(DriveAuto.TurnDirection.RIGHT, 180);
                checkStateCompletion();
                break;
            case 7:
                driveAuto.drive(DriveAuto.DriveDirection.FORWARD, 140+skystonePosition*20);
                checkStateCompletion();
                break;
            case 8:
                foundationPuller.setServoState(FoundationPuller.ServoState.RIGHT);
                intake.setIntakeMode(Intake.IntakeMode.OUTTAKE);
                driveAuto.drive(DriveAuto.DriveDirection.BACKWARD, 145+skystonePosition*20);
                checkStateCompletion();
                break;
            case 9:
                intake.setIntakeMode(Intake.IntakeMode.REST);
                driveAuto.drive(DriveAuto.DriveDirection.RIGHT, 65);
                checkStateCompletion();
                break;
            case 10:
                foundationPuller.setServoState(FoundationPuller.ServoState.OPEN);
                intake.setIntakeMode(Intake.IntakeMode.INTAKE);
                driveAuto.drive(DriveAuto.DriveDirection.FORWARD, 40);
                checkStateCompletion();
                break;
            case 11:
                driveAuto.drive(DriveAuto.DriveDirection.LEFT, 65);
                checkStateCompletion();
                break;
            case 12:
                driveAuto.drive(DriveAuto.DriveDirection.FORWARD, 185+skystonePosition*20);
                checkStateCompletion();
                break;
            case 13:
                //foundationPuller.setServoState(FoundationPuller.ServoState.PREPARED);
                gripper.close();
                driveAuto.turn(DriveAuto.TurnDirection.RIGHT, 90);
                checkStateCompletion();
                break;
            case 14:
                lift.liftUp();
                foundationPuller.setServoState(FoundationPuller.ServoState.CLOSED);
                driveAuto.drive(DriveAuto.DriveDirection.FORWARD, 25);
                checkStateCompletion();
                break;
            case 15:
                extension.extend();
                //lift.setLiftState(Lift.LiftState.AIMING);
                driveAuto.drive(DriveAuto.DriveDirection.BACKWARD, 105);
                checkStateCompletion();
                break;
            case 16:
                gripper.open();
                //lift.liftUp();
                driveAuto.drive(DriveAuto.DriveDirection.FORWARD, 5);
                checkStateCompletion();
                break;
            case 17:
                foundationPuller.setServoState(FoundationPuller.ServoState.OPEN);
                extension.retract();
                lift.setLiftState(Lift.LiftState.RETRACTED);
                driveAuto.drive(DriveAuto.DriveDirection.BACKWARD, 5);
                checkStateCompletion();
                break;
        }
         */
    private void runDoubleSkystonesRed()
    {

        switch (currentStage) {
            case 0:
                if (ready) {
                    skystonePosition = skystoneDetector.getPosition();
                    extension.forceRetract();
                    stageCompleted = true;
                    skystonePosition = Math.abs(skystonePosition-2);
                }
                break;
            case 1:
                driveAuto.drive(DriveAuto.DriveDirection.LEFT, 60);
                checkStateCompletion();
                break;
            case 2:
                foundationPuller.setServoState(FoundationPuller.ServoState.AUTO_MOVE);
                if(skystonePosition == 0) {
                    driveAuto.drive(DriveAuto.DriveDirection.FORWARD, 0);
                } else {
                    driveAuto.drive(DriveAuto.DriveDirection.BACKWARD, 20 * skystonePosition - 6);
                }
                checkStateCompletion();
                break;
            case 3:
                driveAuto.turn(DriveAuto.TurnDirection.LEFT, 50);
                checkStateCompletion();
                break;
            case 4:
                foundationPuller.setServoState(FoundationPuller.ServoState.OPEN);
                driveAuto.setMaxDriveSpeed(0.4);
                intake.setIntakeMode(Intake.IntakeMode.INTAKE);
                driveAuto.drive(DriveAuto.DriveDirection.FORWARD, 55);
                checkStateCompletion();
                break;
            case 5:
                driveAuto.setMaxDriveSpeed(1);
                driveAuto.drive(DriveAuto.DriveDirection.BACKWARD, 50);
                checkStateCompletion();
                break;
            case 6:
                driveAuto.turn(DriveAuto.TurnDirection.RIGHT, 50);
                checkStateCompletion();
                break;
            case 7:
                driveAuto.setMaxDriveSpeed(0.7);
                intake.setIntakeMode(Intake.IntakeMode.REST);
                //driveAuto.drive(DriveAuto.DriveDirection.FORWARD, 140 + 20 * skystonePosition);

                if(skystonePosition == 0) {

                    driveAuto.drive(DriveAuto.DriveDirection.FORWARD, 142 + 20 * skystonePosition);
                } else {
                    driveAuto.drive(DriveAuto.DriveDirection.FORWARD, 142 + 20 * skystonePosition);

                }
                checkStateCompletion();
                break;
            case 8:
                intake.setIntakeMode(Intake.IntakeMode.OUTTAKE);
                extension.push();
                gripper.open();
                driveAuto.turn(DriveAuto.TurnDirection.RIGHT, 180);
                checkStateCompletion();
                break;
            case 9:
                foundationPuller.setServoState(FoundationPuller.ServoState.AUTO_MOVE);
                extension.forceRetract();
                intake.setIntakeMode(Intake.IntakeMode.REST);
                intake.resetBlockIntaked();
                lift.setLiftState(Lift.LiftState.RETRACTED);
                driveAuto.drive(DriveAuto.DriveDirection.FORWARD, 126 + 20 * skystonePosition);
                checkStateCompletion();
                break;
            case 10:
                driveAuto.setMaxDriveSpeed(1);
                intake.resetBlockIntaked();
                lift.setLiftState(Lift.LiftState.RETRACTED);
                gripper.open();
                driveAuto.drive(DriveAuto.DriveDirection.RIGHT, 28);
                checkStateCompletion();
                break;
            case 11:
                driveAuto.turn(DriveAuto.TurnDirection.RIGHT, 10);
                checkStateCompletion();
                break;
            case 12:
                driveAuto.setMaxDriveSpeed(0.4);
                intake.setIntakeMode(Intake.IntakeMode.INTAKE);
                foundationPuller.setServoState(FoundationPuller.ServoState.OPEN);
                driveAuto.drive(DriveAuto.DriveDirection.FORWARD, 20);
                checkStateCompletion();
                break;
            case 13:
                driveAuto.setMaxDriveSpeed(1);
                driveAuto.drive(DriveAuto.DriveDirection.BACKWARD, 20);
                checkStateCompletion();
                break;
            case 14:
                driveAuto.turn(DriveAuto.TurnDirection.RIGHT, 80);
                checkStateCompletion();
            case 15:
                driveAuto.drive(DriveAuto.DriveDirection.BACKWARD, 34);
                checkStateCompletion();
                break;
            case 16:
                driveAuto.drive(DriveAuto.DriveDirection.RIGHT, 205+skystonePosition*20);
                checkStateCompletion();
                break;
            case 17:
                intake.setIntakeMode(Intake.IntakeMode.REST);
                lift.liftUp();
                extension.extend();
                foundationPuller.setServoState(FoundationPuller.ServoState.CLOSED);
                driveAuto.drive(DriveAuto.DriveDirection.FORWARD, 30);
                checkStateCompletion();
                break;
            case 18:
                lift.setLiftState(Lift.LiftState.AIMING);
                driveAuto.specialTurn(DriveAuto.TurnDirection.RIGHT);
                checkStateCompletion();
                break;
            case 19:
                gripper.open();
                foundationPuller.setServoState(FoundationPuller.ServoState.OPEN);
                driveAuto.drive(DriveAuto.DriveDirection.FORWARD, 50);
                checkStateCompletion();
                break;
            case 20:
                extension.forceRetract();
                lift.liftUp();
                driveAuto.turn(DriveAuto.TurnDirection.RIGHT, 5);
                checkStateCompletion();
                break;
            case 21:
                //driveAuto.brakeFloat();
                intake.resetBlockIntaked();
                lift.setLiftState(Lift.LiftState.RETRACTED);
                driveAuto.drive(DriveAuto.DriveDirection.BACKWARD, 95);
                checkStateCompletion();
                break;
        }
    }

    private void runDoubleSkystonesBlue()
    {
        switch (currentStage) {
            case 0:
                if (ready) {
                    skystonePosition = skystoneDetector.getPosition();
                    extension.forceRetract();
                    stageCompleted = true;
                }
                break;
            case 1:
                driveAuto.drive(DriveAuto.DriveDirection.LEFT, 63);
                checkStateCompletion();
                break;
            case 2:
                foundationPuller.setServoState(FoundationPuller.ServoState.AUTO_MOVE);
                driveAuto.drive(DriveAuto.DriveDirection.FORWARD, 20 * skystonePosition + 21);
                checkStateCompletion();
                break;
            case 3:
                driveAuto.turn(DriveAuto.TurnDirection.RIGHT, 230);
                checkStateCompletion();
                break;
            case 4:
                foundationPuller.setServoState(FoundationPuller.ServoState.OPEN);
                driveAuto.setMaxDriveSpeed(0.4);
                intake.setIntakeMode(Intake.IntakeMode.INTAKE);
                driveAuto.drive(DriveAuto.DriveDirection.FORWARD, 60);
                checkStateCompletion();
                break;
            case 5:
                driveAuto.setMaxDriveSpeed(1);
                driveAuto.drive(DriveAuto.DriveDirection.BACKWARD, 58);
                checkStateCompletion();
                break;
            case 6:
                driveAuto.turn(DriveAuto.TurnDirection.LEFT, 50);
                checkStateCompletion();
                break;
            case 7:
                driveAuto.setMaxDriveSpeed(0.7);
                intake.setIntakeMode(Intake.IntakeMode.REST);
                driveAuto.drive(DriveAuto.DriveDirection.FORWARD, 140 + 20 * skystonePosition);
                checkStateCompletion();
                break;
            case 8:
                intake.setIntakeMode(Intake.IntakeMode.OUTTAKE);
                extension.push();
                gripper.open();
                driveAuto.turn(DriveAuto.TurnDirection.LEFT, 180);
                checkStateCompletion();
                break;
            case 9:
                foundationPuller.setServoState(FoundationPuller.ServoState.AUTO_MOVE);
                extension.forceRetract();
                intake.setIntakeMode(Intake.IntakeMode.REST);
                intake.resetBlockIntaked();
                lift.setLiftState(Lift.LiftState.RETRACTED);
                driveAuto.drive(DriveAuto.DriveDirection.FORWARD, 112 + 20 * skystonePosition);
                checkStateCompletion();
                break;
            case 10:
                driveAuto.setMaxDriveSpeed(1);
                intake.resetBlockIntaked();
                lift.setLiftState(Lift.LiftState.RETRACTED);
                gripper.open();
                driveAuto.drive(DriveAuto.DriveDirection.LEFT, 32);
                checkStateCompletion();
                break;
            case 11:
                driveAuto.turn(DriveAuto.TurnDirection.LEFT, 10);
                checkStateCompletion();
                break;
            case 12:
                driveAuto.setMaxDriveSpeed(0.6);
                intake.setIntakeMode(Intake.IntakeMode.INTAKE);
                foundationPuller.setServoState(FoundationPuller.ServoState.OPEN);
                driveAuto.drive(DriveAuto.DriveDirection.FORWARD, 25);
                checkStateCompletion();
                break;
            case 13:
                driveAuto.setMaxDriveSpeed(1);
                driveAuto.drive(DriveAuto.DriveDirection.BACKWARD, 25);
                checkStateCompletion();
                break;
            case 14:
                driveAuto.turn(DriveAuto.TurnDirection.LEFT, 80);
                checkStateCompletion();
            case 15:
                driveAuto.drive(DriveAuto.DriveDirection.BACKWARD, 29);
                checkStateCompletion();
                break;
            case 16:
                driveAuto.drive(DriveAuto.DriveDirection.LEFT, 205+skystonePosition*20);
                checkStateCompletion();
                break;
            case 17:
                intake.setIntakeMode(Intake.IntakeMode.REST);
                lift.liftUp();
                extension.extend();
                foundationPuller.setServoState(FoundationPuller.ServoState.CLOSED);
                driveAuto.drive(DriveAuto.DriveDirection.FORWARD, 32);
                checkStateCompletion();
                break;
            case 18:
                lift.setLiftState(Lift.LiftState.AIMING);
                driveAuto.specialTurn(DriveAuto.TurnDirection.LEFT);
                checkStateCompletion();
                break;
            case 19:
                gripper.open();
                foundationPuller.setServoState(FoundationPuller.ServoState.OPEN);
                driveAuto.drive(DriveAuto.DriveDirection.FORWARD, 45);
                checkStateCompletion();
                break;
            case 20:
                lift.liftUp();
                extension.forceRetract();
                driveAuto.turn(DriveAuto.TurnDirection.LEFT, 15);
                checkStateCompletion();
                break;
            case 21:
                intake.resetBlockIntaked();
                lift.setLiftState(Lift.LiftState.RETRACTED);
                driveAuto.drive(DriveAuto.DriveDirection.BACKWARD, 100);
                checkStateCompletion();
                break;
        }
    }

    //the test auto program
    private void runPark()
    {
        switch(currentStage)
        {
            case 0:
                if(ready) {
                    stageCompleted = true;
                }
                break;
            case 1:
                driveAuto.drive(DriveAuto.DriveDirection.FORWARD, 20);
                checkStateCompletion();
                break;
            case 2:
                extension.forceRetract();
                stageCompleted = true;
                break;
        }
    }

    public void setAutoMode(AutoMode autoMode) {
        this.autoMode = autoMode;
        switch(autoMode)
        {
            case TEST:
                initialiseTest();
                break;
            case FOUNDATION_BLUE:
                initialiseFoundationBlue();
                break;
            case FOUNDATION_RED:
                initialiseFoundationRed();
                break;
            case FOUNDATION_RED_CLOSE:
                initialiseFoundationRedClose();
                break;
            case FOUNDATION_BLUE_CLOSE:
                initialiseFoundationBlueClose();
                break;
            case SKYSTONES_BLUE:
                initialiseSkystonesBlue();
                break;
            case SKYSTONES_RED:
                initialiseSkystonesRed();
                break;
            case DOUBLE_SKYSTONES_BLUE:
                initialiseDoubleSkystones();
                break;
            case DOUBLE_SKYSTONES_RED:
                initialiseDoubleSkystones();
                break;
            case PARK:
                initialisePark();
                break;
            default:
                break;
        }
    }

    public void update(double currentRuntime)
    {
        this.currentRuntime = currentRuntime;
        driveAuto.update(this.currentRuntime);
        switch(autoMode)
        {
            case TEST:
                runTest();
                break;
            case FOUNDATION_BLUE:
                runFoundationBlue();
                break;
            case FOUNDATION_RED:
                runFoundationRed();
                break;
            case FOUNDATION_RED_CLOSE:
                runFoundationRedClose();
                break;
            case FOUNDATION_BLUE_CLOSE:
                runFoundationBlueClose();
                break;
            case SKYSTONES_BLUE:
                gripper.update(currentRuntime);
                intake.update(currentRuntime);
                lift.update(0, 1,extension.getExtensionState() == Extension.ExtensionState.RETRACTED, gripper.getClawMode() == Gripper.ClawMode.CLOSED);
                extension.update(currentRuntime, gripper.getClawMode() == Gripper.ClawMode.OPENED, true);
                runSkystonesBlue();
                break;
            case SKYSTONES_RED:
                gripper.update(currentRuntime);
                intake.update(currentRuntime);
                lift.update(0, 1,extension.getExtensionState() == Extension.ExtensionState.RETRACTED, gripper.getClawMode() == Gripper.ClawMode.CLOSED);
                extension.update(currentRuntime, gripper.getClawMode() == Gripper.ClawMode.OPENED, true);
                runSkystonesRed();
                break;
            case DOUBLE_SKYSTONES_BLUE:
                gripper.update(currentRuntime);
                intake.update(currentRuntime);
                if(intake.getBlockIntaked(gripper.getClawMode() == Gripper.ClawMode.OPENED))
                {
                    intake.resetBlockIntaked();
                    intake.setIntakeMode(Intake.IntakeMode.REST);
                    gripper.close();
                }
                lift.update(0, 1, extension.getExtensionState() == Extension.ExtensionState.RETRACTED, gripper.getClawMode() == Gripper.ClawMode.CLOSED);
                extension.update(currentRuntime, gripper.getClawMode() == Gripper.ClawMode.OPENED, true);
                runDoubleSkystonesBlue();
                break;
            case DOUBLE_SKYSTONES_RED:
                gripper.update(currentRuntime);
                intake.update(currentRuntime);
                if(intake.getBlockIntaked(gripper.getClawMode() == Gripper.ClawMode.OPENED))
                {
                intake.resetBlockIntaked();
                intake.setIntakeMode(Intake.IntakeMode.REST);
                gripper.close();
                }
                lift.update(0, 1, extension.getExtensionState() == Extension.ExtensionState.RETRACTED, gripper.getClawMode() == Gripper.ClawMode.CLOSED);
                extension.update(currentRuntime, gripper.getClawMode() == Gripper.ClawMode.OPENED, true);
                runDoubleSkystonesRed();
                break;
            case PARK:
                runPark();
                break;
            default:
                break;
        }
        if(stageCompleted && currentStage < maxStages + 1)
        {
            currentStage += 1;
            stageCompleted = false;
        }
    }

    public void start()
    {
        ready = true;
    }

    public boolean getRunning()
    {
        return(currentStage < maxStages + 1);
    }

    public int getCurrentStage()
    {
        return currentStage;
    }
}
