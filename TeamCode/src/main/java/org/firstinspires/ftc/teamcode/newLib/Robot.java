package org.firstinspires.ftc.teamcode.newLib;

import org.firstinspires.ftc.teamcode.newLib.hardware.Intake;
import org.firstinspires.ftc.teamcode.newLib.hardware.Shooter;
import org.firstinspires.ftc.teamcode.newLib.hardware.Transfer;

public class Robot {
    Shooter shooter;
    Intake intake;
    Transfer transfer;

    Mode expectedShooterMode;
    Mode expectedIntakeMode;
    Mode expectedTransferMode = Mode.RUNNING;

    Mode shooterToggleMode = Mode.RUNNING;
    Mode intakeToggleMode = Mode.RUNNING;
    Mode transferToggleMode = Mode.RUNNING;

    public enum Mode {
        IDLE,
        RUNNING,
        REVERSE;
    }


    public Robot() {
        shooter = new Shooter();
        intake = new Intake();
        transfer = new Transfer();
    }

    //sets driveMode to Modified
    private void setModified() {
        if (Comms.driveMode == Comms.DriveMode.ROBOT_CENTRIC) {
            Comms.driveMode = Comms.DriveMode.MODIFIED_ROBOT_CENTRIC;
        }
        if (Comms.driveMode == Comms.DriveMode.GOAL_CENTRIC) {
            Comms.driveMode = Comms.DriveMode.MODIFIED_GOAL_CENTRIC;
        }
    }

    private void goalCentric() {
        shooter.setMode(Mode.RUNNING);
        intake.setMode(Mode.IDLE);

        expectedShooterMode = Mode.RUNNING;
        expectedIntakeMode = Mode.IDLE;
    }

    private void robtoCentric() {
        shooter.setMode(Mode.IDLE);
        intake.setMode(Mode.RUNNING);

        expectedShooterMode = Mode.IDLE;
        expectedIntakeMode = Mode.RUNNING;
    }

    private void checkModified() {
        if( intake.getMode() == expectedIntakeMode &&
            shooter.getMode() == expectedShooterMode &&
            transfer.getMode() == expectedTransferMode)
        {
            switch (Comms.driveMode) {
                case MODIFIED_GOAL_CENTRIC:
                    Comms.driveMode = Comms.DriveMode.GOAL_CENTRIC;
                    break;
                case MODIFIED_ROBOT_CENTRIC:
                    Comms.driveMode = Comms.DriveMode.ROBOT_CENTRIC;
                    break;
            }
        }
    }

    private void checkHardware() {
        if (intake.getMode() == Mode.RUNNING) {
            transfer.setMode(Mode.RUNNING);
            setModified();
        }
        if (transfer.getMode() == Mode.REVERSE) {
            intake.setMode(Mode.REVERSE);
            setModified();
        }
    }


    public void update(double currentRuntime) {
        //update shooter runtime
        shooter.update(currentRuntime);

        //make sure every part's state is correct
        switch (Comms.driveMode) {
            case GOAL_CENTRIC:
                goalCentric();
                break;
            case ROBOT_CENTRIC:
                robtoCentric();
                break;
            default:
                break;
        }

        //execute instructions
        for (int i = 0; i < Comms.tasks.size(); i++) {
            switch(Comms.tasks.get(i)) {
                case SHOOT:
                    if (Comms.driveMode == Comms.DriveMode.GOAL_CENTRIC) {
                        shooter.shoot();
                    }
                    break;
                case REVERSE_INTAKE:
                    setModified();
                    intakeToggleMode = intake.getMode();
                    intake.setMode(Mode.REVERSE);
                    break;
                case REVERSE_TRANSFER:
                    setModified();
                    intakeToggleMode = intake.getMode();
                    transferToggleMode = transfer.getMode();
                    intake.setMode(Mode.REVERSE);
                    transfer.setMode(Mode.REVERSE);
                    break;
                case RESET_INTAKE:
                    setModified();
                    intake.setMode(intakeToggleMode);
                    break;
                case RESET_TRANSFER:
                    setModified();
                    intake.setMode(intakeToggleMode);
                    transfer.setMode(transferToggleMode);
                    break;
                case TOGGLE_INTAKE:
                    setModified();
                    switch (intake.getMode()) {
                        case RUNNING:
                            intakeToggleMode = Mode.RUNNING;
                            intake.setMode(Mode.IDLE);
                            break;
                        case REVERSE:
                            intakeToggleMode = Mode.REVERSE;
                            intake.setMode(Mode.IDLE);
                            break;
                        case IDLE:
                            intake.setMode(intakeToggleMode);
                            break;

                    }
                    break;
                case TOGGLE_TRANSFER:
                    setModified();
                    switch (transfer.getMode()) {
                        case RUNNING:
                            transferToggleMode = Mode.RUNNING;
                            transfer.setMode(Mode.IDLE);
                            break;
                        case REVERSE:
                            transferToggleMode = Mode.REVERSE;
                            transfer.setMode(Mode.IDLE);
                            break;
                        case IDLE:
                            transfer.setMode(transferToggleMode);
                            break;
                    }
                    break;
            }
        }

        //check Incompatible hardware modes
        checkHardware();

        //check if current modes are modified
        checkModified();

    }
}
