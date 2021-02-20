package org.firstinspires.ftc.teamcode.newLib;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.newLib.hardware.Intake;
import org.firstinspires.ftc.teamcode.newLib.hardware.Shooter;
import org.firstinspires.ftc.teamcode.newLib.hardware.Transfer;

public class Robot {
    Shooter shooter;
    Intake intake;
    Transfer transfer;
    AlignToPoint alignToPoint;

    Mode expectedShooterMode;
    Mode expectedIntakeMode;
    Mode expectedTransferMode = Mode.RUNNING;

    Telemetry telemetry;

    public enum Mode {
        IDLE,
        RUNNING,
        REVERSE;
    }


    public Robot() {
        shooter = new Shooter();
        intake = new Intake();
        transfer = new Transfer();
        alignToPoint = new AlignToPoint();

        telemetry = Comms.telemetry;
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
        alignToPoint.setCurrentMode(AlignToPoint.Mode.ALIGN_TO_POINT);

        expectedShooterMode = Mode.RUNNING;
        expectedIntakeMode = Mode.IDLE;
    }

    private void robotCentric() {
        shooter.setMode(Mode.IDLE);
        intake.setMode(Mode.RUNNING);
        alignToPoint.setCurrentMode(AlignToPoint.Mode.NORMAL_CONTROL);

        expectedShooterMode = Mode.IDLE;
        expectedIntakeMode = Mode.RUNNING;
    }

    private void checkModified() {
        setModified();
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
        }
        if (transfer.getMode() == Mode.REVERSE) {
            intake.setMode(Mode.REVERSE);
        }
    }


    public void update(double currentRuntime) {
        //update shooter runtime
        shooter.update(currentRuntime);
        alignToPoint.update();

        //make sure every part's state is correct
        switch (Comms.driveMode) {
            case GOAL_CENTRIC:
                goalCentric();
                break;
            case ROBOT_CENTRIC:
                robotCentric();
                break;
            default:
                break;
        }

        //execute instructions
        for (int i = 0; i < Comms.tasks.size(); i++) {
            switch(Comms.tasks.get(i)) {
                case SET_GOAL_CENTRIC:
                    goalCentric();
                    Comms.driveMode = Comms.DriveMode.GOAL_CENTRIC;
                    break;
                case SET_ROBOT_CENTRIC:
                    robotCentric();
                    Comms.driveMode = Comms.DriveMode.ROBOT_CENTRIC;
                    break;
                case SHOOT:
                    shooter.shoot();
                    break;
                case REVERSE_INTAKE:
                    intake.setMode(Mode.REVERSE);
                    break;
                case REVERSE_TRANSFER:
                    intake.setMode(Mode.REVERSE);
                    transfer.setMode(Mode.REVERSE);
                    break;
                case RESET_INTAKE:
                    intake.setMode(intake.getLastMode());
                    break;
                case RESET_TRANSFER:
                    transfer.setMode(transfer.getLastMode());
                    break;
                case DISABLE_INTAKE:
                    intake.setMode(Mode.IDLE);
                    break;
                case DISABLE_TRANSFER:
                    transfer.setMode(Mode.IDLE);
                    break;
                case DISABLE_SHOOTER:
                    shooter.setMode(Mode.IDLE);
                    break;
                case TOGGLE_INTAKE:
                    switch (intake.getMode()) {
                        case IDLE:
                            intake.setMode(Mode.RUNNING);
                            break;
                        default:
                            intake.setMode(Mode.IDLE);
                            break;
                    }
                    break;
                case TOGGLE_TRANSFER:
                    switch (transfer.getMode()) {
                        case IDLE:
                            transfer.setMode(Mode.RUNNING);
                            break;
                        default:
                            transfer.setMode(Mode.IDLE);
                            break;
                    }
                    break;
                case TOGGLE_SHOOTER:
                    switch (shooter.getMode()) {
                        case IDLE:
                            shooter.setMode(Mode.RUNNING);
                            break;
                        default:
                            shooter.setMode(Mode.IDLE);
                            break;
                    }
                default:
                    break;
            }
        }

        //check Incompatible hardware modes
        checkHardware();

        //check if current modes are modified
        checkModified();

        telemetry.addData("Target", Targets.currentTargetNum);
        telemetry.addData("Target", Targets.currentTargetName);
        telemetry.addData("transfer", transfer.getLastMode());
        telemetry.addData("transferCurrent", transfer.getMode());
        telemetry.addData("intake", intake.getLastMode());
        telemetry.addData("intakeCurrent", intake.getMode());
        telemetry.addData("transferCalls", Comms.TEMP);
        telemetry.update();
    }
}

