package org.firstinspires.ftc.teamcode.newLib;

import org.firstinspires.ftc.teamcode.newLib.hardware.Intake;
import org.firstinspires.ftc.teamcode.newLib.hardware.Shooter;
import org.firstinspires.ftc.teamcode.newLib.hardware.Transfer;

public class Robot {
    Shooter shooter;
    Intake intake;
    Transfer transfer;

    Shooter.Mode expectedShooterMode;
    Intake.Mode expectedIntakeMode;
    Transfer.Mode expectedTransferMode = Transfer.Mode.NORMAL;

    Shooter.Mode currentShooterMode;
    Intake.Mode currentIntakeMode;
    Transfer.Mode currentTransferMode;



    public Robot() {
        shooter = new Shooter();
        intake = new Intake();
        transfer = new Transfer();
    }

    private void

    private void goalCentric() {
        shooter.setMode(Shooter.Mode.SHOOTING);
        intake.setMode(Intake.Mode.IDLE);

        expectedShooterMode = Shooter.Mode.SHOOTING;
        expectedIntakeMode = Intake.Mode.IDLE;
    }

    private void robtoCentric() {
        shooter.setMode(Shooter.Mode.IDLE);
        intake.setMode(Intake.Mode.NORMAL);

        expectedShooterMode = Shooter.Mode.IDLE;
        expectedIntakeMode = Intake.Mode.NORMAL;
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
            case MODIFIED_GOAL_CENTRIC:
                break;
            case MODIFIED_ROBOT_CENTRIC:
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

            }
        }

        //check if current mode is modified
        if( currentIntakeMode == expectedIntakeMode &&
            currentShooterMode == expectedShooterMode &&
            currentTransferMode == expectedTransferMode)
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
}
