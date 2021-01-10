package org.firstinspires.ftc.teamcode.lib;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.teamcode.drive.opmode.Intake;
import org.firstinspires.ftc.teamcode.drive.opmode.Shooter;
import org.firstinspires.ftc.teamcode.drive.opmode.Transfer;

public class Hardware {

    Intake intake;
    Shooter shooter;
    Transfer transfer;

    public Hardware (HardwareMap hardwareMap, boolean Telop) {
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);
        transfer = new Transfer(hardwareMap);
    }

    private void updateShooter(/*tuple*/) {
        //
    }

    public void update() {
        updateShooter(/*Befehle für den Shooter als Tuple zeugs*/);
    }


}
