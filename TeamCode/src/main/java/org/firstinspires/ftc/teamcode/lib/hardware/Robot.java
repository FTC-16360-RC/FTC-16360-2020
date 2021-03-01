package org.firstinspires.ftc.teamcode.lib.hardware;

import com.acmerobotics.roadrunner.drive.Drive;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.List;

public class Robot {
    HardwareMap hardwareMap;

    private DcMotorEx shooterMotor1, shooterMotor2, intakeMotor, transferMotor;

    public SampleMecanumDrive drive;
    public Shooter shooter;
    public Intake intake;
    public Transfer transfer;

    public Robot(HardwareMap hardwareMap, boolean autonomous) {
        this.hardwareMap = hardwareMap;

        //initialize hardware classes
        drive = new SampleMecanumDrive(hardwareMap);
        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
        transfer = new Transfer(hardwareMap);


        //turn on bulk reading
        for (LynxModule module : this.hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
    }

    public void update() {

    }
}
