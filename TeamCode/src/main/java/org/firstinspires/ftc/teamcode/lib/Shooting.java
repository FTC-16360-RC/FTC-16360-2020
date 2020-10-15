package org.firstinspires.ftc.teamcode.lib;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.opmodes.tele.FTC_2020_Tele;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class Shooting {
    public Targets currentTarget;
    private String alliance;
    private HardwareMap hardwareMap;

    private DcMotorEx motorOne;
    private DcMotorEx motorTwo;
    private int lastDistance = 0;
    private long lastTime = 0;
    private long timeSinceLast = 0;
    private double distance = 0;
    private double MIN_DISTANCE = 0;
    private boolean motorsReady = false;

    public enum Targets  {
        blueUpper (new Vector(5,-3,3)),
        blueMiddle (new Vector(1,2,3)),
        blueLower (new Vector(1,2,3)),
        bluePsOuter (new Vector(1,2,3)),
        bluePsMiddle (new Vector(1,2,3)),
        bluePsInner (new Vector(1,2,3)),
        redUpper (new Vector(1,2,3)),
        redMiddle (new Vector(1,2,3)),
        redLower (new Vector(1,2,3)),
        redPsOuter (new Vector(1,2,3)),
        redPsMiddle (new Vector(1,2,3)),
        redPsInner (new Vector(1,2,3));

        public Vector coordinates = new Vector();

        public Vector getCoordinates() {
            return coordinates;
        }

        private Targets(Vector coordinates) {
            this.coordinates = coordinates;
        }
    }

    public Shooting (HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        currentTarget = Targets.blueUpper;

        motorOne = hardwareMap.get(DcMotorEx.class, "shooterOne");
        motorTwo = hardwareMap.get(DcMotorEx.class, "shoorerTwo");
    }

    private void adjustTheta() {
    }

    private void motorsReadyLoop() {
        distance = motorOne.getCurrentPosition() + motorTwo.getCurrentPosition() - lastDistance;
        lastDistance = motorOne.getCurrentPosition() + motorTwo.getCurrentPosition();

        timeSinceLast = System.nanoTime() - lastTime;
        lastTime = System.nanoTime();
        if (distance / timeSinceLast > MIN_DISTANCE) {
            motorsReady = true;
        } else {
            motorsReady = false;
        }
    }

    public void shootingLoop() {
        motorsReadyLoop();
    }

    public void shootAllStatic(Pose2d position) {
        Servo servo = hardwareMap.get(Servo.class, "shooter");
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Vector currentPos = new Vector(position.component1(), position.component2());
        double expectedRotation = currentTarget.getCoordinates().subtract(currentPos).angle();
        Pose2d targetPose = new Pose2d(currentPos.x, currentPos.y, Math.toRadians(expectedRotation));
        Trajectory turn = drive.trajectoryBuilder(new Pose2d()).lineToSplineHeading(targetPose).build();

        drive.followTrajectory(turn);

        motorOne.setPower(1);
        motorTwo.setPower(1);

        for (int i = 0; i < 3; i++) {
            adjustTheta();
            while (!motorsReady) {
                motorsReadyLoop();
            }
            servo.setPosition(0);   //Feed Ring
            servo.setPosition(1);
        }
        motorOne.setPower(0);
        motorTwo.setPower(0);
    }
}

