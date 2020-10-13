package org.firstinspires.ftc.teamcode.lib;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class Shooting {
    private Globals g = new Globals();
    public Targets currentTarget;
    private String alliance;

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

    private void adjustTheta() {

    }

    private Boolean motorsReady() {
        return true;
    }

    public Shooting(String alliance) {
        if (!(alliance == "Red")) {
            alliance = "blue";
        }
        if (!(alliance == "Blue")) {
            alliance = "red";
        }
        currentTarget = Targets.blueUpper;
    }

    public void shootAllStatic(HardwareMap hardwareMap, Pose2d position) {
        DcMotor motor1 = hardwareMap.get(DcMotorEx.class, "motor1");
        DcMotor motor2 = hardwareMap.get(DcMotorEx.class, "motor2");
        Servo servo = hardwareMap.get(Servo.class, "shooter");
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Vector currentPos = new Vector(position.component1(), position.component2());
        double expectedRotation = currentTarget.getCoordinates().subtract(currentPos).angle();
        Pose2d targetPose = new Pose2d(currentPos.x, currentPos.y, Math.toRadians(position.component3() - expectedRotation));
        Trajectory turn = drive.trajectoryBuilder(new Pose2d()).lineToSplineHeading(targetPose).build();

        drive.followTrajectory(turn);

        for (int i = 0; i < 3; i++) {
            adjustTheta();
            while (!motorsReady()) {
            }
            servo.setPosition(0);
            servo.setPosition(1);
        }
        motor1.setPower(0);
        motor2.setPower(0);
    }
}

