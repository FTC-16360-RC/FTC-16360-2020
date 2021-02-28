package org.firstinspires.ftc.teamcode.lib;

import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.arcrobotics.ftclib.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.List;

public class Comms {

    public enum Team {
        RED,
        BLU
    }

    public enum Tasks {
        SHOOT,
        REVERSE_INTAKE,
        REVERSE_TRANSFER,
        TOGGLE_INTAKE,
        TOGGLE_TRANSFER,
        TOGGLE_SHOOTER,
        RESET_INTAKE,
        RESET_TRANSFER,
        DISABLE_TRANSFER,
        DISABLE_INTAKE,
        DISABLE_SHOOTER,
        SET_ROBOT_CENTRIC,
        SET_GOAL_CENTRIC;
    }

    public enum DriveMode {
        GOAL_CENTRIC,
        ROBOT_CENTRIC,
        MODIFIED_GOAL_CENTRIC,
        MODIFIED_ROBOT_CENTRIC;
    }

    public static Pose2d position;

    public static Team team;
    public static Gamepad gamepad1;
    public static Gamepad gamepad2;
    public static Telemetry telemetry;
    public static HardwareMap hardwareMap;

    public static DriveMode driveMode;
    public static List<Tasks> tasks;

    public static int TEMP = 0;

    public static void reset() {
        tasks = new ArrayList<>();
        driveMode  = DriveMode.ROBOT_CENTRIC;
    }
}