package org.firstinspires.ftc.teamcode.newLib;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;

public class Comms {

    enum Team {
        RED,
        BLU
    }

    enum Tasks {
        SHOOT,
        REVERSE_INTAKE,
        REVERSE_TRANSFER,
        TOGGLE_INTAKE,
        TOGGLE_TRANSFER,
        RESET_INTAKE,
        RESET_TRANSFER;
    }

    enum DriveMode {
        GOAL_CENTRIC,
        ROBOT_CENTRIC,
        MODIFIED_GOAL_CENTRIC,
        MODIFIED_ROBOT_CENTRIC;
    }

    public static Team team;
    public static DriveMode driveMode;
    public static Gamepad gamepad1;
    public static Gamepad gamepad2;
    public static Telemetry telemetry;
    public static HardwareMap hardwareMap;

    public static List<Tasks> tasks;

}
