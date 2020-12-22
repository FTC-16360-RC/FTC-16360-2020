package org.firstinspires.ftc.teamcode.lib;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.lib.datatypes.INTuple;
import org.firstinspires.ftc.teamcode.lib.datatypes.Twople;
import org.firstinspires.ftc.teamcode.lib.datatypes.Vector;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

public class Odometry {
/*
    SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

    public Odometry() {

    }

    public Twople[] update(INTuple[] instructions, Vector position) {
        for (int i = 0; i < instructions.length; i++) {
            //  instructions is an array of INTuples (Instruction, [data, data])
            switch (instructions[i].a) {
                case "turn":
                    turn(instructions[i].get_b()[0],position);
                    break;

                default:
                    break;
            }
        }
    }

    private void turn(double angle, Vector position) {
        Trajectory trajectory = new TrajectoryBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(position.x, position.y, Math.toRadians(90)))
                .build();
        drive.followTrajectory(trajectory);
    }*/
}
