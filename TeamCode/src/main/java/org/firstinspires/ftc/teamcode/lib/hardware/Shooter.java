package org.firstinspires.ftc.teamcode.lib.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.lib.datatypes.INTuple;
import org.firstinspires.ftc.teamcode.lib.datatypes.Twople;

import java.util.ArrayList;

public class Shooter {

    HardwareMap hardwareMap;
    Gamepad gamepad2;
    DcMotor

    public Shooter (HardwareMap hardwareMap, Gamepad gamepad2) {
        this.hardwareMap = hardwareMap;
        this.gamepad2 = gamepad2;

    }

    public ArrayList<Twople> update(ArrayList<INTuple> instructions) {
        ArrayList<Twople> output = new ArrayList<>();

        return output;
    }
}
