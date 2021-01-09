package org.firstinspires.ftc.teamcode.lib;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lib.AlignToPoint;
import org.firstinspires.ftc.teamcode.lib.G;
import org.firstinspires.ftc.teamcode.lib.Keybindings;
import org.firstinspires.ftc.teamcode.lib.datatypes.TUtil;
import org.firstinspires.ftc.teamcode.lib.datatypes.UTuple;
import org.firstinspires.ftc.teamcode.lib.datatypes.util.Adresses;
import org.firstinspires.ftc.teamcode.lib.hardware.Intake;
import org.firstinspires.ftc.teamcode.lib.hardware.Shooter;

import java.util.ArrayList;

public class Robot{

    private Keybindings keybindings;
    private AlignToPoint alignToPoint;
    private Shooter shooter;
    private Intake intake;

    TUtil comms;
    TUtil lastComms;

    Telemetry telemetry;
    HardwareMap hardwareMap;
    Gamepad gamepad1;
    Gamepad gamepad2;

    public Robot(Telemetry telemetry, HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2) {
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
    }

    private void append(TUtil input) {
        for (int i = 0; i < input.size(); i++) {
            comms.add(input.get(i));
        }
    }

    private TUtil returnComs(Adresses name) {
        TUtil output = new TUtil();
        for (int i = 0; i < lastComms.size(); i++) {
            if (lastComms.get(i).a_adr == name) {
                output.add(lastComms.get(i).b_utp);
            }
        }
        return output;
    }

    public void init() {
        keybindings = new Keybindings(gamepad1, gamepad2, telemetry);
        alignToPoint = new AlignToPoint(hardwareMap, telemetry, gamepad1, gamepad2);
        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap, telemetry);

        comms = new TUtil();
        lastComms = new TUtil();
    }

    public void init_loop() {
    }

    public void start() {
        intake.lowerIntake();
    }

    public void loop() {
        //clear comms
        comms.clear();


        append(keybindings.update(returnComs(Adresses.KEYBINDINGS)));
        append(alignToPoint.update(returnComs(Adresses.ALIGN_TO_POINT)));
        append(shooter.update(returnComs(Adresses.SHOOTER)));

        //lastComms is the comms that is used to give back to the classes. Comms picks up information.
        lastComms = comms;
    }
}
