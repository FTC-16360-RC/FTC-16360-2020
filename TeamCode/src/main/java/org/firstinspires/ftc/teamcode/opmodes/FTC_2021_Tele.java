package org.firstinspires.ftc.teamcode.opmodes;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.lib.AlignToPoint;
import org.firstinspires.ftc.teamcode.lib.G;
import org.firstinspires.ftc.teamcode.lib.Keybindings;
import org.firstinspires.ftc.teamcode.lib.datatypes.INTuple;
import org.firstinspires.ftc.teamcode.lib.datatypes.Twople;
import org.firstinspires.ftc.teamcode.lib.hardware.Intake;
import org.firstinspires.ftc.teamcode.lib.hardware.Shooter;

import java.util.ArrayList;


@TeleOp(name="FTC 2020 Tele", group="Iterative Opmode")
//@Disabled
public class FTC_2021_Tele extends OpMode {

    private Keybindings keybindings;
    private AlignToPoint alignToPoint;
    private Shooter shooter;
    private Intake intake;

    ArrayList<Twople> comms;
    ArrayList<Twople> lastComms;

    private void append(ArrayList<Twople> input) {
        for (int i = 0; i < input.size(); i++) {
            comms.add(input.get(i));
        }
    }

    private ArrayList<INTuple> returnComs(G.a name) {
        ArrayList<INTuple> output = new ArrayList<INTuple>();
        for (int i = 0; i < lastComms.size(); i++) {
            if (lastComms.get(i).a == name) {
                output.add(lastComms.get(i).b);
            }
        }
        return output;
    }

    @Override
    public void init() {
        keybindings = new Keybindings(gamepad1, gamepad2);
        alignToPoint = new AlignToPoint(hardwareMap, telemetry, gamepad1, gamepad2);
        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap, telemetry);

        comms = new ArrayList<Twople>();
        lastComms = new ArrayList<Twople>();
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        intake.lowerIntake();
    }

    @Override
    public void loop() {
        //clear comms
        comms.clear();

        ArrayList<INTuple> temp = new ArrayList<>();

        //every update function returns an Twople[] (String recipient, (String instruction, (JSONArray data)))
        //append appends every element in this Twople[] to comms. Comms is also Twople[]
        //retrunComs returns a INTuple[], an array of (String instruction, (JSONArray data))

        append(keybindings.update(returnComs(G.a.KEYBINDINGS)));
        append(alignToPoint.update(returnComs(G.a.ALIGN_TO_POINT)));
        append(shooter.update(returnComs(G.a.SHOOTER)));

        //lastComms is the comms that is used to give back to the classes. Comms picks up information.
        lastComms = comms;
    }
}
