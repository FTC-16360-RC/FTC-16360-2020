package org.firstinspires.ftc.teamcode.opmodes;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.lib.AlignToPoint;
import org.firstinspires.ftc.teamcode.lib.G;
import org.firstinspires.ftc.teamcode.lib.Keybindings;
import org.firstinspires.ftc.teamcode.lib.datatypes.UTuple;
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

    ArrayList<UTuple> comms;
    ArrayList<UTuple> lastComms;

    private void append(ArrayList<UTuple> input) {
        for (int i = 0; i < input.size(); i++) {
            comms.add(input.get(i));
        }
    }

    private ArrayList<UTuple> returnComs(UTuple.A name) {
        ArrayList<UTuple> output = new ArrayList<UTuple>();
        for (int i = 0; i < lastComms.size(); i++) {
            if (lastComms.get(i).a_adr == name) {
                output.add(lastComms.get(i).b_utp);
            }
        }
        return output;
    }

    @Override
    public void init() {
        keybindings = new Keybindings(gamepad1, gamepad2, telemetry);
        alignToPoint = new AlignToPoint(hardwareMap, telemetry, gamepad1, gamepad2);
        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap, telemetry);

        comms = new ArrayList<UTuple>();
        lastComms = new ArrayList<UTuple>();
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

        ArrayList<UTuple> temp = new ArrayList<>();

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
