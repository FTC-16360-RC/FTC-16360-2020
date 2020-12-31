package org.firstinspires.ftc.teamcode.opmodes;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.lib.Keybindings;
import org.firstinspires.ftc.teamcode.lib.datatypes.INTuple;
import org.firstinspires.ftc.teamcode.lib.datatypes.Twople;
import org.firstinspires.ftc.teamcode.lib.hardware.Drivetrain;

import java.util.ArrayList;


@TeleOp(name="FTC 2020 Tele", group="Iterative Opmode")
//@Disabled
public class FTC_2020_Tele extends OpMode {

    private Keybindings keys;
    private Drivetrain drivetrain;
    ArrayList<Twople> comms;
    ArrayList<Twople> lastComms;

    private void append(ArrayList<Twople> input) {
        for (int i = 0; i < input.size(); i++) {
            comms.add(input.get(i));
        }
    }



    private ArrayList<INTuple> returnComs(String name) {
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
        comms = new ArrayList<Twople>();
        lastComms = new ArrayList<Twople>();
        drivetrain = new Drivetrain(hardwareMap, telemetry);
        keys = new Keybindings(gamepad1, gamepad2);
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        //myLocalizer.setPoseEstimate(new Pose2d(10, 10, Math.toRadians(90)));
    }

    @Override
    public void loop() {
        //clear comms
        comms.clear();

        //every update function returns an Twople[] (String recipient, (String instruction, (JSONArray data)))
        //append appends every element in this Twople[] to comms. Comms is also Twople[]
        //retrunComs returns a INTuple[], an array of (String instruction, (JSONArray data))

        //append(shooter.update(returnComs("shooter"), currentPosition));
        append(keys.update(returnComs("keys")));
        append(drivetrain.update(returnComs("drivetrain")));
        telemetry.addData("len", comms.size());
        telemetry.log();
        //append(odometry.update(returnComs("odometry"), currentPosition));

        //lastComms is the comms that is used to give back to the classes. Comms picks up information.
        lastComms = comms;
    }
}
