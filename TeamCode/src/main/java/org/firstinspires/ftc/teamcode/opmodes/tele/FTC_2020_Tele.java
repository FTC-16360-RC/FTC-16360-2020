package org.firstinspires.ftc.teamcode.opmodes.tele;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.apache.commons.math3.util.OpenIntToDoubleHashMap;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.lib.Keybindings;
import org.firstinspires.ftc.teamcode.lib.Odometry;
import org.firstinspires.ftc.teamcode.lib.Shooter;
import org.firstinspires.ftc.teamcode.lib.datatypes.INTuple;
import org.firstinspires.ftc.teamcode.lib.datatypes.Twople;
import org.firstinspires.ftc.teamcode.lib.datatypes.Vector;


@TeleOp(name="FTC 2020 Tele", group="Iterative Opmode")
//@Disabled
public class FTC_2020_Tele extends OpMode {


    private Shooter shooter = new Shooter(hardwareMap);
    private Keybindings keys = new Keybindings(gamepad1);
    private Odometry odometry = new Odometry();
    Vector currentPosition = new Vector();
    StandardTrackingWheelLocalizer myLocalizer = new StandardTrackingWheelLocalizer(hardwareMap);
    Twople[] comms = {};
    Twople[] lastComms = {};
    INTuple[] globalData = {};

    private void append(Twople[] input) {
        for (int i = 0; i < input.length; i++) {
            comms[comms.length + 1] = input[i];
        }
    }

    private INTuple[] returnComs(String name) {
        INTuple[] output = {};
        for (int i = 0; i < lastComms.length; i++) {
            if (lastComms[i].a == name) {
                output[output.length + 1] = lastComms[i].b;
            }
        }
        return output;
    }

    @Override
    public void init() {
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        //drivemode.startTracking();
        myLocalizer.setPoseEstimate(new Pose2d(10, 10, Math.toRadians(90)));
    }

    @Override
    public void loop() {
        //clear comms
        for (int i = 0; i < comms.length; i++) {
            comms[i] = null;
        }

        //every update function returns an Twople[] (String recipient, (String instruction, (JSONArray data)))
        //append appends every element in this Twople[] to comms. Comms is also Twople[]
        //retrunComs returns a INTuple[], an array of (String instruction, (JSONArray data))
        myLocalizer.update();
        append(shooter.update(returnComs("shooter"), currentPosition));
        append(keys.update(returnComs("keys")));
        //append(odometry.update(returnComs("odometry"), currentPosition));

        //lastComms is the comms that is used to give back to the classes. Comms picks up information.
        lastComms = comms;
    }
}
