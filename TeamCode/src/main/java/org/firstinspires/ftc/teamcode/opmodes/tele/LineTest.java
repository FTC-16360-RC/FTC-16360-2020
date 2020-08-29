package org.firstinspires.ftc.teamcode.opmodes.tele;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="line test", group="Iterative Opmode")
@Disabled
public class LineTest extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();

    private ColorSensor line_sensor = null;
    @Override
    public void init() {
        line_sensor = hardwareMap.get(ColorSensor.class, "line sensor");

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        //imu.setTargetHeading(imu.getHeading());
        runtime.reset();
    }

    @Override
    public void loop() {


        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("blue",Double.valueOf(line_sensor.blue()).toString());
        telemetry.addData("red",Double.valueOf(line_sensor.red()).toString());
        telemetry.update();
    }
}
