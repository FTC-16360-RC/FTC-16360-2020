package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="Shooter")
public class Shooter extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();
    double lastTimeMeasurement = 0;
    double lastEncoderMeasurement = 0;
    DcMotor motor;
    boolean motorOn = false;
    boolean stateChanged = false;
    boolean lastUp = false;
    boolean lastDown = false;
    double power = 1;
    double RPM = 0;
    double encoderMeasurement = 0;
    double tiime = 0;

    @Override
    public void init() {

        motor = hardwareMap.get(DcMotor.class, "motor");

        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status:", "Initialized");    //
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        motor.setPower(0);
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {


        if(gamepad1.a) {
            motorOn = true;
            stateChanged = true;
        }

        if(gamepad1.b) {
            motorOn = false;
            stateChanged = true;
        }

        if(gamepad1.dpad_up && !lastUp) {
            power += 0.05;
        }

        if(gamepad1.dpad_down && !lastDown) {
            power -= 0.05;
        }

        if(motorOn && stateChanged)
        {
            motor.setPower(power);
        } else {
            motor.setPower(0);
        }
        lastUp = gamepad1.dpad_up;
        lastDown = gamepad1.dpad_down;
        tiime = getRuntime();
        if(tiime > lastTimeMeasurement + 5) {
            encoderMeasurement = motor.getCurrentPosition();
            RPM = Math.abs(encoderMeasurement - lastEncoderMeasurement)/28*5/5*60/3.7;
            lastEncoderMeasurement = encoderMeasurement;
            lastTimeMeasurement = tiime;
        }

        telemetry.addData("power:",Double.valueOf(power).toString());
        telemetry.addData("RPM:",Double.valueOf(RPM).toString());
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}

