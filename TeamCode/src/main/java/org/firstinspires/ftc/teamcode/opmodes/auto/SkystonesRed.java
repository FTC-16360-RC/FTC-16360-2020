package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.control.IMU;
import org.firstinspires.ftc.teamcode.lib.AutoControl;

//@Disabled
@Autonomous(name="Skystone Red", group="Iterative Opmode")
public class SkystonesRed extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private AutoControl autoControl;
    private IMU imu;

    @Override
    public void init() {
        autoControl = new AutoControl(hardwareMap, true);
        imu = new IMU(hardwareMap);
        telemetry.addData("Status", "Initialized");
        autoControl.setAutoMode(AutoControl.AutoMode.SKYSTONES_RED);
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
        runtime.reset();
        autoControl.start();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        if(autoControl.getRunning()) {
            autoControl.update(getRuntime());
        }
        imu.update();
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Stage", Integer.valueOf(autoControl.getCurrentStage()).toString());
        telemetry.addData("Heading", Double.valueOf(imu.getHeading()).toString());
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
