package org.firstinspires.ftc.teamcode.lib.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lib.Globals;
import org.firstinspires.ftc.teamcode.lib.datatypes.INTuple;
import org.firstinspires.ftc.teamcode.lib.datatypes.Twople;

import java.util.ArrayList;

public class Drivetrain {

    Globals Globals = new Globals();
    MecanumControl motors;
    HardwareMap hardwareMap;
    Telemetry telemetry;

    public Drivetrain(HardwareMap h, Telemetry t) {
        hardwareMap = h;
        telemetry = t;
        motors = new MecanumControl(hardwareMap, DcMotor.RunMode.RUN_WITHOUT_ENCODER, DcMotor.ZeroPowerBehavior.FLOAT, t);
    }

    //alpha counterclockwise from front
    public void setSpeedAngleVelocity(double v, double alpha) {
        /*double fl = v *;
        double fr = v *;
        double rl = v *;
        double rr = v *; */

    }
    public void setSpeedXY(double x, double y) {
        double fl = x - y;
        double fr = x + y;
        double rl = x + y;
        double rr = x - y;
        motors.setPower(fl, fr, rl, rr);
    }

    public void setSpeed(double x, double y, double a) {
        motors.setPower(x,y,a,0.5, 0.5);
    }

    public ArrayList<Twople> update(ArrayList<INTuple> instructions) {
        ArrayList<Twople> output = new ArrayList<Twople>();
        for (INTuple i: instructions)
        {
            switch (i.a) {
                case "setSpeed":
                    setSpeed(i.b_array[0], i.b_array[1], i.b_array[2]);
                    //setSpeedXY(i.b_array[0], i.b_array[1]);
                    telemetry.addData("0", i.b_array[0]);
                    telemetry.addData("1", i.b_array[1]);
                    break;
                default:
                    break;
            }
        }
        return output;
    }
}
