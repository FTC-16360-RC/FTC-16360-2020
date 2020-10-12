package org.firstinspires.ftc.teamcode.control;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {

    /*
    //  DECLARATION
    */

    //declare the intake motors
    private DcMotor intake_left;
    private DcMotor intake_right;

    //declare the sensor that looks for blocks
    private ColorSensor color_sensor = null;

    //power control variables for the motors
    private double intakeRightPower = 0;
    private double intakeLeftPower = 0;

    //constants dependant on motor type
    private static final double gearRatio = 3.7;     //gear ratio of intake motors
    private static final double rawEncoderPPR = 28;  //pulses per revolution of encoder, not output shaft
    private static final double rawMaxRPM = 5994;    //the maximum RPM the motor can turn without gearbox

    //constants depending on current settings
    private static final double intakePower = 0.45; //power setting used to outtake the stones       0.45
    private static final double outtakePower = -0.35;//power setting used to outtake the stones
    private final double correctionTime = 0.5;      //time during which the intake is powered differently to allow the stone to align //0.2
    private final double correctionPower = 1;       //power at which the intake is powered to align the stone
    private final double armingModifier = 0.7;      //ratio of top speed at which the auto-align is armed
    private final double triggerModifier = 0.6;     //ratio of top speed at which the auto-align is triggered

    //constants derived from other constants
    private final double armingRPM = rawMaxRPM / gearRatio * armingModifier * intakePower;//200     //minimum RPM needed to arm auto-align
    private final double triggerRPM = rawMaxRPM / gearRatio * triggerModifier * intakePower;//30   //maximum RPM needed to trigger auto-align

    //variables depending on current measurements
    private int currentRightPosition = 0;   //encoder reading of right intake motor in current passage
    private int currentLeftPosition = 0;    //encoder reading of left intake motor in current passage
    private double currentRuntime = 0;      //total runtime at current measurement

    //variables derived of measured variables
    private int lastRightPosition = 0;      //encoder reading of right intake motor in last passage
    private int lastLeftPosition = 0;       //encoder reading of left intake motor in last passage
    private double lastRuntime = 0;         //total runtime at last measurement
    private double rightRPM = 0;            //encoder count per second of right intake motor
    private double leftRPM = 0;             //encoder count per second of left intake motor
    private double loopTime = 0;            //time taken for an entire passage of the loop from measurement to measurement

    //variables used for if-statement control
    private boolean autoAlignArmed = false; //boolean to control if auto-align is armed
    private boolean blockIntaked = false;   //boolean to deactivate intake if block is intaken

    private double timerStart = 0;          //Runtime at start of timer used to check if correction time is completed

    //intake modes
    public enum IntakeMode
    {
        INTAKE,
        OUTTAKE,
        REST
    }

    //variables used for switches
    private IntakeMode intakeMode = IntakeMode.REST;    //variable used to stock the intake mode

    /*
    //  METHODS
    */

    //constructor
    public Intake(HardwareMap hardwareMap)
    {
        //configure intake motors
        intake_right = hardwareMap.get(DcMotor.class, "intake right");
        intake_left = hardwareMap.get(DcMotor.class, "intake left");
        color_sensor = hardwareMap.get(ColorSensor.class, "color sensor");

        intake_right.setDirection(DcMotor.Direction.REVERSE);
        intake_left.setDirection(DcMotor.Direction.FORWARD);

        intake_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        intake_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setIntakeMode(IntakeMode intakeMode)
    {
        this.intakeMode = intakeMode;
        if(intakeMode == IntakeMode.INTAKE)
        {
            //reset Encoders to prevent encoder values from being negative when intaking
            intake_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            intake_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            intake_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            intake_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    //update the time variables for this passage
    private void updateTime(double currentRuntime)
    {
        lastRuntime = this.currentRuntime;
        this.currentRuntime = currentRuntime;
        loopTime = this.currentRuntime - lastRuntime;
    }

    //execute measurements and calculations directly after time measurement
    private void  updateEncoderMeasurements()
    {
        lastRightPosition = currentRightPosition;
        lastLeftPosition = currentLeftPosition;
        currentRightPosition = Math.abs(intake_right.getCurrentPosition());
        currentLeftPosition = Math.abs(intake_left.getCurrentPosition());
        rightRPM = (currentRightPosition - lastRightPosition) / loopTime * 60 / (gearRatio*rawEncoderPPR);
        leftRPM = (currentLeftPosition - lastLeftPosition) / loopTime * 60 / (gearRatio*rawEncoderPPR);
    }

    //arms auto-Align if RPM is high enough
    private void armAutoAlign()
    {
        if((rightRPM > armingRPM) && (leftRPM > armingRPM))
        {
            autoAlignArmed = true;
        }
    }

    //checks if auto-align should be executed
    private boolean checkAutoAlign() {
        return ((((rightRPM < triggerRPM) || (leftRPM < triggerRPM)) && autoAlignArmed) && (currentRuntime > (timerStart + correctionTime)));
    }

    //global method to update the intake
    public void update(double currentRuntime)
    {
        switch (intakeMode) {               //the switch differentiates between the different intake modes
            case REST:
                intakeRightPower = 0;
                intakeLeftPower = 0;
                break;
            case OUTTAKE:
                intakeRightPower = outtakePower;
                intakeLeftPower = outtakePower;
                break;
            case INTAKE:
                updateTime(currentRuntime);
                updateEncoderMeasurements();
                armAutoAlign();
                if(checkAutoAlign())
                {
                    timerStart = this.currentRuntime;
                    autoAlignArmed = false;
                }
                if(this.currentRuntime < timerStart + correctionTime)
                {
                    intakeRightPower = correctionPower;
                    intakeLeftPower = correctionPower;
                } else {
                    intakeRightPower = intakePower;
                    intakeLeftPower = intakePower;
                }
                if(color_sensor.alpha() > 500)
                {
                    blockIntaked = true;
                }
                break;
            default:
                break;

        }
        intake_right.setPower(intakeRightPower);
        intake_left.setPower(intakeLeftPower);
    }

    public boolean getBlockIntaked(boolean gripperOpened) {
        if(gripperOpened) {
            return blockIntaked;
        } else {
            return false;
        }
    }

    public void resetBlockIntaked() {
        blockIntaked = false;
    }
    public IntakeMode getIntakeMode() {
        return intakeMode;
    }
}
