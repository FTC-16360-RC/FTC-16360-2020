package org.firstinspires.ftc.teamcode.control;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DriveAuto extends MecanumControl {
    private ColorSensor line_sensor;

    private static final double wheelDiameter = 10;   //in cm
    private static final double gearRatio = 19.2;     //gearbox of drive motors
    private static final double rawEncoderPPR = 28;   //pulses per revolution of encoder, not output shaft
    private static final double wheelDistance = 73;   //not measured!!! tested

    private static final double EPC = rawEncoderPPR*gearRatio/(wheelDiameter*Math.PI); //encoder counts per cm of driving

    private static double maxDriveSpeed = 1;
    private static final double minDriveSpeed = 0.2;
    private static final double minStrafeSpeed = 0.25;
    private static double maxTurnSpeed = 1;
    private static final double minTurnSpeed = 0.25;
    private double driveSpeedCoefficient = 1;
    private double turnSpeedCoefficient = 1;
    private boolean braking = false;


    private static final double weirdForwardDistanceCoefficient = 0.9;
    private static final double weirdSidewaysDistanceCoefficient = 1.06;

    private static final double specialTurnRadius = 60;
    private static final double specialTurnRatio = (specialTurnRadius-120)/(specialTurnRadius+120);

    private boolean completed = false;
    private boolean driveOrdered = false;

    private boolean useIMU = true;

    private double timerStart;

    public enum DriveDirection
    {
        FORWARD,
        BACKWARD,
        LEFT,
        RIGHT
    }

    private DriveDirection driveDirection;

    public enum TurnDirection {
        LEFT,
        RIGHT
    }

    private TurnDirection turnDirection;

    private enum DriveState
    {
        DRIVING,
        TURNING,
        SPECIAL_TURNING,
        ACCELERATE,
        BRAKE,
        RESTING
    }

    private DriveState driveState = DriveState.RESTING;

    private double distance;
    //private int isRunning;

    private double absoluteHeading;

    private IMU imu;


    //matches superclass constructor
    public DriveAuto(HardwareMap hardwareMap, DcMotor.RunMode runMode, DcMotor.ZeroPowerBehavior brakeMode) {
        super(hardwareMap, runMode, brakeMode);
        imu = new IMU(hardwareMap);
        imu.setDrivePID(0.1, 0, 0.05);
        imu.setTurnKP(0.03);
        line_sensor = hardwareMap.get(ColorSensor.class, "line sensor");
        absoluteHeading = 180;
    }

    public void drive(DriveDirection driveDirection, double distance)
    {
        if(!driveOrdered) {
            this.driveDirection = driveDirection;
            this.distance = distance;
            switch (driveDirection) {
                case FORWARD:
                    front_left_power = maxDriveSpeed;
                    front_right_power = maxDriveSpeed;
                    rear_left_power = maxDriveSpeed;
                    rear_right_power = maxDriveSpeed;
                    this.distance *= weirdForwardDistanceCoefficient;
                    break;
                case BACKWARD:
                    front_left_power = -maxDriveSpeed;
                    front_right_power = -maxDriveSpeed;
                    rear_left_power = -maxDriveSpeed;
                    rear_right_power = -maxDriveSpeed;
                    this.distance *= weirdForwardDistanceCoefficient;
                    break;
                case RIGHT:
                    front_left_power = maxDriveSpeed;
                    front_right_power = -maxDriveSpeed;
                    rear_left_power = -maxDriveSpeed;
                    rear_right_power = maxDriveSpeed;
                    this.distance *= weirdSidewaysDistanceCoefficient;
                    break;
                case LEFT:
                    front_left_power = -maxDriveSpeed;
                    front_right_power = maxDriveSpeed;
                    rear_left_power = maxDriveSpeed;
                    rear_right_power = -maxDriveSpeed;
                    this.distance *= weirdSidewaysDistanceCoefficient;
                    break;
                default:
                    break;
            }

            // reset encoders
            front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            front_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            front_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            rear_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rear_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            rear_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rear_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            // wait for end of execution
            //isRunning = 0b1111;
            imu.setTargetHeading(absoluteHeading);
            completed = false;
            driveOrdered = true;
            driveState = DriveState.DRIVING;
            driveSpeedCoefficient = 0.1+minDriveSpeed/maxDriveSpeed;
            braking = false;
        }
    }

    public void searchForLine(DriveDirection driveDirection, double distance)
    {
        if(!driveOrdered) {
            this.distance = distance;
            this.driveDirection = driveDirection;
            switch (driveDirection) {
                case FORWARD:
                    front_left_power = maxDriveSpeed;
                    front_right_power = maxDriveSpeed;
                    rear_left_power = maxDriveSpeed;
                    rear_right_power = maxDriveSpeed;
                    this.distance *= weirdForwardDistanceCoefficient;
                    break;
                case BACKWARD:
                    front_left_power = -maxDriveSpeed;
                    front_right_power = -maxDriveSpeed;
                    rear_left_power = -maxDriveSpeed;
                    rear_right_power = -maxDriveSpeed;
                    this.distance *= weirdForwardDistanceCoefficient;
                    break;
                case RIGHT:
                    front_left_power = maxDriveSpeed;
                    front_right_power = -maxDriveSpeed;
                    rear_left_power = -maxDriveSpeed;
                    rear_right_power = maxDriveSpeed;
                    this.distance *= weirdSidewaysDistanceCoefficient;
                    break;
                case LEFT:
                    front_left_power = -maxDriveSpeed;
                    front_right_power = maxDriveSpeed;
                    rear_left_power = maxDriveSpeed;
                    rear_right_power = -maxDriveSpeed;
                    this.distance *= weirdSidewaysDistanceCoefficient;
                    break;
                default:
                    break;
            }

            // reset encoders
            front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            front_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            front_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            rear_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rear_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            rear_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rear_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            // wait for end of execution
            //isRunning = 0b1111;
            completed = false;
            driveOrdered = true;
            useIMU = false;
            driveState = DriveState.ACCELERATE;
            driveSpeedCoefficient /= maxDriveSpeed;
        }
    }

    public void accelerate(DriveDirection driveDirection, double distance)
    {
        if(!driveOrdered) {
            useIMU = true;
            this.distance = distance;
            this.driveDirection = driveDirection;
            switch (driveDirection) {
                case FORWARD:
                    front_left_power = maxDriveSpeed;
                    front_right_power = maxDriveSpeed;
                    rear_left_power = maxDriveSpeed;
                    rear_right_power = maxDriveSpeed;
                    this.distance *= weirdForwardDistanceCoefficient;
                    break;
                case BACKWARD:
                    front_left_power = -maxDriveSpeed;
                    front_right_power = -maxDriveSpeed;
                    rear_left_power = -maxDriveSpeed;
                    rear_right_power = -maxDriveSpeed;
                    this.distance *= weirdForwardDistanceCoefficient;
                    break;
                case RIGHT:
                    front_left_power = maxDriveSpeed;
                    front_right_power = -maxDriveSpeed;
                    rear_left_power = -maxDriveSpeed;
                    rear_right_power = maxDriveSpeed;
                    this.distance *= weirdSidewaysDistanceCoefficient;
                    break;
                case LEFT:
                    front_left_power = -maxDriveSpeed;
                    front_right_power = maxDriveSpeed;
                    rear_left_power = maxDriveSpeed;
                    rear_right_power = -maxDriveSpeed;
                    this.distance *= weirdSidewaysDistanceCoefficient;
                    break;
                default:
                    break;
            }

            // reset encoders
            front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            front_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            front_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            rear_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rear_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            rear_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rear_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            // wait for end of execution
            //isRunning = 0b1111;
            imu.setTargetHeading(absoluteHeading);
            completed = false;
            driveOrdered = true;
            driveState = DriveState.ACCELERATE;
            driveSpeedCoefficient = 0.1+minDriveSpeed/maxDriveSpeed;
        }
    }


    public void brake(DriveDirection driveDirection, double distance)
    {
        if(!driveOrdered) {
            this.driveDirection = driveDirection;
            this.distance = distance;
            switch (driveDirection) {
                case FORWARD:
                    front_left_power = maxDriveSpeed;
                    front_right_power = maxDriveSpeed;
                    rear_left_power = maxDriveSpeed;
                    rear_right_power = maxDriveSpeed;
                    this.distance *= weirdForwardDistanceCoefficient;
                    break;
                case BACKWARD:
                    front_left_power = -maxDriveSpeed;
                    front_right_power = -maxDriveSpeed;
                    rear_left_power = -maxDriveSpeed;
                    rear_right_power = -maxDriveSpeed;
                    this.distance *= weirdForwardDistanceCoefficient;
                    break;
                case RIGHT:
                    front_left_power = maxDriveSpeed;
                    front_right_power = -maxDriveSpeed;
                    rear_left_power = -maxDriveSpeed;
                    rear_right_power = maxDriveSpeed;
                    this.distance *= weirdSidewaysDistanceCoefficient;
                    break;
                case LEFT:
                    front_left_power = -maxDriveSpeed;
                    front_right_power = maxDriveSpeed;
                    rear_left_power = maxDriveSpeed;
                    rear_right_power = -maxDriveSpeed;
                    this.distance *= weirdSidewaysDistanceCoefficient;
                    break;
                default:
                    break;
            }

            front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            front_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            front_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            rear_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rear_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            rear_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rear_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            // wait for end of execution
            //isRunning = 0b1111;
            imu.setTargetHeading(absoluteHeading);
            completed = false;
            driveOrdered = true;
            driveState = DriveState.BRAKE;
            driveSpeedCoefficient /= maxDriveSpeed;
        }
    }

    public void turn(TurnDirection turnDirection, double angle)
    {
        if(!driveOrdered) {
            this.turnDirection = turnDirection;
            switch (turnDirection) {
                case RIGHT:
                    front_left_power = maxDriveSpeed;
                    front_right_power = -maxDriveSpeed;
                    rear_left_power = maxDriveSpeed;
                    rear_right_power = -maxDriveSpeed;
                    absoluteHeading -= angle;

                    break;
                case LEFT:
                    front_left_power = -maxDriveSpeed;
                    front_right_power = maxDriveSpeed;
                    rear_left_power = -maxDriveSpeed;
                    rear_right_power = maxDriveSpeed;
                    absoluteHeading += angle;
                    break;
                default:
                    break;
            }

            // reset encoders
            front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            front_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            front_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            rear_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rear_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            rear_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rear_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            if (absoluteHeading > 360) {
                absoluteHeading -= 360;
            }
            if (absoluteHeading < 0) {
                absoluteHeading += 360;
            }
            imu.setTargetHeading(absoluteHeading);
            this.distance = ((wheelDistance*Math.PI)/360)*angle;
            driveOrdered = true;
            completed = false;
            driveState = DriveState.TURNING;
            turnSpeedCoefficient = 0.1+minTurnSpeed/maxTurnSpeed;
            braking = false;
        }
        /*
        if(!driveOrdered) {
            this.turnDirection = turnDirection;
            switch (turnDirection) {
                case LEFT:
                    front_left_power = -turnSpeed;
                    front_right_power = turnSpeed;
                    rear_left_power = -turnSpeed;
                    rear_right_power = turnSpeed;
                    absoluteHeading += angle;
                    break;
                case RIGHT:
                    front_left_power = turnSpeed;
                    front_right_power = -turnSpeed;
                    rear_left_power = turnSpeed;
                    rear_right_power = -turnSpeed;
                    absoluteHeading -= angle;
                    break;
                default:
                    break;
            }
            if (absoluteHeading > 360) {
                absoluteHeading -= 360;
            }
            if (absoluteHeading < 0) {
                absoluteHeading += 360;
            }
            imu.setTargetHeading(absoluteHeading);
            driveOrdered = true;
            completed = false;
            driveState = DriveState.TURNING;
            imu.update();
        }

         */
    }

    public void specialTurn(TurnDirection turnDirection) {
        if(!driveOrdered) {
            this.turnDirection = turnDirection;
            switch (turnDirection) {
                case RIGHT:
                    front_left_power = -0.6*maxDriveSpeed;
                    front_right_power = -maxDriveSpeed;
                    rear_left_power = -0.6*maxDriveSpeed;
                    rear_right_power = -maxDriveSpeed;
                    absoluteHeading -= 90;

                    break;
                case LEFT:
                    front_left_power = -maxDriveSpeed;
                    front_right_power = -specialTurnRatio*maxDriveSpeed;
                    rear_left_power = -maxDriveSpeed;
                    rear_right_power = -specialTurnRatio*maxDriveSpeed;
                    absoluteHeading += 90;
                    break;
                default:
                    break;
            }

            // reset encoders
            front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            front_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            front_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            rear_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rear_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            rear_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rear_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            if (absoluteHeading > 360) {
                absoluteHeading -= 360;
            }
            if (absoluteHeading < 0) {
                absoluteHeading += 360;
            }
            imu.setTargetHeading(absoluteHeading);
            //this.distance = (((22+specialTurnRadius)*2 * Math.PI) / 360) * 90 * specialTurnDistanceCoefficient;
            if(turnDirection == TurnDirection.LEFT) {
                this.distance = 180;
            } else {
                this.distance = 155;
            }
            driveOrdered = true;
            completed = false;
            driveState = DriveState.SPECIAL_TURNING;
            turnSpeedCoefficient = 0.1 + minTurnSpeed / maxTurnSpeed;
            braking = false;
        }
    }

    private void getDriveSpeedCoefficient(double minDriveSpeed, double averageMotorDistance)
    {
        double remainingDistance = distance - averageMotorDistance/EPC;
        double changeCoefficient = 1.4; //the distance in cm the robot has to travel to brake/accelerate at theoretical 0.1 power
        if(remainingDistance < changeCoefficient*(maxDriveSpeed*driveSpeedCoefficient*10*
                (maxDriveSpeed*driveSpeedCoefficient*10+1)/2
                - minDriveSpeed*10*(minDriveSpeed*10+1)/2)) //check for deceleration with gauss
        {
            if(!braking)
            {
                braking = true;
            }
            driveSpeedCoefficient -= (1-minDriveSpeed/maxDriveSpeed)/*the difference from 1 to the min driveCoefficient*/
                    /((maxDriveSpeed-minDriveSpeed)/0.1)/*the amount of steps*/;
        } else if(((averageMotorDistance / EPC) > (changeCoefficient * (((maxDriveSpeed * driveSpeedCoefficient
                * 10 * ((maxDriveSpeed * driveSpeedCoefficient * 10) + 1)) / 2)
                - ((minDriveSpeed * 10 * ((minDriveSpeed * 10) + 1)) / 2))))
                && (driveSpeedCoefficient < 1) && !braking) //check for acceleration with gauss
        {
            driveSpeedCoefficient += (1-minDriveSpeed/maxDriveSpeed)/*the difference from 1 to the min driveCoefficient*/
                    /((maxDriveSpeed-minDriveSpeed)/0.1)/*the amount of steps*/;
        }
        driveSpeedCoefficient = Math.max(Math.min(driveSpeedCoefficient, 1), minDriveSpeed/maxDriveSpeed);
    }

    private void getAcceleration(double minDriveSpeed, double averageMotorDistance)
    {
        double changeCoefficient = 1.3; //the distance in cm the robot has to travel to brake/accelerate at theoretical 0.1 power
        if(((averageMotorDistance / EPC) > (changeCoefficient * (((maxDriveSpeed * driveSpeedCoefficient * 10 * ((maxDriveSpeed * driveSpeedCoefficient * 10) + 1)) / 2)
                - ((minDriveSpeed * 10 * ((minDriveSpeed * 10) + 1)) / 2)))) && (driveSpeedCoefficient < 1)) //check for acceleration with gauss
        {
            driveSpeedCoefficient += (1-minDriveSpeed/maxDriveSpeed)/*the difference from 1 to the min driveCoefficient*/
                    /((maxDriveSpeed-minDriveSpeed)/0.1)/*the amount of steps*/;
        }
        driveSpeedCoefficient = Math.max(Math.min(driveSpeedCoefficient, 1), minDriveSpeed/maxDriveSpeed);
    }

    private void getBrake(double minDriveSpeed, double averageMotorDistance)
    {
        double remainingDistance = distance - averageMotorDistance/EPC;
        double changeCoefficient = 1.3; //the distance in cm the robot has to travel to brake/accelerate at theoretical 0.1 power
        if(remainingDistance < changeCoefficient*(maxDriveSpeed*driveSpeedCoefficient*10*(maxDriveSpeed*driveSpeedCoefficient*10+1)/2
                - minDriveSpeed*10*(minDriveSpeed*10+1)/2)) //check for deceleration with gauss
        {
            driveSpeedCoefficient -= (1-minDriveSpeed/maxDriveSpeed)/*the difference from 1 to the min driveCoefficient*/
                    /((maxDriveSpeed-minDriveSpeed)/0.1)/*the amount of steps*/;
        }
        driveSpeedCoefficient = Math.max(Math.min(driveSpeedCoefficient, 1), minDriveSpeed/maxDriveSpeed);
    }

    private void getTurnSpeedCoefficient(double averageMotorDistance)
    {
        double remainingDistance = distance - averageMotorDistance/EPC;
        double changeCoefficient = 1.7; //the distance in cm the robot has to travel to brake/accelerate at theoretical 0.1 power
        if(remainingDistance < changeCoefficient*(maxTurnSpeed*turnSpeedCoefficient*10*(maxTurnSpeed*turnSpeedCoefficient*10+1)/2
                - minTurnSpeed*10*(minTurnSpeed*10+1)/2)) //check for deceleration with gauss
        {
            if(!braking)
            {
                braking = true;
            }
            turnSpeedCoefficient -= (1-minTurnSpeed/maxTurnSpeed)/*the difference from 1 to the min driveCoefficient*/
                    /((maxTurnSpeed-minTurnSpeed)/0.1)/*the amount of steps*/;
        } else if(((averageMotorDistance / EPC) > (changeCoefficient * (((maxTurnSpeed * turnSpeedCoefficient * 10 * ((maxTurnSpeed * turnSpeedCoefficient * 10) + 1)) / 2)
                - ((minTurnSpeed * 10 * ((minTurnSpeed * 10) + 1)) / 2)))) && (turnSpeedCoefficient < 1) && !braking) //check for acceleration with gauss
        {
            turnSpeedCoefficient += (1-minTurnSpeed/maxTurnSpeed)/*the difference from 1 to the min driveCoefficient*/
                    /((maxTurnSpeed-minTurnSpeed)/0.1)/*the amount of steps*/;
        }
        turnSpeedCoefficient = Math.max(Math.min(turnSpeedCoefficient, 1), minTurnSpeed/maxTurnSpeed);
    }


    public void update(double currentRuntime)
    {
        double averageMotorDistance = (Math.abs(front_left.getCurrentPosition())
                + Math.abs(front_right.getCurrentPosition())
                + Math.abs(rear_left.getCurrentPosition())
                + Math.abs(rear_right.getCurrentPosition())) / 4;
        switch(driveState)
        {
            case DRIVING:
                //if (isRunning == 0b1111) {

                // calculate power
                if (averageMotorDistance < Math.abs(distance*EPC)/*Math.abs(front_left.getCurrentPosition()) < Math.abs(distance*EPC)
                        && Math.abs(front_right.getCurrentPosition()) < Math.abs(distance*EPC)
                        && Math.abs(rear_left.getCurrentPosition()) < Math.abs(distance*EPC)
                        && Math.abs(rear_right.getCurrentPosition()) < Math.abs(distance*EPC)*/) {
                    imu.update();
                    double correction = imu.getDriveCorrection();
                    if(driveDirection == DriveDirection.FORWARD || driveDirection == DriveDirection.BACKWARD)
                    {
                        getDriveSpeedCoefficient(minDriveSpeed, averageMotorDistance);
                    } else {
                        getDriveSpeedCoefficient(minStrafeSpeed, averageMotorDistance);
                    }
                    front_left_power = maxDriveSpeed*driveSpeedCoefficient*(front_left_power/Math.abs(front_left_power)+correction);
                    front_right_power = maxDriveSpeed*driveSpeedCoefficient*(front_right_power/Math.abs(front_right_power)-correction);
                    rear_left_power = maxDriveSpeed*driveSpeedCoefficient*(rear_left_power/Math.abs(rear_left_power)+correction);
                    rear_right_power = maxDriveSpeed*driveSpeedCoefficient*(rear_right_power/Math.abs(rear_right_power)-correction);
                    // apply power
                    updatePower();
                    timerStart = currentRuntime;
                } else {
                    stop();
                    if(currentRuntime > timerStart + 0.15)
                    {
                        driveState = DriveState.RESTING;
                        completed = true;
                    }
                }
                break;
            case ACCELERATE:
                //if (isRunning == 0b1111) {

                // calculate power
                if (Math.abs(line_sensor.blue()-line_sensor.red()) < 10
                        && Math.abs(front_left.getCurrentPosition()) < Math.abs(distance*EPC)
                        && Math.abs(front_right.getCurrentPosition()) < Math.abs(distance*EPC)
                        && Math.abs(rear_left.getCurrentPosition()) < Math.abs(distance*EPC)
                        && Math.abs(rear_right.getCurrentPosition()) < Math.abs(distance*EPC)) {
                    double correction = 0;
                    if(useIMU) {
                        imu.update();
                        correction = imu.getDriveCorrection();
                    }
                    if(driveDirection == DriveDirection.FORWARD || driveDirection == DriveDirection.BACKWARD)
                    {
                        getAcceleration(minDriveSpeed, averageMotorDistance);
                    } else {
                        getAcceleration(minStrafeSpeed, averageMotorDistance);
                    }
                    front_left_power = maxDriveSpeed*driveSpeedCoefficient*(front_left_power/Math.abs(front_left_power)+correction);
                    front_right_power = maxDriveSpeed*driveSpeedCoefficient*(front_right_power/Math.abs(front_right_power)-correction);
                    rear_left_power = maxDriveSpeed*driveSpeedCoefficient*(rear_left_power/Math.abs(rear_left_power)+correction);
                    rear_right_power = maxDriveSpeed*driveSpeedCoefficient*(rear_right_power/Math.abs(rear_right_power)-correction);
                    // apply power
                    updatePower();
                    timerStart = currentRuntime;
                } else {
                    driveSpeedCoefficient *= maxDriveSpeed;
                    driveState = DriveState.RESTING;
                    completed = true;
                }
                break;
            case BRAKE:
                //if (isRunning == 0b1111) {

                // calculate power
                if (Math.abs(front_left.getCurrentPosition()) < Math.abs(distance*EPC)
                        && Math.abs(front_right.getCurrentPosition()) < Math.abs(distance*EPC)
                        && Math.abs(rear_left.getCurrentPosition()) < Math.abs(distance*EPC)
                        && Math.abs(rear_right.getCurrentPosition()) < Math.abs(distance*EPC)) {
                    imu.update();
                    double correction = imu.getDriveCorrection();
                    if(driveDirection == DriveDirection.FORWARD || driveDirection == DriveDirection.BACKWARD)
                    {
                        getBrake(minDriveSpeed, averageMotorDistance);
                    } else {
                        getBrake(minStrafeSpeed, averageMotorDistance);
                    }
                    front_left_power = maxDriveSpeed*driveSpeedCoefficient*(front_left_power/Math.abs(front_left_power)+correction);
                    front_right_power = maxDriveSpeed*driveSpeedCoefficient*(front_right_power/Math.abs(front_right_power)-correction);
                    rear_left_power = maxDriveSpeed*driveSpeedCoefficient*(rear_left_power/Math.abs(rear_left_power)+correction);
                    rear_right_power = maxDriveSpeed*driveSpeedCoefficient*(rear_right_power/Math.abs(rear_right_power)-correction);
                    // apply power
                    updatePower();
                    timerStart = currentRuntime;
                } else {
                    stop();
                    if(currentRuntime > timerStart + 0.15)
                    {
                        driveState = DriveState.RESTING;
                        completed = true;
                    }
                }
                break;
            case TURNING:
                // calculate power
                if (Math.abs(front_left.getCurrentPosition()) < Math.abs(distance*EPC)
                        && Math.abs(front_right.getCurrentPosition()) < Math.abs(distance*EPC)
                        && Math.abs(rear_left.getCurrentPosition()) < Math.abs(distance*EPC)
                        && Math.abs(rear_right.getCurrentPosition()) < Math.abs(distance*EPC)) {
                    getTurnSpeedCoefficient(averageMotorDistance);
                    front_left_power = maxTurnSpeed*turnSpeedCoefficient*(front_left_power/Math.abs(front_left_power));
                    front_right_power = maxTurnSpeed*turnSpeedCoefficient*(front_right_power/Math.abs(front_right_power));
                    rear_left_power = maxTurnSpeed*turnSpeedCoefficient*(rear_left_power/Math.abs(rear_left_power));
                    rear_right_power = maxTurnSpeed*turnSpeedCoefficient*(rear_right_power/Math.abs(rear_right_power));
                    // apply power
                    updatePower();
                    timerStart = currentRuntime;
                } else {
                    stop();
                    if(currentRuntime > timerStart + 0.15)
                    {
                        driveState = DriveState.RESTING;
                        completed = true;
                    }
                }
                break;
            case SPECIAL_TURNING:
                // calculate power
                //imu.update();
                if(turnDirection == TurnDirection.LEFT)
                {
                    if (Math.abs(front_left.getCurrentPosition()) < Math.abs(distance*EPC)
                            && Math.abs(front_right.getCurrentPosition()*specialTurnRatio) < Math.abs(distance*EPC)
                            && Math.abs(rear_left.getCurrentPosition()) < Math.abs(distance*EPC)
                            && Math.abs(rear_right.getCurrentPosition()*specialTurnRatio) < Math.abs(distance*EPC)) {
                        front_left_power = maxTurnSpeed*(front_left_power/Math.abs(front_left_power));
                        front_right_power = -0.65*maxTurnSpeed*(front_right_power/Math.abs(front_right_power));
                        rear_left_power = maxTurnSpeed*(rear_left_power/Math.abs(rear_left_power));
                        rear_right_power = -0.65*maxTurnSpeed*(rear_right_power/Math.abs(rear_right_power));
                        // apply power
                        updatePower();
                        timerStart = currentRuntime;
                    } else {
                        stop();
                        if(currentRuntime > timerStart + 0.15)
                        {
                            driveState = DriveState.RESTING;
                            completed = true;
                        }
                    }
                } else {
                    if (Math.abs(front_left.getCurrentPosition()*specialTurnRatio) < Math.abs(distance*EPC)
                            && Math.abs(front_right.getCurrentPosition()) < Math.abs(distance*EPC)
                            && Math.abs(rear_left.getCurrentPosition()*specialTurnRatio) < Math.abs(distance*EPC)
                            && Math.abs(rear_right.getCurrentPosition()) < Math.abs(distance*EPC)) {
                        front_left_power = -0.65*maxTurnSpeed*(front_left_power/Math.abs(front_left_power));
                        front_right_power = maxTurnSpeed*(front_right_power/Math.abs(front_right_power));
                        rear_left_power = -0.65*maxTurnSpeed*(rear_left_power/Math.abs(rear_left_power));
                        rear_right_power = maxTurnSpeed*(rear_right_power/Math.abs(rear_right_power));
                        // apply power
                        updatePower();
                        timerStart = currentRuntime;
                    } else {
                        stop();
                        if(currentRuntime > timerStart + 0.15)
                        {
                            driveState = DriveState.RESTING;
                            completed = true;
                        }
                    }
                }
                break;
                /*
                if (Math.abs(imu.getHeading() - absoluteHeading) > 5) {
                    imu.update();
                    double turnSpeedCoefficient = Math.max(Math.min(imu.getTurnSpeedCoefficient(turnDirection == TurnDirection.LEFT)/turnSpeed, 1), minTurnSpeed/turnSpeed);
                    switch (turnDirection) {
                        case LEFT:
                            front_left_power = -turnSpeed*turnSpeedCoefficient;
                            front_right_power = turnSpeed*turnSpeedCoefficient;
                            rear_left_power = -turnSpeed*turnSpeedCoefficient;
                            rear_right_power = turnSpeed*turnSpeedCoefficient;
                            break;
                        case RIGHT:
                            front_left_power = turnSpeed*turnSpeedCoefficient;
                            front_right_power = -turnSpeed*turnSpeedCoefficient;
                            rear_left_power = turnSpeed*turnSpeedCoefficient;
                            rear_right_power = -turnSpeed*turnSpeedCoefficient;
                            break;
                        default:
                            break;
                    }
                    updatePower();
                    timerStart = currentRuntime;
                } else {
                    stop();
                    if(currentRuntime > timerStart + 0.3)
                    {
                        driveState = DriveState.RESTING;
                        completed = true;
                    }
                }
                break;
                 */
            case RESTING:
                break;
            default:
                break;
        }
    }

    public void resetOrder()
    {
        driveOrdered = false;
    }

    public boolean getCompleted()
    {
        return  completed;
    }

    public void setMaxDriveSpeed(double maxDriveSpeed) {
        this.maxDriveSpeed = maxDriveSpeed;
    }

    public void setMaxTurnSpeed(double maxTurnSpeed) {
        this.maxTurnSpeed = maxTurnSpeed;
    }

    public void brakeFloat() {
        brakeFloat();
    }
}
