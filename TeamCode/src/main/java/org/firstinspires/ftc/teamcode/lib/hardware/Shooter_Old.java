package org.firstinspires.ftc.teamcode.lib.hardware;

public class Shooter_Old {/*

    private String alliance;
    private HardwareMap hardwareMap;

    private enum State {
        Idle,
        Shooting;
    }
    private State state = State.Idle;

    Twople[] comms = {};

    private DcMotorEx motorOne;
    private DcMotorEx motorTwo;
    private int lastDistance = 0;
    private long lastTime = 0;
    private long timeSinceLast = 0;
    private double distance = 0;
    private double MIN_DISTANCE = 0;
    private boolean motorsReady = false;

    public enum Targets  {
        blueUpper (new Vector(5,-3,3)),
        blueMiddle (new Vector(1,2,3)),
        blueLower (new Vector(1,2,3)),
        bluePsOuter (new Vector(1,2,3)),
        bluePsMiddle (new Vector(1,2,3)),
        bluePsInner (new Vector(1,2,3)),
        redUpper (new Vector(1,2,3)),
        redMiddle (new Vector(1,2,3)),
        redLower (new Vector(1,2,3)),
        redPsOuter (new Vector(1,2,3)),
        redPsMiddle (new Vector(1,2,3)),
        redPsInner (new Vector(1,2,3));

        public Vector coordinates = new Vector();

        public Vector getCoordinates() {
            return coordinates;
        }

        private Targets(Vector coordinates) {
            this.coordinates = coordinates;
        }
    }

    public Shooter (HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        currentTarget = Targets.blueUpper;

        motorOne = hardwareMap.get(DcMotorEx.class, "shooterOne");
        motorTwo = hardwareMap.get(DcMotorEx.class, "shooterTwo");
    }

    private void adjustAlpha(Vector position) {
        Vector target = new Vector(0,2);
        double angle = Math.atan((target.x - position.x) /(target.y - position.y));
        comms[comms.length + 1] = new Twople("odometry", new INTuple("turn", new double[]{angle}));
    }

    private void adjustTheta() {
    }

    public Twople[] update(INTuple[] instructions, Vector position) {
        for (int i = 0; i < instructions.length; i++) {
            //  instructions is an array of INTuples (Instruction, [data, data])
            switch (instructions[i].a) {
                case "fire":
                    adjustAlpha(position);
                    state = State.Shooting;
                    break;

                default:
                    break;
            }
        }
        return comms;
    }

    private void motorsReadyLoop() {
        distance = motorOne.getCurrentPosition() + motorTwo.getCurrentPosition() - lastDistance;
        lastDistance = motorOne.getCurrentPosition() + motorTwo.getCurrentPosition();

        timeSinceLast = System.nanoTime() - lastTime;
        lastTime = System.nanoTime();
        if (distance / timeSinceLast > MIN_DISTANCE) {
            motorsReady = true;
        } else {
            motorsReady = false;
        }
    }

    public void shootingLoop() {
        motorsReadyLoop();
    }

    public void shootAllStatic(Pose2d position) {
        Servo servo = hardwareMap.get(Servo.class, "shooter");

        Vector currentPos = new Vector(position.component1(), position.component2());
        double expectedRotation = currentTarget.getCoordinates().subtract(currentPos).angle();
        Pose2d targetPose = new Pose2d(currentPos.x, currentPos.y, Math.toRadians(expectedRotation));
        Trajectory turn = drive.trajectoryBuilder(new Pose2d()).lineToSplineHeading(targetPose).build();

        drive.followTrajectory(turn);

        motorOne.setPower(1);
        motorTwo.setPower(1);

        for (int i = 0; i < 3; i++) {
            adjustTheta();
            while (!motorsReady) {
                motorsReadyLoop();
            }
            servo.setPosition(0);   //Feed Ring
            servo.setPosition(1);
        }
        motorOne.setPower(0);
        motorTwo.setPower(0);
    }
    */
}

