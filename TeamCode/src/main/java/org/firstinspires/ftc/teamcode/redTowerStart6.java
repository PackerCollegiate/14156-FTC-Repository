package org.firstinspires.ftc.teamcode; // ← update to match your package

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

// ┌─────────────────────────────────────────────────────────────────────────────┐
// │  redTowerStart6 — 6-Ball PedroPathing Autonomous  (RED SIDE)                │
// │                                                                             │
// │  Mirror of DecodeAuto (blue side). Transformations applied:                 │
// │    X coordinate : x_red = 144 - x_blue                                     │
// │    Heading      : h_red = 180° - h_blue                                     │
// │      135° → 45°   (faces upper-right toward red tower)                     │
// │      180° → 0°    (faces right while sweeping)                              │
// │                                                                             │
// │  SEQUENCE:                                                                  │
// │  Path1  Start(115,127) → Score(88,100)         launcher ramps up           │
// │  SHOOT  Fire 2 preloaded balls                                              │
// │  Path2  Score → Row1 Entry(77.965,83.583)      intake ON                   │
// │  Path3  Row1 Entry → Row1 End(116.209,83.661)  intake sweeping →           │
// │  DWELL  1.2s at Row1 End                       intake collecting           │
// │  Path4  Row1 End → Score                       launcher ramps up           │
// │  SHOOT  Fire 2 row-1 balls                                                  │
// │  Path5  Score → Row2 Entry(78.591,60.209)      intake ON                   │
// │  Path6  Row2 Entry → Row2 End(111.948,60.209)  intake sweeping →           │
// │  DWELL  1.2s at Row2 End                       intake collecting           │
// │  Path7  Row2 End → Score                       launcher ramps up           │
// │  SHOOT  Fire 2 row-2 balls                                                  │
// └─────────────────────────────────────────────────────────────────────────────┘

@Autonomous(name = "Red Tower Start 6-Ball", group = "Competition")
public class redTowerStart6 extends OpMode {

    // ═══════════════════════════════════════════════════════
    //  PedroPathing
    // ═══════════════════════════════════════════════════════
    private Follower    follower;
    private Timer       pathTimer;
    private ElapsedTime opmodeTimer;
    private int         pathState;

    // ═══════════════════════════════════════════════════════
    //  Hardware
    // ═══════════════════════════════════════════════════════
    private DcMotorEx launchMotor;
    private DcMotorEx frontIntake;
    private CRServo   servoIntake;
    private Servo     servoGate;
    private DcMotorEx leftExtension;
    private DcMotorEx rightExtension;

    // ═══════════════════════════════════════════════════════
    //  Launcher PID
    // ═══════════════════════════════════════════════════════
    private double kP            = 0.006;
    private double kD            = 0.00002;
    private double kF            = 0.00043;
    private double offsetF       = 0.0;
    private double previousError = 0.0;
    private double lastLoopTime  = 0.0;

    // ═══════════════════════════════════════════════════════
    //  RPM settings
    // ═══════════════════════════════════════════════════════
    private static final double SCORE_DISTANCE = 49.0;
    private static final double IDLE_RPM       = 1500;
    private double rpmTarget = IDLE_RPM;

    // ═══════════════════════════════════════════════════════
    //  Gate / Intake constants
    // ═══════════════════════════════════════════════════════
    private static final double GATE_CLOSED    = 0.86;
    private static final double GATE_OPEN      = 1.0;
    private static final double PUSH_TIME      = 2.5;
    private static final double INTAKE_POWER   = 1.0;
    private static final double INTAKE_FEED    = 0.7;
    private static final double INTAKE_DWELL   = 1.2;
    private static final double SWEEP_SPEED    = 0.35;
    private static final double APPROACH_SPEED = 0.6;
    private static final double FULL_SPEED     = 1.0;

    // ═══════════════════════════════════════════════════════
    //  Shoot sub-state machine
    // ═══════════════════════════════════════════════════════
    private enum ShootState { IDLE, FIRST_BALL, RECOVERY, SECOND_BALL, DONE }
    private ShootState  shootState    = ShootState.IDLE;
    private ElapsedTime ballTimer     = new ElapsedTime();
    private ElapsedTime recoveryTimer = new ElapsedTime();

    // ═══════════════════════════════════════════════════════
    //  Linear slide constants
    // ═══════════════════════════════════════════════════════
    private static final int    SLIDE_RETRACTED  = 0;
    private static final double SLIDE_MOVE_POWER = 0.9;
    private static final double SLIDE_HOLD_POWER = 0.4;
//    private static final int    SYNC_TOLERANCE   = 25;

    // ═══════════════════════════════════════════════════════
    //  Field poses  — RED SIDE
    //  Formula: x_red = 144 - x_blue,  h_red = 180° - h_blue
    //
    //  Blue start  (29,    127,  135°) → Red (115,    127, 45°)
    //  Blue score  (56,    100,  135°) → Red (88,     100, 45°)
    //  Blue r1Rt   (66.035, 83.583, 180°) → Red (77.965, 83.583, 0°)
    //  Blue r1Lt   (27.791, 83.661, 180°) → Red (116.209, 83.661, 0°)
    //  Blue r2Rt   (65.409, 60.209, 180°) → Red (78.591, 60.209, 0°)
    //  Blue r2Lt   (32.052, 60.209, 180°) → Red (111.948, 60.209, 0°)
    // ═══════════════════════════════════════════════════════
    private final Pose startPose     = new Pose(115.000,  127.000, Math.toRadians(45));
    private final Pose scorePose     = new Pose(88.000,   100.000, Math.toRadians(45));
    private final Pose row1EntryPose = new Pose(77.965,   83.583,  Math.toRadians(0));  // entry side (right of field)
    private final Pose row1EndPose   = new Pose(116.209,  83.661,  Math.toRadians(0));  // sweep end  (left of field)
    private final Pose row2EntryPose = new Pose(78.591,   60.209,  Math.toRadians(0));
    private final Pose row2EndPose   = new Pose(111.948,  60.209,  Math.toRadians(0));

    // ═══════════════════════════════════════════════════════
    //  Paths
    // ═══════════════════════════════════════════════════════
    private PathChain Path1; // Start       → Score          45°→45°
    private PathChain Path2; // Score       → Row1 Entry     45°→0°
    private PathChain Path3; // Row1 Entry  → Row1 End       0°→0°   (sweep →)
    private PathChain Path4; // Row1 End    → Score          0°→45°
    private PathChain Path5; // Score       → Row2 Entry     45°→0°
    private PathChain Path6; // Row2 Entry  → Row2 End       0°→0°   (sweep →)
    private PathChain Path7; // Row2 End    → Score          0°→45°

    // ═══════════════════════════════════════════════════════════════════════
    //  buildPaths()
    //  All headings are 180° - blue_heading:  135°→45°,  180°→0°
    // ═══════════════════════════════════════════════════════════════════════
    public void buildPaths() {

        Path1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(45))
                .build();

        Path2 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, row1EntryPose))
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                .build();

        // Sweep goes entry→end (low X → high X, i.e. left→right on red side)
        Path3 = follower.pathBuilder()
                .addPath(new BezierLine(row1EntryPose, row1EndPose))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        Path4 = follower.pathBuilder()
                .addPath(new BezierLine(row1EndPose, scorePose))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
                .build();

        Path5 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, row2EntryPose))
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                .build();

        Path6 = follower.pathBuilder()
                .addPath(new BezierLine(row2EntryPose, row2EndPose))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        Path7 = follower.pathBuilder()
                .addPath(new BezierLine(row2EndPose, scorePose))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
                .build();
    }

    // ═══════════════════════════════════════════════════════════════════════
    //  beginShootSequence()
    // ═══════════════════════════════════════════════════════════════════════
    private void beginShootSequence() {
        rpmTarget     = distanceToRPM(SCORE_DISTANCE);
        previousError = 0;
        shootState    = ShootState.IDLE;
        servoGate.setPosition(GATE_CLOSED);
        servoIntake.setPower(0);
        frontIntake.setPower(0);
    }

    // ═══════════════════════════════════════════════════════════════════════
    //  runShootSequence()  — identical to DecodeAuto
    //  Returns true when both balls have been fired.
    // ═══════════════════════════════════════════════════════════════════════
    private boolean runShootSequence() {
        double actualRPM = (launchMotor.getVelocity() * 60.0) / 28.0;

        switch (shootState) {
            case IDLE:
                servoGate.setPosition(GATE_CLOSED);
                servoIntake.setPower(0);
                frontIntake.setPower(0);
                if (actualRPM > 0.9 * rpmTarget) {
                    shootState = ShootState.FIRST_BALL;
                    ballTimer.reset();
                    servoIntake.setPower(1.0);
                    frontIntake.setPower(INTAKE_FEED);
                    servoGate.setPosition(GATE_OPEN);
                }
                break;

            case FIRST_BALL:
                servoGate.setPosition(GATE_OPEN);
                servoIntake.setPower(1.0);
                frontIntake.setPower(INTAKE_FEED);
                if (ballTimer.seconds() > PUSH_TIME) {
                    shootState = ShootState.RECOVERY;
                    servoIntake.setPower(1.0);
                    frontIntake.setPower(INTAKE_POWER);
                    servoGate.setPosition(GATE_CLOSED);
                    recoveryTimer.reset();
                }
                break;

            case RECOVERY:
                servoIntake.setPower(1.0);
                frontIntake.setPower(INTAKE_POWER);
                servoGate.setPosition(GATE_CLOSED);
                if (actualRPM > 0.95 * rpmTarget) {
                    shootState = ShootState.SECOND_BALL;
                    ballTimer.reset();
                }
                break;

            case SECOND_BALL:
                servoIntake.setPower(1.0);
                frontIntake.setPower(INTAKE_FEED);
                servoGate.setPosition(GATE_OPEN);
                if (ballTimer.seconds() > PUSH_TIME) {
                    shootState = ShootState.DONE;
                    servoIntake.setPower(0);
                    frontIntake.setPower(0);
                    servoGate.setPosition(GATE_CLOSED);
                }
                break;

            case DONE:
                return true;
        }
        return false;
    }

    // ═══════════════════════════════════════════════════════════════════════
    //  autonomousPathUpdate()  — identical logic to DecodeAuto
    //  Only the poses and headings differ (handled in buildPaths/poses above)
    // ═══════════════════════════════════════════════════════════════════════
    public void autonomousPathUpdate() {
        switch (pathState) {

            case 0:
                rpmTarget     = distanceToRPM(SCORE_DISTANCE);
                previousError = 0;
                servoGate.setPosition(GATE_CLOSED);
                frontIntake.setPower(0);
                servoIntake.setPower(0);
                follower.setMaxPower(FULL_SPEED);
                follower.followPath(Path1);
                setPathState(1);
                break;

            case 1:
                if (!follower.isBusy()) {
                    beginShootSequence();
                    setPathState(2);
                }
                break;

            case 2: // Shoot 2 preloaded balls
                if (runShootSequence()) {
                    rpmTarget     = IDLE_RPM;
                    previousError = 0;
                    frontIntake.setPower(INTAKE_POWER);
                    servoIntake.setPower(1.0);
                    follower.setMaxPower(APPROACH_SPEED);
                    follower.followPath(Path2, true);
                    setPathState(3);
                }
                break;

            case 3: // Driving to row 1 entry — intake ON
                frontIntake.setPower(INTAKE_POWER);
                servoIntake.setPower(1.0);
                if (!follower.isBusy()) {
                    follower.setMaxPower(SWEEP_SPEED);
                    follower.followPath(Path3, true);
                    setPathState(4);
                }
                break;

            case 4: // Sweeping across row 1 →
                frontIntake.setPower(INTAKE_POWER);
                servoIntake.setPower(1.0);
                if (!follower.isBusy()) {
                    setPathState(5);
                }
                break;

            case 5: // Dwell at row 1 end
                frontIntake.setPower(INTAKE_POWER);
                servoIntake.setPower(1.0);
                if (pathTimer.getElapsedTimeSeconds() > INTAKE_DWELL) {
                    frontIntake.setPower(0);
                    servoIntake.setPower(0);
                    servoGate.setPosition(GATE_CLOSED);
                    rpmTarget     = distanceToRPM(SCORE_DISTANCE);
                    previousError = 0;
                    follower.setMaxPower(FULL_SPEED);
                    follower.followPath(Path4, true);
                    setPathState(6);
                }
                break;

            case 6: // Driving back to score — launcher ramping
                if (!follower.isBusy()) {
                    beginShootSequence();
                    setPathState(7);
                }
                break;

            case 7: // Shoot 2 row-1 balls
                if (runShootSequence()) {
                    rpmTarget     = IDLE_RPM;
                    previousError = 0;
                    frontIntake.setPower(INTAKE_POWER);
                    servoIntake.setPower(1.0);
                    follower.setMaxPower(APPROACH_SPEED);
                    follower.followPath(Path5, true);
                    setPathState(8);
                }
                break;

            case 8: // Driving to row 2 entry — intake ON
                frontIntake.setPower(INTAKE_POWER);
                servoIntake.setPower(1.0);
                if (!follower.isBusy()) {
                    follower.setMaxPower(SWEEP_SPEED);
                    follower.followPath(Path6, true);
                    setPathState(9);
                }
                break;

            case 9: // Sweeping across row 2 →
                frontIntake.setPower(INTAKE_POWER);
                servoIntake.setPower(1.0);
                if (!follower.isBusy()) {
                    setPathState(10);
                }
                break;

            case 10: // Dwell at row 2 end
                frontIntake.setPower(INTAKE_POWER);
                servoIntake.setPower(1.0);
                if (pathTimer.getElapsedTimeSeconds() > INTAKE_DWELL) {
                    frontIntake.setPower(0);
                    servoIntake.setPower(0);
                    servoGate.setPosition(GATE_CLOSED);
                    rpmTarget     = distanceToRPM(SCORE_DISTANCE);
                    previousError = 0;
                    follower.setMaxPower(FULL_SPEED);
                    follower.followPath(Path7, true);
                    setPathState(11);
                }
                break;

            case 11: // Driving back to score — launcher ramping
                if (!follower.isBusy()) {
                    beginShootSequence();
                    setPathState(12);
                }
                break;

            case 12: // Shoot 2 row-2 balls — final shot
                if (runShootSequence()) {
                    rpmTarget     = IDLE_RPM;
                    previousError = 0;
                    setPathState(-1);
                }
                break;

            case -1:
                frontIntake.setPower(0);
                servoIntake.setPower(0);
                servoGate.setPosition(GATE_CLOSED);
                break;
        }
    }

    // ═══════════════════════════════════════════════════════════════════════
    //  setPathState()
    // ═══════════════════════════════════════════════════════════════════════
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    // ═══════════════════════════════════════════════════════════════════════
    //  runLauncher()
    // ═══════════════════════════════════════════════════════════════════════
    private void runLauncher(double dt) {
        double velocityTarget = (rpmTarget / 60.0) * 28.0;
        double actualVelocity = launchMotor.getVelocity();
        double actualRPM      = (actualVelocity * 60.0) / 28.0;
        launchMotor.setPower(updatePDF(velocityTarget, actualRPM, dt));
        telemetry.addData("RPM Target", "%.0f", rpmTarget);
        telemetry.addData("RPM Actual", "%.0f", actualRPM);
    }

    // ═══════════════════════════════════════════════════════════════════════
    //  syncSlides()
    // ═══════════════════════════════════════════════════════════════════════
//    private void syncSlides() {
//        int leftPos  = leftExtension.getCurrentPosition();
//        int rightPos = rightExtension.getCurrentPosition();
//        int target   = leftExtension.getTargetPosition();
//
//        boolean atTarget = Math.abs(leftPos  - target) < 20
//                && Math.abs(rightPos - target) < 20;
//
//        double basePower = atTarget ? SLIDE_HOLD_POWER : SLIDE_MOVE_POWER;
//
//        int drift = leftPos - rightPos;
//        if (Math.abs(drift) > SYNC_TOLERANCE) {
//            double correction = 0.08;
//            if (drift > 0) {
//                leftExtension.setPower(Math.max(0.0, basePower - correction));
//                rightExtension.setPower(Math.min(1.0, basePower + correction));
//            } else {
//                leftExtension.setPower(Math.min(1.0, basePower + correction));
//                rightExtension.setPower(Math.max(0.0, basePower - correction));
//            }
//        } else {
//            leftExtension.setPower(basePower);
//            rightExtension.setPower(basePower);
//        }
//    }

    // ═══════════════════════════════════════════════════════════════════════
    //  updatePDF()
    // ═══════════════════════════════════════════════════════════════════════
    private double updatePDF(double targetTicksPerSec, double actualRPM, double dt) {
        double actualTicksPerSec = (actualRPM / 60.0) * 28.0;
        double error = targetTicksPerSec - actualTicksPerSec;
        double P = kP * error;
        double D = (dt > 0) ? kD * (error - previousError) / dt : 0;
        double F = (kF * targetTicksPerSec) + offsetF;
        previousError = error;
        return Math.max(-1.0, Math.min(1.0, P + D + F));
    }

    // ═══════════════════════════════════════════════════════════════════════
    //  distanceToRPM()
    // ═══════════════════════════════════════════════════════════════════════
    private double distanceToRPM(double distanceInches) {
        return (10.63 * distanceInches) + 2197;
    }

    // ═══════════════════════════════════════════════════════════════════════
    //  emergencyStop()
    // ═══════════════════════════════════════════════════════════════════════
    private void emergencyStop() {
        follower.breakFollowing();
        launchMotor.setPower(0);
        frontIntake.setPower(0);
        servoIntake.setPower(0);
        servoGate.setPosition(GATE_CLOSED);
        leftExtension.setPower(SLIDE_HOLD_POWER);
        rightExtension.setPower(SLIDE_HOLD_POWER);
        pathState = -1;
    }

    // ═══════════════════════════════════════════════════════════════════════
    //  OpMode lifecycle
    // ═══════════════════════════════════════════════════════════════════════

    @Override
    public void init() {
        pathTimer   = new Timer();
        opmodeTimer = new ElapsedTime();
        opmodeTimer.reset();

        launchMotor    = hardwareMap.get(DcMotorEx.class, "launch_motor");
        frontIntake    = hardwareMap.get(DcMotorEx.class, "front_intake");
        servoIntake    = hardwareMap.get(CRServo.class,   "servo_intake");
        servoGate      = hardwareMap.get(Servo.class,     "servo_gate");
        leftExtension  = hardwareMap.get(DcMotorEx.class, "left_extension");
        rightExtension = hardwareMap.get(DcMotorEx.class, "right_extension");

        launchMotor.setDirection(DcMotor.Direction.FORWARD);
        launchMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        frontIntake.setDirection(DcMotor.Direction.REVERSE);
        frontIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        servoGate.setPosition(GATE_CLOSED);

//        leftExtension.setDirection(DcMotor.Direction.REVERSE);
//        rightExtension.setDirection(DcMotor.Direction.FORWARD);
//
//        leftExtension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        rightExtension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        leftExtension.setTargetPosition(SLIDE_RETRACTED);
//        rightExtension.setTargetPosition(SLIDE_RETRACTED);
//
//        leftExtension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        rightExtension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        leftExtension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rightExtension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        leftExtension.setPower(SLIDE_HOLD_POWER);
//        rightExtension.setPower(SLIDE_HOLD_POWER);

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);

        telemetry.addData("Status", "Red 6-ball auto initialized");
        telemetry.update();
    }

    @Override
    public void init_loop() {
//        syncSlides();
        telemetry.addData("X (should be ~115)",  "%.2f", follower.getPose().getX());
        telemetry.addData("Y (should be ~127)",  "%.2f", follower.getPose().getY());
        telemetry.addData("H (should be ~45°)",  "%.1f°", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Left Slide",  leftExtension.getCurrentPosition());
        telemetry.addData("Right Slide", rightExtension.getCurrentPosition());
        telemetry.update();
    }

    @Override
    public void start() {
        opmodeTimer.reset();
        lastLoopTime = opmodeTimer.seconds();
        rpmTarget    = IDLE_RPM;
        setPathState(0);
    }

    @Override
    public void loop() {
        if (opmodeTimer.seconds() >= 29.5) {
            emergencyStop();
            telemetry.addData("STATUS", ">>> AUTO COMPLETE — ALL STOPPED <<<");
            telemetry.update();
            return;
        }

        double now = opmodeTimer.seconds();
        double dt  = now - lastLoopTime;
        lastLoopTime = now;

        follower.update();
        runLauncher(dt);
//        syncSlides();

        autonomousPathUpdate();

        telemetry.addData("── PATH ──────────────────", "");
        telemetry.addData("State",         pathState);
        telemetry.addData("ShootState",    shootState.toString());
        telemetry.addData("State Timer",   "%.2fs", pathTimer.getElapsedTimeSeconds());
        telemetry.addData("Auto Timer",    "%.2fs", opmodeTimer.seconds());
        telemetry.addData("Follower Busy", follower.isBusy());
        telemetry.addData("── POSITION ─────────────", "");
        telemetry.addData("X",             "%.2f", follower.getPose().getX());
        telemetry.addData("Y",             "%.2f", follower.getPose().getY());
        telemetry.addData("Heading",       "%.1f°", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("── INTAKE ───────────────", "");
        telemetry.addData("Front Intake",  "%.1f", frontIntake.getPower());
        telemetry.addData("Gate",          "%.2f", servoGate.getPosition());
        telemetry.addData("── SLIDES ───────────────", "");
        telemetry.addData("Left Pos",      leftExtension.getCurrentPosition());
        telemetry.addData("Right Pos",     rightExtension.getCurrentPosition());
        telemetry.addData("Drift",         leftExtension.getCurrentPosition() - rightExtension.getCurrentPosition());
        telemetry.update();
    }

    @Override
    public void stop() {
        launchMotor.setPower(0);
        frontIntake.setPower(0);
        servoIntake.setPower(0);
        servoGate.setPosition(GATE_CLOSED);
    }
}