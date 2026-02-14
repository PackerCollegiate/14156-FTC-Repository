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
// │  DecodeAuto — 6-Ball PedroPathing Autonomous                                │
// │                                                                             │
// │  SEQUENCE:                                                                  │
// │  Path1  Start(29,127) → Score(56,100)          launcher ramps up           │
// │  SHOOT  Fire 2 preloaded balls  (teleop shoot state machine)                │
// │  Path2  Score → Row1 Right(46,84)              intake ON                   │
// │  Path3  Row1 Right → Row1 Left(26,84)          intake sweeping             │
// │  DWELL  1.2s at Row1 Left                      intake collecting           │
// │  Path4  Row1 Left → Score  (BezierCurve)       launcher ramps up           │
// │  SHOOT  Fire 2 row-1 balls                                                  │
// │  Path5  Score → Row2 Right(46,60)              intake ON                   │
// │  Path6  Row2 Right → Row2 Left(26,60)          intake sweeping             │
// │  DWELL  1.2s at Row2 Left                      intake collecting           │
// │  Path7  Row2 Left → Score  (BezierCurve)       launcher ramps up           │
// │  SHOOT  Fire 2 row-2 balls                                                  │
// │                                                                             │
// │  SLIDES: Held retracted (position 0) the entire auto via syncSlides().     │
// │  LAUNCHER: PID runs every loop() tick — idles at 1500 RPM, ramps to        │
// │            shot RPM before each scoring stop.                               │
// └─────────────────────────────────────────────────────────────────────────────┘

@Autonomous(name = "Decode Auto 6-Ball", group = "Competition")
public class blueTowerStart6 extends OpMode {

    // ═══════════════════════════════════════════════════════
    //  PedroPathing
    // ═══════════════════════════════════════════════════════
    private Follower follower;
    private Timer       pathTimer;   // resets on every setPathState() call
    private ElapsedTime opmodeTimer; // total elapsed time since start()
    private int      pathState;

    // ═══════════════════════════════════════════════════════
    //  Hardware  (names match your robot config exactly)
    // ═══════════════════════════════════════════════════════
    private DcMotorEx launchMotor;
    private DcMotorEx frontIntake;
    private CRServo   servoIntake;
    private Servo     servoGate;
    private DcMotorEx leftExtension;
    private DcMotorEx rightExtension;

    // ═══════════════════════════════════════════════════════
    //  Launcher PID  (ported 1:1 from launchtest.java)
    // ═══════════════════════════════════════════════════════
    private double kP            = 0.006;
    private double kD            = 0.00002;
    private double kF            = 0.00043;
    private double offsetF       = 0.0;
    private double previousError = 0.0;
    private double lastLoopTime  = 0.0;

    // ═══════════════════════════════════════════════════════
    //  RPM settings
    //  distanceToRPM(d) = 10.63*d + 2197  (your teleop formula)
    //  Set SCORE_DISTANCE to the measured inches from scorePose to goal.
    // ═══════════════════════════════════════════════════════
    private static final double SCORE_DISTANCE = 49.0; // measured ~49 inches from scorePose to goal
    private static final double IDLE_RPM       = 1500;
    private double rpmTarget = IDLE_RPM;

    // ═══════════════════════════════════════════════════════
    //  Gate / Intake constants  (ported from launchtest.java)
    // ═══════════════════════════════════════════════════════
    private static final double GATE_CLOSED  = 0.86;
    private static final double GATE_OPEN    = 1.0;
    private static final double PUSH_TIME    = 2.5;  // seconds per ball push — increased to help second ball clear ramp
    private static final double INTAKE_POWER = 1.0;  // collection power
    private static final double INTAKE_FEED  = 0.7;  // feed power while shooting
    private static final double INTAKE_DWELL = 1.2;  // seconds to dwell at row end
    private static final double SWEEP_SPEED  = 0.35; // max follower power during ball sweep — slow enough to intake not push
    private static final double APPROACH_SPEED = 0.6; // max power approaching the row — gives odometry time to correct
    private static final double FULL_SPEED   = 1.0;  // full power for all other paths

    // ═══════════════════════════════════════════════════════
    //  Shoot sub-state machine
    //  Mirrors the IntakeState enum in launchtest.java exactly.
    //  Runs entirely inside runShootSequence() — the outer path
    //  state machine just calls it each tick and waits for true.
    // ═══════════════════════════════════════════════════════
    private enum ShootState { IDLE, FIRST_BALL, RECOVERY, SECOND_BALL, DONE }
    private ShootState shootState = ShootState.IDLE;
    private ElapsedTime ballTimer     = new ElapsedTime();
    private ElapsedTime recoveryTimer = new ElapsedTime();

    // ═══════════════════════════════════════════════════════
    //  Linear slide constants  (ported from launchtest.java)
    //  Slides stay RETRACTED the entire auto — they never extend.
    //  syncSlides() still runs every tick to hold the plate up.
    // ═══════════════════════════════════════════════════════
    private static final int    SLIDE_RETRACTED  = 0;
    private static final double SLIDE_MOVE_POWER = 0.9;
    private static final double SLIDE_HOLD_POWER = 0.4;
//    private static final int    SYNC_TOLERANCE   = 25;

    // ═══════════════════════════════════════════════════════
    //  Field poses  (exact values from your Pedro Pathing visualizer)
    // ═══════════════════════════════════════════════════════
    private final Pose startPose     = new Pose(29.000, 127.000, Math.toRadians(135));
    private final Pose scorePose     = new Pose(56.000, 100.000, Math.toRadians(135));
    private final Pose row1RightPose = new Pose(66.035, 83.583,  Math.toRadians(180));
    private final Pose row1LeftPose  = new Pose(27.791, 83.661,  Math.toRadians(180));
    private final Pose row2RightPose = new Pose(65.409, 60.209,  Math.toRadians(180));
    private final Pose row2LeftPose  = new Pose(32.052, 60.209,  Math.toRadians(180));

    // ═══════════════════════════════════════════════════════
    //  Paths  (7 paths, matching your visualizer export exactly)
    // ═══════════════════════════════════════════════════════
    private PathChain Path1; // Start        → Score          linear  135→135
    private PathChain Path2; // Score        → Row1 Right     linear  135→180
    private PathChain Path3; // Row1 Right   → Row1 Left      tangential
    private PathChain Path4; // Row1 Left    → Score          curve   180→135
    private PathChain Path5; // Score        → Row2 Right     linear  135→180
    private PathChain Path6; // Row2 Right   → Row2 Left      linear  180→180
    private PathChain Path7; // Row2 Left    → Score          curve   180→135

    // ═══════════════════════════════════════════════════════════════════════
    //  buildPaths()
    // ═══════════════════════════════════════════════════════════════════════
    public void buildPaths() {

        Path1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(135))
                .build();

        Path2 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, row1RightPose))
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                .build();

        Path3 = follower.pathBuilder()
                .addPath(new BezierLine(row1RightPose, row1LeftPose))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        Path4 = follower.pathBuilder()
                .addPath(new BezierLine(row1LeftPose, scorePose))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                .build();

        Path5 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, row2RightPose))
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                .build();

        Path6 = follower.pathBuilder()
                .addPath(new BezierLine(row2RightPose, row2LeftPose))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        Path7 = follower.pathBuilder()
                .addPath(new BezierLine(row2LeftPose, scorePose))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                .build();
    }

    // ═══════════════════════════════════════════════════════════════════════
    //  beginShootSequence()
    //  Call ONCE when entering a shoot state. Resets the sub-state machine,
    //  sets the correct RPM target, and ensures everything starts clean.
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
    //  runShootSequence()
    //  Call every loop() tick while inside a shoot state.
    //  Mirrors the IntakeState switch in launchtest.java exactly:
    //
    //  IDLE        → wait for actualRPM > 90% target
    //                → servoIntake=1, frontIntake=0.7, gate OPEN, reset ballTimer
    //                → FIRST_BALL
    //  FIRST_BALL  → wait for ballTimer > PUSH_TIME
    //                → servoIntake=1, frontIntake=1, gate CLOSED, reset recoveryTimer
    //                → RECOVERY
    //  RECOVERY    → wait for actualRPM > 95% target
    //                → reset ballTimer
    //                → SECOND_BALL
    //  SECOND_BALL → servoIntake=1, frontIntake=0.7, gate OPEN
    //                → wait for ballTimer > PUSH_TIME
    //                → servoIntake=0, frontIntake=0, gate CLOSED
    //                → DONE
    //  DONE        → returns true (outer state machine advances)
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
                // Hold gate open and keep feeding until push time elapses
                servoGate.setPosition(GATE_OPEN);
                servoIntake.setPower(1.0);
                frontIntake.setPower(INTAKE_FEED);
                if (ballTimer.seconds() > PUSH_TIME) {
                    shootState = ShootState.RECOVERY;
                    servoIntake.setPower(1.0);   // keep spinning so next ball loads
                    frontIntake.setPower(INTAKE_POWER);
                    servoGate.setPosition(GATE_CLOSED);
                    recoveryTimer.reset();
                }
                break;

            case RECOVERY:
                // Keep intake running while launcher recovers RPM
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
                return true; // signal to outer state machine to advance
        }
        return false;
    }

    // ═══════════════════════════════════════════════════════════════════════
    //  autonomousPathUpdate()  — THE OUTER STATE MACHINE
    //
    //  runLauncher(dt) and syncSlides() are called in loop() BEFORE this
    //  so the launcher PID and slide hold run every tick no matter what.
    //
    //  ┌───────┬──────────────────────────────────────────────────────────┐
    //  │ State │ Action                                                   │
    //  ├───────┼──────────────────────────────────────────────────────────┤
    //  │   0   │ Set shot RPM, intake off, start Path1                    │
    //  │   1   │ DRIVING Path1 — launcher ramping                         │
    //  │       │  → isBusy=false → beginShootSequence → State 2          │
    //  │   2   │ SHOOT #1 — 2 preloaded balls via runShootSequence()      │
    //  │       │  → done → idle RPM, intake ON, start Path2 → State 3    │
    //  │   3   │ DRIVING Path2 — intake ON approaching row 1              │
    //  │       │  → isBusy=false → start Path3 → State 4                 │
    //  │   4   │ DRIVING Path3 — intake sweeping across row 1             │
    //  │       │  → isBusy=false → State 5                               │
    //  │   5   │ DWELL at row1Left — intake collecting for INTAKE_DWELL s │
    //  │       │  → timer done → stop intake, set RPM, start Path4 → 6   │
    //  │   6   │ DRIVING Path4 curve — launcher ramping                   │
    //  │       │  → isBusy=false → beginShootSequence → State 7          │
    //  │   7   │ SHOOT #2 — 2 row-1 balls via runShootSequence()          │
    //  │       │  → done → idle RPM, intake ON, start Path5 → State 8    │
    //  │   8   │ DRIVING Path5 — intake ON approaching row 2              │
    //  │       │  → isBusy=false → start Path6 → State 9                 │
    //  │   9   │ DRIVING Path6 — intake sweeping across row 2             │
    //  │       │  → isBusy=false → State 10                              │
    //  │  10   │ DWELL at row2Left — intake collecting for INTAKE_DWELL s │
    //  │       │  → timer done → stop intake, set RPM, start Path7 → 11  │
    //  │  11   │ DRIVING Path7 curve — launcher ramping                   │
    //  │       │  → isBusy=false → beginShootSequence → State 12         │
    //  │  12   │ SHOOT #3 — 2 row-2 balls via runShootSequence()          │
    //  │       │  → done → idle RPM → State -1                           │
    //  │  -1   │ DONE — intake off, gate closed, launcher idles           │
    //  └───────┴──────────────────────────────────────────────────────────┘
    // ═══════════════════════════════════════════════════════════════════════
    public void autonomousPathUpdate() {
        switch (pathState) {

            // ─────────────────────────────────────────────
            //  SHOT 1: drive to score, shoot 2 preloads
            // ─────────────────────────────────────────────

            case 0: // Set RPM, clear intake, start Path1
                rpmTarget     = distanceToRPM(SCORE_DISTANCE);
                previousError = 0;
                servoGate.setPosition(GATE_CLOSED);
                frontIntake.setPower(0);
                servoIntake.setPower(0);
                follower.setMaxPower(FULL_SPEED);
                follower.followPath(Path1);
                setPathState(1);
                break;

            case 1: // Driving Path1 — launcher PID ramping in background
                if (!follower.isBusy()) {
                    beginShootSequence();
                    setPathState(2);
                }
                break;

            case 2: // Shoot 2 preloaded balls
                if (runShootSequence()) {
                    rpmTarget     = IDLE_RPM;
                    previousError = 0;
                    frontIntake.setPower(INTAKE_POWER);  // ON before path starts
                    servoIntake.setPower(1.0);
                    follower.setMaxPower(APPROACH_SPEED); // slow approach so odometry stays accurate
                    follower.followPath(Path2, true);
                    setPathState(3);
                }
                break;

            // ─────────────────────────────────────────────
            //  INTAKE ROW 1: approach, sweep, dwell
            // ─────────────────────────────────────────────

            case 3: // Driving Path2 — intake spinning, approaching row 1 right
                frontIntake.setPower(INTAKE_POWER);
                servoIntake.setPower(1.0);
                if (!follower.isBusy()) {
                    follower.setMaxPower(SWEEP_SPEED); // slow sweep so intake grabs balls instead of pushing them
                    follower.followPath(Path3, true);
                    setPathState(4);
                }
                break;

            case 4: // Driving Path3 — intake sweeping across row 1
                frontIntake.setPower(INTAKE_POWER);
                servoIntake.setPower(1.0);
                if (!follower.isBusy()) {
                    setPathState(5);
                }
                break;

            case 5: // Dwell at row1Left — extra collection time
                frontIntake.setPower(INTAKE_POWER);
                servoIntake.setPower(1.0);
                if (pathTimer.getElapsedTimeSeconds() > INTAKE_DWELL) {
                    frontIntake.setPower(0);
                    servoIntake.setPower(0);
                    servoGate.setPosition(GATE_CLOSED);
                    rpmTarget     = distanceToRPM(SCORE_DISTANCE);
                    previousError = 0;
                    follower.setMaxPower(FULL_SPEED); // restore full speed for return drive
                    follower.followPath(Path4, true);
                    setPathState(6);
                }
                break;

            // ─────────────────────────────────────────────
            //  SHOT 2: curve back to score, shoot row-1 balls
            // ─────────────────────────────────────────────

            case 6: // Driving Path4 curve — launcher ramping during arc
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
                    follower.setMaxPower(APPROACH_SPEED); // slower approach — reduces compounded odometry error
                    follower.followPath(Path5, true);
                    setPathState(8);
                }
                break;

            // ─────────────────────────────────────────────
            //  INTAKE ROW 2: approach, sweep, dwell
            // ─────────────────────────────────────────────

            case 8: // Driving Path5 — intake spinning, approaching row 2 right
                frontIntake.setPower(INTAKE_POWER);
                servoIntake.setPower(1.0);
                if (!follower.isBusy()) {
                    follower.setMaxPower(SWEEP_SPEED); // slow sweep so intake grabs balls instead of pushing them
                    follower.followPath(Path6, true);
                    setPathState(9);
                }
                break;

            case 9: // Driving Path6 — intake sweeping across row 2
                frontIntake.setPower(INTAKE_POWER);
                servoIntake.setPower(1.0);
                if (!follower.isBusy()) {
                    setPathState(10);
                }
                break;

            case 10: // Dwell at row2Left — extra collection time
                frontIntake.setPower(INTAKE_POWER);
                servoIntake.setPower(1.0);
                if (pathTimer.getElapsedTimeSeconds() > INTAKE_DWELL) {
                    frontIntake.setPower(0);
                    servoIntake.setPower(0);
                    servoGate.setPosition(GATE_CLOSED);
                    rpmTarget     = distanceToRPM(SCORE_DISTANCE);
                    previousError = 0;
                    follower.setMaxPower(FULL_SPEED); // restore full speed for return drive
                    follower.followPath(Path7, true);
                    setPathState(11);
                }
                break;

            // ─────────────────────────────────────────────
            //  SHOT 3: curve back to score, shoot row-2 balls
            // ─────────────────────────────────────────────

            case 11: // Driving Path7 curve — launcher ramping during arc
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

            // ─────────────────────────────────────────────
            //  DONE
            // ─────────────────────────────────────────────

            case -1:
                frontIntake.setPower(0);
                servoIntake.setPower(0);
                servoGate.setPosition(GATE_CLOSED);
                // Launcher keeps idling at IDLE_RPM via runLauncher() — no hard stop
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
    //  runLauncher()  — called every loop() tick before the state machine
    //  PID holds the launcher at rpmTarget at all times.
    //  rpmTarget is IDLE_RPM during driving/intake and shot RPM during scoring.
    // ═══════════════════════════════════════════════════════════════════════
    private void runLauncher(double dt) {
        double velocityTarget = (rpmTarget / 60.0) * 28.0;
        double actualVelocity = launchMotor.getVelocity();
        double actualRPM      = (actualVelocity * 60.0) / 28.0;
        launchMotor.setPower(updatePDF(velocityTarget, actualRPM, dt));
        telemetry.addData("RPM Target",   "%.0f", rpmTarget);
        telemetry.addData("RPM Actual",   "%.0f", actualRPM);
    }

    // ═══════════════════════════════════════════════════════════════════════
    //  syncSlides()  — called every loop() tick before the state machine
    //  Ported 1:1 from launchtest.java.
    //  Solves two problems:
    //  1. GRAVITY HOLD — applies SLIDE_HOLD_POWER when at target so the plate
    //     never sags under its own weight.
    //  2. SYNC — if the two motors drift apart by more than SYNC_TOLERANCE
    //     ticks, the leading motor is reduced and the lagging motor is boosted
    //     to pull them back together continuously.
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
    //  updatePDF()  — ported 1:1 from launchtest.java
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
    //  distanceToRPM()  — ported 1:1 from launchtest.java
    // ═══════════════════════════════════════════════════════════════════════
    private double distanceToRPM(double distanceInches) {
        return (10.63 * distanceInches) + 2197;
    }

    // ═══════════════════════════════════════════════════════════════════════
    //  emergencyStop()
    //  Hard-stops every motor and servo. Called when the 30-second timer
    //  expires so the robot is guaranteed to be still when teleop begins.
    //  Also sets pathState to -1 so the state machine does nothing if
    //  loop() somehow continues to run after the cutoff.
    // ═══════════════════════════════════════════════════════════════════════
    private void emergencyStop() {
        // Stop drivetrain via Pedro (sets all wheel powers to 0)
        follower.breakFollowing();

        // Stop launcher
        launchMotor.setPower(0);

        // Stop intake
        frontIntake.setPower(0);
        servoIntake.setPower(0);
        servoGate.setPosition(GATE_CLOSED);

        // Slides: keep hold power so plate doesn't drop at the end of auto
        leftExtension.setPower(SLIDE_HOLD_POWER);
        rightExtension.setPower(SLIDE_HOLD_POWER);

        // Lock state machine so nothing restarts
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

        // ── Hardware map ──────────────────────────────────────────────────
        launchMotor    = hardwareMap.get(DcMotorEx.class, "launch_motor");
        frontIntake    = hardwareMap.get(DcMotorEx.class, "front_intake");
        servoIntake    = hardwareMap.get(CRServo.class,   "servo_intake");
        servoGate      = hardwareMap.get(Servo.class,     "servo_gate");
        leftExtension  = hardwareMap.get(DcMotorEx.class, "left_extension");
        rightExtension = hardwareMap.get(DcMotorEx.class, "right_extension");

        // ── Launcher (matches launchtest.java) ────────────────────────────
        launchMotor.setDirection(DcMotor.Direction.FORWARD);
        launchMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // ── Intake ────────────────────────────────────────────────────────
        frontIntake.setDirection(DcMotor.Direction.REVERSE);
        frontIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        servoGate.setPosition(GATE_CLOSED);

        // ── Slides (matches launchtest.java) ──────────────────────────────
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
//        // Hold plate immediately during init so it never sags
//        leftExtension.setPower(SLIDE_HOLD_POWER);
//        rightExtension.setPower(SLIDE_HOLD_POWER);

        // ── PedroPathing ──────────────────────────────────────────────────
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);

        telemetry.addData("Status", "Initialized — 6-ball auto ready");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        // Verify start pose alignment on Driver Hub before pressing play
//        syncSlides(); // keep holding plate during init_loop too
        telemetry.addData("X (should be ~29)",   "%.2f", follower.getPose().getX());
        telemetry.addData("Y (should be ~127)",  "%.2f", follower.getPose().getY());
        telemetry.addData("H (should be ~135°)", "%.1f°", Math.toDegrees(follower.getPose().getHeading()));
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
        // ── 30-second hard stop ───────────────────────────────────────────
        // FTC autonomous is 30 seconds. If we are still running at 29.5s,
        // kill everything immediately so the robot is dead still at the
        // moment the teleop period begins.
        if (opmodeTimer.seconds() >= 29.5) {
            emergencyStop();
            telemetry.addData("STATUS", ">>> AUTO COMPLETE — ALL STOPPED <<<");
            telemetry.update();
            return; // skip the rest of loop() entirely
        }

        // ── Delta time for PD derivative ──────────────────────────────────
        double now = opmodeTimer.seconds();
        double dt  = now - lastLoopTime;
        lastLoopTime = now;

        // ── These three MUST run every single tick ─────────────────────────
        follower.update();        // Pedro path following
        runLauncher(dt);          // launcher PID — idle or shot speed
//        syncSlides();             // slide gravity hold + sync

        // ── State machine ──────────────────────────────────────────────────
        autonomousPathUpdate();

        // ── Driver Hub ─────────────────────────────────────────────────────
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
        // Slides keep their last power so plate doesn't drop on stop
    }
}