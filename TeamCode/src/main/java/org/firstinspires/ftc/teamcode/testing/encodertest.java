package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

// ┌─────────────────────────────────────────────────────────────────────────┐
// │  EncoderTest — Linear Slide Calibration                                 │
// │                                                                         │
// │  Behaves exactly like launchtest.java:                                  │
// │  RUN_TO_POSITION + syncSlides() holds the plate against gravity at all  │
// │  times. You increment/decrement a target value to move the slide, and   │
// │  the motor holds at whatever position you stop at.                      │
// │                                                                         │
// │  CONTROLS:                                                              │
// │  Dpad up    → Increase target by STEP (extend plate down)               │
// │  Dpad down  → Decrease target by STEP (retract plate up)                │
// │  Right bumper → Increase STEP size (coarse)                             │
// │  Left bumper  → Decrease STEP size (fine)                               │
// │  X          → Snap target back to 0 (full retract)                      │
// │                                                                         │
// │  Read "Target" on Driver Hub. When the plate is fully extended,         │
// │  paste that number into SLIDE_EXTENDED in launchtest.java.              │
// └─────────────────────────────────────────────────────────────────────────┘

@TeleOp(name = "Encoder Test - Slides", group = "Calibration")
public class encodertest extends LinearOpMode {

    private DcMotorEx leftExtension  = null;
    private DcMotorEx rightExtension = null;

    // Copied exactly from launchtest.java so behavior is identical
    private static final double SLIDE_MOVE_POWER = 0.9;
    private static final double SLIDE_HOLD_POWER = 0.4;
    private static final int    SYNC_TOLERANCE   = 25;

    // How many ticks to increment per dpad press — toggle with bumpers
    private static final int STEP_FINE   = 10;
    private static final int STEP_MEDIUM = 50;
    private static final int STEP_COARSE = 100;

    @Override
    public void runOpMode() {

        // ── Hardware init (identical to launchtest.java) ──────────────────
        leftExtension  = hardwareMap.get(DcMotorEx.class, "left_extension");
        rightExtension = hardwareMap.get(DcMotorEx.class, "right_extension");

        leftExtension.setDirection(DcMotor.Direction.REVERSE);
        rightExtension.setDirection(DcMotor.Direction.FORWARD);

        leftExtension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightExtension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftExtension.setTargetPosition(0);
        rightExtension.setTargetPosition(0);

        leftExtension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightExtension.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftExtension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightExtension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Hold the plate from the moment we init — same as launchtest.java
        leftExtension.setPower(SLIDE_HOLD_POWER);
        rightExtension.setPower(SLIDE_HOLD_POWER);

        telemetry.addLine("Initialized — plate held at 0. Press PLAY.");
        telemetry.update();

        waitForStart();

        int    target   = 0;
        int    step     = STEP_MEDIUM;
        boolean lastUp      = false;
        boolean lastDown    = false;
        boolean lastX       = false;
        boolean lastRB      = false;
        boolean lastLB      = false;

        while (opModeIsActive()) {

            boolean pressUp   = gamepad1.dpad_up;
            boolean pressDown = gamepad1.dpad_down;
            boolean pressX    = gamepad1.x;
            boolean pressRB   = gamepad1.right_bumper;
            boolean pressLB   = gamepad1.left_bumper;

            // ── Step size selection ───────────────────────────────────────
            if (pressRB && !lastRB) {
                if      (step == STEP_FINE)   step = STEP_MEDIUM;
                else if (step == STEP_MEDIUM) step = STEP_COARSE;
                // already at coarse — stay
            }
            if (pressLB && !lastLB) {
                if      (step == STEP_COARSE) step = STEP_MEDIUM;
                else if (step == STEP_MEDIUM) step = STEP_FINE;
                // already at fine — stay
            }

            // ── Target adjustment (rising edge only — one step per press) ─
            if (pressUp && !lastUp) {
                target += step; // extend further
            }
            if (pressDown && !lastDown) {
                target = Math.max(0, target - step); // retract, floor at 0
            }

            // ── X: snap back to fully retracted ──────────────────────────
            if (pressX && !lastX) {
                target = 0;
            }

            // ── Send target to motors, syncSlides() handles all power ──────
            leftExtension.setTargetPosition(target);
            rightExtension.setTargetPosition(target);
            syncSlides();

            // ── Driver Hub ─────────────────────────────────────────────────
            int leftPos  = leftExtension.getCurrentPosition();
            int rightPos = rightExtension.getCurrentPosition();

            telemetry.addLine("=== SLIDE ENCODER CALIBRATION ===");
            telemetry.addLine("");
            telemetry.addData("Target          (ticks)", target);
            telemetry.addData("Left  Actual    (ticks)", leftPos);
            telemetry.addData("Right Actual    (ticks)", rightPos);
            telemetry.addData("Drift L - R     (ticks)", leftPos - rightPos);
            telemetry.addLine("");
            telemetry.addLine(">>> When fully extended, paste Target into SLIDE_EXTENDED <<<");
            telemetry.addLine("");
            telemetry.addData("Step size", step + " ticks  (LB=finer  RB=coarser)");
            telemetry.addLine("");
            telemetry.addLine("Dpad Up   → extend    Dpad Down → retract");
            telemetry.addLine("X         → return to 0");
            telemetry.update();

            // ── Button edge tracking ──────────────────────────────────────
            lastUp   = pressUp;
            lastDown = pressDown;
            lastX    = pressX;
            lastRB   = pressRB;
            lastLB   = pressLB;
        }
    }

    // ── Copied exactly from launchtest.java — no changes ─────────────────────
    private void syncSlides() {
        int leftPos    = leftExtension.getCurrentPosition();
        int rightPos   = rightExtension.getCurrentPosition();
        int target     = leftExtension.getTargetPosition();

        boolean atTarget = Math.abs(leftPos  - target) < 20
                && Math.abs(rightPos - target) < 20;

        double basePower = atTarget ? SLIDE_HOLD_POWER : SLIDE_MOVE_POWER;

        int drift = leftPos - rightPos;
        if (Math.abs(drift) > SYNC_TOLERANCE) {
            double correction = 0.08;
            if (drift > 0) {
                leftExtension.setPower(Math.max(0.0, basePower - correction));
                rightExtension.setPower(Math.min(1.0, basePower + correction));
            } else {
                leftExtension.setPower(Math.min(1.0, basePower + correction));
                rightExtension.setPower(Math.max(0.0, basePower - correction));
            }
        } else {
            leftExtension.setPower(basePower);
            rightExtension.setPower(basePower);
        }
    }
}