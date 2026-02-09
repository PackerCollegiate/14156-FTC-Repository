package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="Linear Slide Position Finder", group="Calibration")
public class encodertest extends LinearOpMode {

    private DcMotorEx leftExtension = null;
    private DcMotorEx rightExtension = null;
    private int targetPosition = 0;

    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;
    private boolean lastDpadLeft = false;
    private boolean lastDpadRight = false;

    @Override
    public void runOpMode() {

        // Initialize slide motors
        leftExtension = hardwareMap.get(DcMotorEx.class, "left_extension");
        rightExtension = hardwareMap.get(DcMotorEx.class, "right_extension");

        // Set directions (adjust if needed)
        leftExtension.setDirection(DcMotor.Direction.REVERSE);
        rightExtension.setDirection(DcMotor.Direction.FORWARD);

        leftExtension.setTargetPosition(0);
        rightExtension.setTargetPosition(0);
        // Reset encoders to 0
        leftExtension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightExtension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set to RUN_TO_POSITION mode
        leftExtension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightExtension.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set brake mode to hold position
        leftExtension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightExtension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set initial target position to 0 (retracted)
        targetPosition = 0;
        leftExtension.setTargetPosition(targetPosition);
        rightExtension.setTargetPosition(targetPosition);
        leftExtension.setPower(0.6);
        rightExtension.setPower(0.6);

        telemetry.addData("Status", "Initialized!");
        telemetry.addData("Starting Position", targetPosition);
        telemetry.addLine("");
        telemetry.addLine("=== LINEAR SLIDE POSITION FINDER ===");
        telemetry.addLine("");
        telemetry.addLine("CONTROLS:");
        telemetry.addLine("DPAD UP    = +100 ticks (big steps)");
        telemetry.addLine("DPAD DOWN  = -100 ticks (big steps)");
        telemetry.addLine("DPAD RIGHT = +10 ticks (fine tuning)");
        telemetry.addLine("DPAD LEFT  = -10 ticks (fine tuning)");
        telemetry.addLine("");
        telemetry.addLine("A = Jump to 0 (retracted)");
        telemetry.addLine("B = Jump to 500");
        telemetry.addLine("X = Jump to 1000");
        telemetry.addLine("Y = Jump to 1500");
        telemetry.addLine("");
        telemetry.addLine("Find your positions:");
        telemetry.addLine("1. RETRACTED (0)");
        telemetry.addLine("2. LOW position");
        telemetry.addLine("3. MID position");
        telemetry.addLine("4. HIGH position");
        telemetry.addLine("5. MAX safe extension");
        telemetry.addLine("");
        telemetry.addLine("Write down the values!");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // Large adjustments (100 ticks)
            if (gamepad1.dpad_up && !lastDpadUp) {
                targetPosition += 100;
            }
            if (gamepad1.dpad_down && !lastDpadDown) {
                targetPosition -= 100;
                if (targetPosition < 0) targetPosition = 0; // Don't go negative
            }

            // Fine adjustments (10 ticks)
            if (gamepad1.dpad_right && !lastDpadRight) {
                targetPosition += 10;
            }
            if (gamepad1.dpad_left && !lastDpadLeft) {
                targetPosition -= 10;
                if (targetPosition < 0) targetPosition = 0;
            }

            // Quick jump buttons
            if (gamepad1.a) {
                targetPosition = 0;
            }
            if (gamepad1.b) {
                targetPosition = 500;
            }
            if (gamepad1.x) {
                targetPosition = 1000;
            }
            if (gamepad1.y) {
                targetPosition = 1500;
            }

            // Update last button states
            lastDpadUp = gamepad1.dpad_up;
            lastDpadDown = gamepad1.dpad_down;
            lastDpadLeft = gamepad1.dpad_left;
            lastDpadRight = gamepad1.dpad_right;

            // Set target positions
            leftExtension.setTargetPosition(targetPosition);
            rightExtension.setTargetPosition(targetPosition);

            // Get current positions
            int leftPos = leftExtension.getCurrentPosition();
            int rightPos = rightExtension.getCurrentPosition();

            // Display current position
            telemetry.addData("=== TARGET POSITION ===", targetPosition);
            telemetry.addLine("");
            telemetry.addData("Left Slide Current", leftPos);
            telemetry.addData("Right Slide Current", rightPos);

            // Show if motors are moving
            if (Math.abs(leftPos - targetPosition) > 5) {
                telemetry.addData("Status", "MOVING...");
            } else {
                telemetry.addData("Status", "AT TARGET ✓");
            }

            telemetry.addLine("");

            // Visual indicator
            telemetry.addLine("Position Range:");
            telemetry.addLine(getPositionBar(targetPosition, 2000));
            telemetry.addLine("0 ├─────┼─────┼─────┤ 2000");
            telemetry.addLine("");

            // Position suggestions
            if (targetPosition == 0) {
                telemetry.addLine("📍 RETRACTED position");
            } else if (targetPosition < 600) {
                telemetry.addLine("📍 LOW range - good for LOW position");
            } else if (targetPosition < 1200) {
                telemetry.addLine("📍 MID range - good for MID position");
            } else {
                telemetry.addLine("📍 HIGH range - good for HIGH/MAX position");
            }

            telemetry.addLine("");

            // Warning if motors are out of sync
            int difference = Math.abs(leftPos - rightPos);
            if (difference > 50) {
                telemetry.addLine("⚠️ WARNING: Motors out of sync!");
                telemetry.addData("Difference", difference + " ticks");
            }

            telemetry.addLine("");
            telemetry.addLine("=== RECOMMENDED VALUES ===");
            telemetry.addLine("Once you find positions:");
            telemetry.addLine("SLIDE_MIN = 0");
            telemetry.addLine("SLIDE_LOW = (your low value)");
            telemetry.addLine("SLIDE_MID = (your mid value)");
            telemetry.addLine("SLIDE_HIGH = (your high value)");
            telemetry.addLine("SLIDE_MAX = (your max safe value)");
            telemetry.addLine("");
            telemetry.addLine("Add to your main code:");
            telemetry.addLine("private final int SLIDE_MIN = 0;");
            telemetry.addLine("private final int SLIDE_LOW = ?;");
            telemetry.addLine("private final int SLIDE_MID = ?;");
            telemetry.addLine("private final int SLIDE_HIGH = ?;");
            telemetry.addLine("private final int SLIDE_MAX = ?;");

            telemetry.update();
        }

        // Stop motors when OpMode ends
        leftExtension.setPower(0);
        rightExtension.setPower(0);
    }

    // Helper method to create visual position bar
    private String getPositionBar(int position, int maxEstimate) {
        int barLength = 20;
        double ratio = Math.min(1.0, (double)position / maxEstimate);
        int markerPos = (int)(ratio * barLength);

        StringBuilder bar = new StringBuilder("    ");
        for (int i = 0; i < barLength; i++) {
            if (i == markerPos) {
                bar.append("█");
            } else {
                bar.append("─");
            }
        }
        return bar.toString();
    }
}