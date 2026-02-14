package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Servo Gate Calibration", group="Calibration")
public class servotest extends LinearOpMode {

    private Servo servoGate = null;
    private double servoPosition = 0.5; // Start in middle
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;
    private boolean lastDpadLeft = false;
    private boolean lastDpadRight = false;

    @Override
    public void runOpMode() {

        // Initialize servo
        servoGate = hardwareMap.get(Servo.class, "servo_gate");
        servoGate.setPosition(servoPosition);

        telemetry.addData("Status", "Initialized!");
        telemetry.addLine("");
        telemetry.addLine("=== SERVO CALIBRATION TOOL ===");
        telemetry.addLine("");
        telemetry.addLine("CONTROLS:");
        telemetry.addLine("DPAD UP    = +0.05 (bigger steps)");
        telemetry.addLine("DPAD DOWN  = -0.05 (bigger steps)");
        telemetry.addLine("DPAD RIGHT = +0.01 (fine tuning)");
        telemetry.addLine("DPAD LEFT  = -0.01 (fine tuning)");
        telemetry.addLine("");
        telemetry.addLine("A = Jump to 0.0 (minimum)");
        telemetry.addLine("B = Jump to 0.5 (middle)");
        telemetry.addLine("X = Jump to 1.0 (maximum)");
        telemetry.addLine("");
        telemetry.addLine("Find your positions:");
        telemetry.addLine("1. CLOSED position (blocks balls)");
        telemetry.addLine("2. OPEN position (lets balls through)");
        telemetry.addLine("");
        telemetry.addLine("Write down the values!");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // Large adjustments (0.05)
            if (gamepad1.dpad_up && !lastDpadUp) {
                servoPosition += 0.05;
                if (servoPosition > 1.0) servoPosition = 1.0;
            }
            if (gamepad1.dpad_down && !lastDpadDown) {
                servoPosition -= 0.05;
                if (servoPosition < 0.0) servoPosition = 0.0;
            }

            // Fine adjustments (0.01)
            if (gamepad1.dpad_right && !lastDpadRight) {
                servoPosition += 0.01;
                if (servoPosition > 1.0) servoPosition = 1.0;
            }
            if (gamepad1.dpad_left && !lastDpadLeft) {
                servoPosition -= 0.01;
                if (servoPosition < 0.0) servoPosition = 0.0;
            }

            // Quick jump buttons
            if (gamepad1.a) {
                servoPosition = 0.0;
            }
            if (gamepad1.b) {
                servoPosition = 0.5;
            }
            if (gamepad1.x) {
                servoPosition = 1.0;
            }

            // Update last button states
            lastDpadUp = gamepad1.dpad_up;
            lastDpadDown = gamepad1.dpad_down;
            lastDpadLeft = gamepad1.dpad_left;
            lastDpadRight = gamepad1.dpad_right;

            // Set servo position
            servoGate.setPosition(servoPosition);

            // Display current position
            telemetry.addData("=== CURRENT POSITION ===", "");
            telemetry.addData("Servo Position", "%.3f", servoPosition);
            telemetry.addLine("");

            // Visual indicator
            telemetry.addLine("Position Range:");
            telemetry.addLine(getPositionBar(servoPosition));
            telemetry.addLine("0.0 ├───────┼───────┤ 1.0");
            telemetry.addLine("   MIN    MID     MAX");
            telemetry.addLine("");

            // Suggestions based on position
            if (servoPosition < 0.3) {
                telemetry.addLine("📍 Low position - might be CLOSED");
            } else if (servoPosition > 0.7) {
                telemetry.addLine("📍 High position - might be OPEN");
            } else {
                telemetry.addLine("📍 Middle range");
            }

            telemetry.addLine("");
            telemetry.addLine("=== RECOMMENDED VALUES ===");
            telemetry.addLine("Once you find positions:");
            telemetry.addLine("GATE_CLOSED = (your closed value)");
            telemetry.addLine("GATE_OPEN = (your open value)");
            telemetry.addLine("");
            telemetry.addLine("Add to your main code:");
            telemetry.addLine("private final double GATE_CLOSED = ?;");
            telemetry.addLine("private final double GATE_OPEN = ?;");

            telemetry.update();
        }
    }

    // Helper method to create visual position bar
    private String getPositionBar(double position) {
        int barLength = 20;
        int markerPos = (int)(position * barLength);

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