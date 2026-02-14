package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name="Encoder Test - Motor Mapper", group="Calibration")
public class wheeltest extends LinearOpMode {

    private DcMotorEx motor0 = null;
    private DcMotorEx motor1 = null;
    private DcMotorEx motor2 = null;
    private DcMotorEx motor3 = null;

    @Override
    public void runOpMode() {

        // Initialize all motors from Control Hub ports
        try {
            motor0 = hardwareMap.get(DcMotorEx.class, "front_left_drive");
        } catch (Exception e) {
            telemetry.addLine("Motor 0 (front_left_drive) NOT FOUND");
        }

        try {
            motor1 = hardwareMap.get(DcMotorEx.class, "front_right_drive");
        } catch (Exception e) {
            telemetry.addLine("Motor 1 (front_right_drive) NOT FOUND");
        }

        try {
            motor2 = hardwareMap.get(DcMotorEx.class, "back_left_drive");
        } catch (Exception e) {
            telemetry.addLine("Motor 2 (back_left_drive) NOT FOUND");
        }

        try {
            motor3 = hardwareMap.get(DcMotorEx.class, "back_right_drive");
        } catch (Exception e) {
            telemetry.addLine("Motor 3 (back_right_drive) NOT FOUND");
        }

        // Reset all encoders to 0
        if (motor0 != null) motor0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        if (motor1 != null) motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        if (motor2 != null) motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        if (motor3 != null) motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set to run using encoders
        if (motor0 != null) motor0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (motor1 != null) motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (motor2 != null) motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (motor3 != null) motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Status", "Initialized - Ready to Test!");
        telemetry.addLine("");
        telemetry.addLine("INSTRUCTIONS:");
        telemetry.addLine("DPAD UP    = Test Motor 0 (front_left)");
        telemetry.addLine("DPAD RIGHT = Test Motor 1 (front_right)");
        telemetry.addLine("DPAD DOWN  = Test Motor 2 (back_left)");
        telemetry.addLine("DPAD LEFT  = Test Motor 3 (back_right)");
        telemetry.addLine("");
        telemetry.addLine("Press button to spin motor.");
        telemetry.addLine("Watch which wheel spins!");
        telemetry.addLine("Encoder will count up.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // Stop all motors first
            if (motor0 != null) motor0.setPower(0);
            if (motor1 != null) motor1.setPower(0);
            if (motor2 != null) motor2.setPower(0);
            if (motor3 != null) motor3.setPower(0);

            // Test Motor 0 (front_left_drive)
            if (gamepad1.dpad_up && motor0 != null) {
                motor0.setPower(0.5);
                telemetry.addData("TESTING", "Motor 0 - front_left_drive");
                telemetry.addData("Power", "0.5 (50%)");
            }

            // Test Motor 1 (front_right_drive)
            else if (gamepad1.dpad_right && motor1 != null) {
                motor1.setPower(0.5);
                telemetry.addData("TESTING", "Motor 1 - front_right_drive");
                telemetry.addData("Power", "0.5 (50%)");
            }

            // Test Motor 2 (back_left_drive)
            else if (gamepad1.dpad_down && motor2 != null) {
                motor2.setPower(0.5);
                telemetry.addData("TESTING", "Motor 2 - back_left_drive");
                telemetry.addData("Power", "0.5 (50%)");
            }

            // Test Motor 3 (back_right_drive)
            else if (gamepad1.dpad_left && motor3 != null) {
                motor3.setPower(0.5);
                telemetry.addData("TESTING", "Motor 3 - back_right_drive");
                telemetry.addData("Power", "0.5 (50%)");
            }
            else {
                telemetry.addData("STATUS", "No motor running");
                telemetry.addLine("Press DPAD to test motors");
            }

            telemetry.addLine("");
            telemetry.addLine("--- ENCODER POSITIONS ---");

            // Display encoder values
            if (motor0 != null) {
                telemetry.addData("Motor 0 (front_left)", motor0.getCurrentPosition());
            }
            if (motor1 != null) {
                telemetry.addData("Motor 1 (front_right)", motor1.getCurrentPosition());
            }
            if (motor2 != null) {
                telemetry.addData("Motor 2 (back_left)", motor2.getCurrentPosition());
            }
            if (motor3 != null) {
                telemetry.addData("Motor 3 (back_right)", motor3.getCurrentPosition());
            }

            telemetry.addLine("");
            telemetry.addLine("Press Y to RESET all encoders");

            // Reset encoders with Y button
            if (gamepad1.y) {
                if (motor0 != null) motor0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                if (motor1 != null) motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                if (motor2 != null) motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                if (motor3 != null) motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                if (motor0 != null) motor0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                if (motor1 != null) motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                if (motor2 != null) motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                if (motor3 != null) motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                telemetry.addLine(">>> ENCODERS RESET <<<");
            }

            telemetry.update();
        }
    }
}