package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.dfrobot.HuskyLens.Block;

@TeleOp(name="HuskyLens: Mecanum + Auto Aim", group="Linear OpMode")
public class BasicOmniHusky_Linear extends LinearOpMode {

    // Drive motors
    private DcMotor leftFrontDrive, leftBackDrive, rightFrontDrive, rightBackDrive;
    private ElapsedTime runtime = new ElapsedTime();

    // HuskyLens
    private HuskyLens husky;

    @Override
    public void runOpMode() {
        // Initialize drive motors
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive   = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive  = hardwareMap.get(DcMotor.class, "right_back_drive");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        // Initialize HuskyLens on I2C
        husky = hardwareMap.get(HuskyLens.class, "huskylens");
        husky.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        boolean lastDpadDown = false;
        boolean lastDpadUp = false;

        while (opModeIsActive()) {
            double axial   = -gamepad1.left_stick_y; // forward/back
            double lateral =  gamepad1.left_stick_x; // strafe
            double yaw     =  gamepad1.right_stick_x; // rotate

            boolean currentDpadDown = gamepad1.dpad_down;
            boolean currentDpadUp   = gamepad1.dpad_up;

            // D-Pad Down → Log HuskyLens RBE-style data
            if (currentDpadDown && !lastDpadDown) {
                Block target = getLargestBlock(husky);
                if (target != null) {
                    double range = 240 - target.height; // estimate distance
                    double bearing = (target.x - 160) * 0.25; // deg off-center
                    double elevation = (target.y - 120) * 0.25;
                    telemetry.addLine("==== HUSKYLENS TARGET ====");
                    telemetry.addData("Range", "%.1f (sim)", range);
                    telemetry.addData("Bearing", "%.1f deg", bearing);
                    telemetry.addData("Elevation", "%.1f deg", elevation);
                    telemetry.update();
                } else {
                    telemetry.addLine("No target detected");
                    telemetry.update();
                }
            }

            // D-Pad Up → Auto-aim
            if (currentDpadUp) {
                Block target = getLargestBlock(husky);
                if (target != null) {
                    double error = target.x - 160; // 320x240 frame center
                    double kP = 0.004;
                    yaw = -error * kP;
                    telemetry.addData("Auto Aim", "error=%.1f, yaw=%.2f", error, yaw);
                } else {
                    telemetry.addLine("No target to aim at");
                }
            }

            // Mecanum power calculation
            double leftFrontPower  = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower   = axial - lateral + yaw;
            double rightBackPower  = axial + lateral - yaw;

            double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));
            if (max > 1.0) {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
            }

            // Set motor power
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front L/R", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  L/R", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.update();

            lastDpadDown = currentDpadDown;
            lastDpadUp = currentDpadUp;
        }
    }

    /** Returns the largest detected block. */
    private Block getLargestBlock(HuskyLens husky) {
        Block[] blocks = husky.blocks();
        if (blocks == null || blocks.length == 0) return null;

        Block largest = blocks[0];
        for (Block b : blocks) {
            if (b.width * b.height > largest.width * largest.height) {
                largest = b;
            }
        }
        return largest;
    }
}
