package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "RPM Test", group = "Linear OpMode")
public class RPMtest extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    private DcMotorEx launchMotor = null;
    private DcMotorEx frontLeftDrive = null;
    private DcMotorEx frontRightDrive = null;
    private DcMotorEx backLeftDrive = null;
    private DcMotorEx backRightDrive = null;
    private DcMotorEx frontIntake = null;

    private Servo servoGate = null;
    private CRServo servoIntake = null;

    // RPM target
    private double rpmTarget = 0;

    // Edge detection
    private boolean lastPowerUp = false;
    private boolean lastPowerDown = false;

    // Ball push timing
    private double pushTime = 2.0;

    // Intake state machine
    private enum IntakeState {
        IDLE,
        FIRST_BALL,
        RECOVERY,
        SECOND_BALL
    }

    private ElapsedTime ballTimer = new ElapsedTime();
    private ElapsedTime recoveryTimer = new ElapsedTime();

    // FIXED: use IntakeState directly
    private IntakeState intakeState = IntakeState.IDLE;

    // Encoder constants
    private static final double TICKS_PER_REVOLUTION = 28.0;

    @Override
    public void runOpMode() {

        // Hardware mapping
        launchMotor = hardwareMap.get(DcMotorEx.class, "launch_motor");

        frontLeftDrive = hardwareMap.get(DcMotorEx.class, "front_left_drive");
        backLeftDrive = hardwareMap.get(DcMotorEx.class, "back_left_drive");
        frontRightDrive = hardwareMap.get(DcMotorEx.class, "front_right_drive");
        backRightDrive = hardwareMap.get(DcMotorEx.class, "back_right_drive");

        frontIntake = hardwareMap.get(DcMotorEx.class, "front_intake");

        servoIntake = hardwareMap.get(CRServo.class, "servo_intake");
        servoGate = hardwareMap.get(Servo.class, "servo_gate");

        // Initial positions
        servoGate.setPosition(0.86);

        // Motor directions
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);

        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        launchMotor.setDirection(DcMotor.Direction.FORWARD);

        frontIntake.setDirection(DcMotor.Direction.REVERSE);

        // Encoder mode
        launchMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Status", "Initialized!");
        telemetry.update();

        waitForStart();

        runtime.reset();

        while (opModeIsActive()) {

            /*
             * =========================
             * DRIVE CONTROL
             * =========================
             */

            double axial = -gamepad1.left_stick_y * 0.7;
            double lateral = gamepad1.left_stick_x * 0.85;
            double yaw = gamepad1.right_stick_x * 0.6;

            double frontLeftPower = axial + lateral + yaw;
            double frontRightPower = axial - lateral - yaw;
            double backLeftPower = axial - lateral + yaw;
            double backRightPower = axial + lateral - yaw;

            double max = Math.max(
                    Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
                    Math.max(Math.abs(backLeftPower), Math.abs(backRightPower))
            );

            if (max > 1.0) {
                frontLeftPower /= max;
                frontRightPower /= max;
                backLeftPower /= max;
                backRightPower /= max;
            }

            frontLeftDrive.setPower(frontLeftPower);
            frontRightDrive.setPower(frontRightPower);
            backLeftDrive.setPower(backLeftPower);
            backRightDrive.setPower(backRightPower);

            /*
             * =========================
             * LAUNCHER RPM
             * =========================
             */

            double velocityTarget = (rpmTarget * TICKS_PER_REVOLUTION) / 60.0;

            double actualVelocity = launchMotor.getVelocity();
            double actualRPM = (actualVelocity * 60.0) / TICKS_PER_REVOLUTION;

            /*
             * =========================
             * INTAKE CONTROLS
             * =========================
             */

            double intakePower = gamepad1.left_trigger;
            boolean eject = gamepad1.left_bumper;

            // FIXED: missing semicolon
            float triggerPress = gamepad1.right_trigger;

            if (intakePower > 0.8) {

                servoIntake.setPower(1.0);
                frontIntake.setPower(1.0);

                intakeState = IntakeState.IDLE;

            } else if (eject) {

                servoIntake.setPower(-1.0);
                frontIntake.setPower(-1.0);

                intakeState = IntakeState.IDLE;

            } else if (triggerPress > 0.8) {

                switch (intakeState) {

                    case IDLE:

                        if (actualRPM > 0.97 * rpmTarget) {

                            intakeState = IntakeState.FIRST_BALL;

                            ballTimer.reset();

//                            servoIntake.setPower(1.0);
//                            frontIntake.setPower(0.7);

                            servoGate.setPosition(1.0);

                        } else {

                            servoIntake.setPower(0);
                            frontIntake.setPower(0);

                            servoGate.setPosition(0.86);
                        }

                        break;

                    case FIRST_BALL:

                        if (ballTimer.seconds() > pushTime) {

                            intakeState = IntakeState.IDLE;

//                            servoIntake.setPower(1.0);
//                            frontIntake.setPower(1.0);

                            servoGate.setPosition(0.86);

                            recoveryTimer.reset();
                        }

                        break;

//                    case RECOVERY:
//
//                        if (actualRPM > 0.95 * rpmTarget) {
//
//                            intakeState = IntakeState.SECOND_BALL;
//
//                            ballTimer.reset();
//                        }
//
//                        break;
//
//                    case SECOND_BALL:
//
//                        servoIntake.setPower(1.0);
//                        frontIntake.setPower(0.7);
//
//                        servoGate.setPosition(1.0);
//
//                        if (ballTimer.seconds() > pushTime) {
//
//                            intakeState = IntakeState.IDLE;
//
//                            servoIntake.setPower(0);
//                            frontIntake.setPower(0);
//
//                            servoGate.setPosition(0.86);
//                        }
//
//                        break;
                }

                telemetry.addData("Intake State", intakeState.toString());

            } else {

                servoIntake.setPower(0);
                frontIntake.setPower(0);

                servoGate.setPosition(0.86);

                intakeState = IntakeState.IDLE;
            }

            /*
             * =========================
             * RPM ADJUSTMENT
             * =========================
             */

            boolean powerUp = gamepad1.dpad_up;
            boolean powerDown = gamepad1.dpad_down;

            // Edge detection
            if (powerUp && !lastPowerUp) {
                rpmTarget += 100;
            }

            if (powerDown && !lastPowerDown) {
                rpmTarget -= 100;
            }

            lastPowerUp = powerUp;
            lastPowerDown = powerDown;

            /*
             * =========================
             * LAUNCH MOTOR CONTROL
             * =========================
             */

            if (gamepad1.right_trigger > 0.8) {
                launchMotor.setVelocity(velocityTarget);
            } else {
                launchMotor.setPower(0);
            }

            /*
             * =========================
             * TELEMETRY
             * =========================
             */

            telemetry.addData("RPM Target", "%.1f", rpmTarget);
            telemetry.addData("Velocity Target", "%.1f", velocityTarget);
            telemetry.addData("Actual Velocity", "%.1f", actualVelocity);
            telemetry.addData("Actual RPM", "%.1f", actualRPM);
            telemetry.addData("Motor Power", "%.2f", launchMotor.getPower());

            telemetry.update();
        }
    }
}