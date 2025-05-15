package org.firstinspires.ftc.teamcode.manual;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Hugo's Manual", group="Linear OpMode")
@Disabled
public class Manual extends LinearOpMode {

    // Declare OpMode members for each of the 7 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor armDrive = null;
    private DcMotor leftLiftDrive = null;
    private DcMotor rightLiftDrive = null;

    // Declare OpMode members for each of the 2 continuous servos and one standard servo.
    private Servo armServo = null;
    private CRServo intakeServo = null;
    private CRServo outtakeServo = null;

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        armDrive = hardwareMap.get(DcMotor.class, "arm_drive");
        leftLiftDrive = hardwareMap.get(DcMotor.class, "left_lift_drive");
        rightLiftDrive = hardwareMap.get(DcMotor.class, "right_lift_drive");
        armServo = hardwareMap.get(Servo.class, "arm_servo");
        intakeServo = hardwareMap.get(CRServo.class, "intake_servo");
        outtakeServo = hardwareMap.get(CRServo.class, "outtake_servo");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        armDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        leftLiftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        rightLiftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
//        armServo.setDirection(DcMotor.Direction.FORWARD);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftLiftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLiftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        int armEncoderCalibration = armDrive.getCurrentPosition();
        int leftLiftEncoderCalibration = leftLiftDrive.getCurrentPosition();
        int rightLiftEncoderCalibration = rightLiftDrive.getCurrentPosition();
        int armEncoderLimit = 500;
        int liftEncoderLimit = 1000;

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized - Waiting for driver to start");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral =  gamepad1.left_stick_x;
            double yaw     =  gamepad1.right_stick_x;
            int armAdjustedEncoder = armDrive.getCurrentPosition() - armEncoderCalibration;
            int leftLiftAdjustedEncoder = leftLiftDrive.getCurrentPosition() - leftLiftEncoderCalibration;
            int rightLiftAdjustedEncoder = rightLiftDrive.getCurrentPosition() - rightLiftEncoderCalibration;
            int armPosition = 0;
            boolean outtake = false;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower  = 0;
            double rightFrontPower = 0;
            double leftBackPower   = 0;
            double rightBackPower  = 0;
            double armPower = -1;
            double leftLiftPower = 0;
            double rightLiftPower = 0;

            if (gamepad1.right_bumper) {
                if (armAdjustedEncoder < armEncoderLimit && armAdjustedEncoder > 0){
                    armPower = gamepad1.left_stick_y;
                }
            } else if (gamepad1.left_bumper) {
//              Digital stop switches
                if (leftLiftAdjustedEncoder < liftEncoderLimit && leftLiftAdjustedEncoder > 0){
                    leftLiftPower = gamepad1.left_stick_y;
                }
                if (rightLiftAdjustedEncoder < liftEncoderLimit && rightLiftAdjustedEncoder > 0){
                    rightLiftPower = gamepad1.left_stick_y;
                }
            } else {
                leftFrontPower  = axial + lateral + yaw;
                rightFrontPower = axial - lateral - yaw;
                leftBackPower   = axial - lateral + yaw;
                rightBackPower  = axial + lateral - yaw;

                if (Math.abs(leftLiftAdjustedEncoder - rightLiftAdjustedEncoder) >= 10){
                   leftLiftDrive.setTargetPosition((leftLiftAdjustedEncoder + rightLiftAdjustedEncoder)/2);
                   rightLiftDrive.setTargetPosition((leftLiftAdjustedEncoder + rightLiftAdjustedEncoder)/2);
                }
            }

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
            }

            if (gamepad1.dpad_down && gamepad1.dpad_up){
                armPosition = 1;
            } else if (gamepad1.dpad_down){
                armPosition = 0;
            } else if (gamepad1.dpad_up){
                armPosition = 2;
            }

            intakeServo.setPower(0);
            if (armPosition == 0){
                armServo.setPosition(0.2);
            } else if (armPosition == 1) {
                armServo.setPosition(0.5);
            } else if (armPosition == 2) {
                armServo.setPosition(0.7);
                intakeServo.setPower(1);
            }
            // Send calculated power to motors
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);
            armDrive.setPower(armPower);
            leftLiftDrive.setPower(leftLiftPower);
            rightLiftDrive.setPower(rightLiftPower);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.update();
        }
    }
}
