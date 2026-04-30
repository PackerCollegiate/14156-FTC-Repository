package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;

import java.util.List;

@TeleOp(name="RPM Test", group="Linear OpMode")
public class RPMtest extends LinearOpMode {
    private static final boolean USE_WEBCAM = true;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx launchMotor = null;
    private DcMotorEx frontLeftDrive = null;
    private DcMotorEx frontRightDrive = null;
    private DcMotorEx backLeftDrive = null;
    private DcMotorEx backRightDrive =  null;
    private DcMotorEx frontIntake = null;
    private Servo servoGate = null;
    private CRServo servoIntake = null;
    private double rpmTarget = 0;
    private double kP = 0.006;//how fast, acceleration
    private double kD = 0.00002;//slow down before gets there
    private double kF = 0.00043;//static friction
    private double offsetF = 0.0;
    private double previousError = 0;
    private boolean lastSpoolUp = false;
    private boolean SpoolOn = false;
    private boolean SlideState = false;
    private boolean lastSlide = false;
    private boolean lastPowerUp = false;
    private boolean lastPowerDown = false;
    private double aimingKp = 0.02; //Coefficient for autoAlign, can be modified
    private int TICKS_PER_REVOLUTION = 28;
    private enum IntakeState {
        idle,
        firstBall,
        recovery,
        secondBall
    }
    private IntakeState intakeState = IntakeState.idle;
    private ElapsedTime ballTimer = new ElapsedTime();
    private ElapsedTime recoveryTimer = new ElapsedTime();
    private double pushTime = 2.0; //Seconds for ball to be pushed, can be modified
    private double recoverTime = 1.0; //Servo rest time, can be modified
    private double idleRPM = 1500; //idleRPM setting, can be modified


    @Override
    public void runOpMode() {

        launchMotor = hardwareMap.get(DcMotorEx.class, "launch_motor");
        frontLeftDrive = hardwareMap.get(DcMotorEx.class, "front_left_drive");
        backLeftDrive = hardwareMap.get(DcMotorEx.class, "back_left_drive");
        frontRightDrive = hardwareMap.get(DcMotorEx.class, "front_right_drive");
        backRightDrive = hardwareMap.get(DcMotorEx.class, "back_right_drive");
        frontIntake = hardwareMap.get(DcMotorEx.class, "front_intake");
        servoIntake = hardwareMap.get(CRServo.class, "servo_intake");
        servoGate = hardwareMap.get(Servo.class, "servo_gate");
        servoGate.setPosition(0.86);

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);
        launchMotor.setDirection(DcMotor.Direction.FORWARD);
        launchMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontIntake.setDirection(DcMotor.Direction.REVERSE);

//        leftExtension.setDirection(DcMotor.Direction.REVERSE); //check
//        rightExtension.setDirection(DcMotor.Direction.FORWARD); //check
//
//        leftExtension.setTargetPosition(0);
//        rightExtension.setTargetPosition(0);
//
//        leftExtension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        leftExtension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        leftExtension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rightExtension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        rightExtension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        rightExtension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        leftExtension.setPower(0.4);
//        rightExtension.setPower(0.4);

        telemetry.addData("Status", "Initialized!");
        telemetry.update();

        waitForStart();
        runtime.reset();

        float triggerPress = 0;
        double lastTime = runtime.seconds();


        while (opModeIsActive()) {

            // Driving Control
            double max;
            double axial = -gamepad1.left_stick_y * 0.7;
            double lateral = gamepad1.left_stick_x * 0.85;
            double yaw = gamepad1.right_stick_x * 0.6;

            double frontLeftPower = axial + lateral + yaw;
            double frontRightPower = axial - lateral - yaw;
            double backLeftPower = axial - lateral + yaw;
            double backRightPower = axial + lateral - yaw;

            max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
            max = Math.max(max, Math.abs(backLeftPower));
            max = Math.max(max, Math.abs(backRightPower));

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

            boolean powerUp = gamepad1.dpad_up;
            boolean powerDown = gamepad1.dpad_down;
            triggerPress = gamepad1.right_trigger;

            lastPowerUp = powerUp;
            lastPowerDown = powerDown;

//            TODO --- Telemetry and activating launcher was not working

            if(powerUp && !lastPowerUp) {
                rpmTarget +=50;
            }
            if(powerDown && !lastPowerDown) {
                rpmTarget -=50;
            }

            double velocityTarget = (rpmTarget / 60.0) * TICKS_PER_REVOLUTION;
            double actualVelocity = launchMotor.getVelocity();
            double actualRPM = (actualVelocity * 60.0) / TICKS_PER_REVOLUTION;

            if(triggerPress > 0.8) {
                launchMotor.setPower(velocityTarget);

            }

            telemetry.addData("RPM Target", "%4.1f", rpmTarget);
            telemetry.addData("Actual RPM", "%4.1f", actualRPM);
        }



    }
}