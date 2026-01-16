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

@TeleOp(name="LinearOp Robot Main", group="Linear OpMode")
public class launchtest extends LinearOpMode {
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

    private CRServo servoIntake = null;

    private double rpmTarget = 0;

    private double kP = 0.006;//how fast, acceleration
    private double kD = 0.00002;//slow down before gets there
    private double kF = 0.00043;//static friction
    private double offsetF = 0.0;

    private double previousError = 0;

    @Override
    public void runOpMode() {

        initAprilTag();
        launchMotor = hardwareMap.get(DcMotorEx.class, "launch_motor");
        frontLeftDrive = hardwareMap.get(DcMotorEx.class, "front_left_drive");
        backLeftDrive = hardwareMap.get(DcMotorEx.class, "back_left_drive");
        frontRightDrive = hardwareMap.get(DcMotorEx.class, "front_right_drive");
        backRightDrive = hardwareMap.get(DcMotorEx.class, "back_right_drive");
        frontIntake = hardwareMap.get(DcMotorEx.class, "front_intake");
        servoIntake = hardwareMap.get(CRServo.class, "servo_intake");


        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);
        launchMotor.setDirection(DcMotor.Direction.FORWARD);
        launchMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontIntake.setDirection(DcMotor.Direction.REVERSE);


        telemetry.addData("Status", "Initialized!");
        telemetry.update();

        waitForStart();
        runtime.reset();

        float triggerPress = 0;
        double lastTime = runtime.seconds();


        while (opModeIsActive()) {

            double max;
            double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral =  gamepad1.left_stick_x;
            double yaw     =  gamepad1.right_stick_x;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double frontLeftPower  = axial + lateral + yaw;
            double frontRightPower = axial - lateral - yaw;
            double backLeftPower   = axial - lateral + yaw;
            double backRightPower  = axial + lateral - yaw;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
            max = Math.max(max, Math.abs(backLeftPower));
            max = Math.max(max, Math.abs(backRightPower));

            if (max > 1.0) {
                frontLeftPower  /= max;
                frontRightPower /= max;
                backLeftPower   /= max;
                backRightPower  /= max;
            }

            frontLeftDrive.setPower(frontLeftPower);
            frontRightDrive.setPower(frontRightPower);
            backLeftDrive.setPower(backLeftPower);
            backRightDrive.setPower(backRightPower);

            double IntakePower = gamepad1.left_trigger;

            if (IntakePower > 0.8) {
                frontIntake.setPower(0.8);
            } else {
                frontIntake.setPower(0);
            }


            double now = runtime.seconds();
            double dt = now - lastTime;
            lastTime = now;
            double distance;
            double elevation;
            double flatdistance;
            triggerPress = gamepad1.right_trigger;
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            telemetryAprilTag();
            for (AprilTagDetection detection : currentDetections) {
                if (detection != null){
                    distance = currentDetections.get(0).ftcPose.range;
                    elevation = currentDetections.get(0).ftcPose.elevation;
                    flatdistance = (distance * (Math.cos(Math.toRadians(elevation)))+3.5);
                    rpmTarget = distanceToRPM(flatdistance);
                    telemetry.addData("Distance (in)", "%.1f", flatdistance);
                }
            }

            // Read actual RPM
            double velocityTarget = (rpmTarget / 60.0) * 28.0;

            double actualVelocity = launchMotor.getVelocity();
            double actualRPM = (actualVelocity * 60.0) / 28.0;

            // Spool motor
            if (triggerPress > 0.8) {
                double power = updatePDF(velocityTarget, actualRPM, dt);
                launchMotor.setPower(power);
                telemetry.addData("Power", "%.1f", power);
            } else {
                launchMotor.setPower(0);
                previousError = 0;
            }

            telemetry.addData("RPM Target", "%.1f", rpmTarget);
            telemetry.addData("RPM Actual", "%.1f", actualRPM);

            if(actualRPM > 0.95*rpmTarget) {

                servoIntake.setPower(0.8);

            } else {
                servoIntake.setPower(0);
            }



            telemetry.update();
        }
    }

    private void initAprilTag() {
        aprilTag = new AprilTagProcessor.Builder()
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
//                .setTagSize(0.1524) // meters (6 inches)
                .build();


        VisionPortal.Builder builder = new VisionPortal.Builder();

        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        builder.enableLiveView(true);

        builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        builder.setAutoStopLiveView(false);

        builder.addProcessor(aprilTag);

        visionPortal = builder.build();

    }
    private void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            try {
                if (detection != null) {
                    telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
                }
            } catch (Exception e) {
                telemetry.addLine(String.format("Error"));

            }
        }   // end for() loop
    }   // end method telemetryAprilTag()

    private  double distanceToRPM(double d) {
        return (10.63 * d) + 2197;
    }

    private double updatePDF(double targetTicksPerSec, double actualRPM, double dt) {

        // Convert actual RPM → ticks/sec so units match
        double actualTicksPerSec = (actualRPM / 60.0) * 28.0;

        // Error in ticks/sec
        double error = targetTicksPerSec - actualTicksPerSec;

        // --- PID terms ---
        double P = kP * error;

        double D = 0;
        if (dt > 0) {
            D = kD * (error - previousError) / dt;
        }

        // Feedforward based on target velocity
        double F = (kF * targetTicksPerSec) + offsetF;

        previousError = error;

        double output = P + D + F;

        // HARD CLAMP (critical)
        output = Math.max(-1.0, Math.min(1.0, output));

        return output;
    }


}