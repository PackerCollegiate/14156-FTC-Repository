package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;

import java.util.List;

@TeleOp(name="Launch Test Auto RPM", group="Linear OpMode")
public class launchtest extends LinearOpMode {
    private static final boolean USE_WEBCAM = true;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx launchMotor = null;

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
        launchMotor.setDirection(DcMotor.Direction.FORWARD);
        launchMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        telemetry.addData("Status", "Initialized!");
        telemetry.update();

        waitForStart();
        runtime.reset();

        float triggerPress = 0;
        double lastTime = runtime.seconds();


        while (opModeIsActive()) {
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