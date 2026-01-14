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

    private double kP = 0.5;
    private double kD = 0.00005;
    private double kF = 0.00005;
    private double offsetF = 0.07;

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
                    flatdistance = (distance * (Math.cos(Math.toRadians(elevation)))+2);
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
            if (detection != null){
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            }
        }   // end for() loop
    }   // end method telemetryAprilTag()

    private  double distanceToRPM(double d) {
        return (10.63 * d) + 2197;
    }

    private double updatePDF(double target, double actual, double dt) {

        // Error in ticks/sec
        double error = target - actual;

        // Proportional term
        double P = kP * error;

        // Derivative term (rate of change of error)
        double derivative = 0;
        if (dt > 0) {
            derivative = (error - previousError) / dt;
        }
        double D = kD * derivative;

        // Feedforward:
        // We assume power is roughly linear with required velocity.
        // F is our "best guess" at the power needed for this target speed.
        double F = (kF * target) + offsetF;

        // Save error for next loop
        previousError = error;

        // Total output power
        double output = P + D + F;

        return output;
    }

}