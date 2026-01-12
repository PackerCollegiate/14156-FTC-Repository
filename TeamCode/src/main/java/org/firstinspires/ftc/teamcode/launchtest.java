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

    // Quadratic fit parameters from regression
    private final double A = -0.0000345;
    private final double B = 0.228;
    private final double C = -289.5;

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

        while (opModeIsActive()) {
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
                    flatdistance = distance * (Math.cos(elevation));
                    rpmTarget = distanceToRPM(flatdistance);
                    telemetry.addData("Distance (in)", "%.1f", flatdistance);
                }
            }

            // Push telemetry to the Driver Station.
            telemetry.update();



            // Share the CPU.
            sleep(20);


            // Compute RPM from quadratic model


            // Convert RPM → ticks/sec
            double velocityTarget = (rpmTarget / 60.0) * 28.0;

            // Read actual RPM
            double actualRPM = (launchMotor.getVelocity() * 60.0) / 28.0;

            // Spool motor
            if (triggerPress > 0.8) {
                launchMotor.setVelocity(velocityTarget);
            } else {
                launchMotor.setPower(0);
            }

            telemetry.addData("RPM Target", "%.1f", rpmTarget);
            telemetry.addData("RPM Actual", "%.1f", actualRPM);
            telemetry.update();
        }
    }

    private double distanceToRPM(double d) {
        return((10.63*d) + 2197);
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
}