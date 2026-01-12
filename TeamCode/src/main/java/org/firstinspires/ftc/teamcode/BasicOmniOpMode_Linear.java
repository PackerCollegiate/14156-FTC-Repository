//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.vision.VisionPortal;
//import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
//import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
//
//import java.util.List;
//
//@TeleOp(name="AprilTag RBE + Fullscreen Stream", group="Linear OpMode")
//@Disabled
//public class BasicOmniOpMode_Linear extends LinearOpMode {
//
//    private VisionPortal visionPortal;
//    private AprilTagProcessor aprilTag;
//
//    @Override
//    public void runOpMode() {
//
//        // --- Initialize AprilTag processor ---
//        aprilTag = AprilTagProcessor.easyCreateWithDefaults();
//
//        visionPortal = new VisionPortal.Builder()
//                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
//                .addProcessor(aprilTag)
//                .build();
//        visionPortal.start();
//
//
//        telemetry.addLine("Initialized! Press START to begin.");
//        telemetry.update();
//
//        waitForStart();
//
//        while (opModeIsActive()) {
//
//            // Get the nearest tag
//            AprilTagDetection nearest = getNearestTag();
//
//            telemetry.clear();
//
//            if (nearest != null && nearest.ftcPose != null) {
//                telemetry.addLine("Nearest AprilTag:");
//                telemetry.addData("ID", nearest.id);
//                telemetry.addData("Range (in)", nearest.ftcPose.range);
//                telemetry.addData("Bearing (deg)", nearest.ftcPose.bearing);
//                telemetry.addData("Elevation (deg)", nearest.ftcPose.elevation);
//            } else {
//                telemetry.addLine("No AprilTag detected.");
//            }
//
//            telemetry.update();
//
//            sleep(50);
//        }
//
//        if (visionPortal != null) {
//            visionPortal.close();
//        }
//    }
//
//    /** Returns the visible AprilTag with the smallest range (closest) */
//    private AprilTagDetection getNearestTag() {
//        List<AprilTagDetection> detections = aprilTag.getDetections();
//        AprilTagDetection best = null;
//        double minRange = Double.MAX_VALUE;
//
//        if (detections != null) {
//            for (AprilTagDetection d : detections) {
//                if (d != null && d.ftcPose != null && d.ftcPose.range < minRange) {
//                    minRange = d.ftcPose.range;
//                    best = d;
//                }
//            }
//        }
//        return best;
//    }
//}
