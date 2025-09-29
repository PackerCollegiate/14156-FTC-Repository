package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name="Basic: Omni Linear OpMode (AprilTag RBE + Aim)", group="Linear OpMode")
public class BasicOmniOpMode_Linear extends LinearOpMode {

    // --- Drive hardware ---
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor leftSlides = null;
    private DcMotor rightSlides = null;

    // --- Vision / AprilTag ---
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    // --- D-Pad edge detection ---
    private boolean lastDpadDown = false;
    private boolean lastDpadUp = false;

    // --- Target tag memory (set when D-Pad Down is pressed) ---
    private Integer targetTagId = null;

    // --- Aiming behavior constants ---
    private static final double MIN_TURN_POWER = 0.10;   // don’t stall
    private static final double MAX_TURN_POWER = 0.50;   // cap turning speed
    private static final double MAX_BEARING_FOR_SCALE = 30.0; // deg; >30° treated as 30 for scaling
    private static final double AIM_TOLERANCE_DEG = 2.0; // stop turning when within ±2°
    // If turning is backwards, flip this to -1:
    private static final int ROTATE_SIGN = +1;

    @Override
    public void runOpMode() {

        // ----- Hardware map -----
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive   = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive  = hardwareMap.get(DcMotor.class, "right_back_drive");
        leftSlides      = hardwareMap.get(DcMotor.class, "left_slides");
        rightSlides     = hardwareMap.get(DcMotor.class, "right_slides");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        leftSlides.setDirection(DcMotor.Direction.FORWARD);
        rightSlides.setDirection(DcMotor.Direction.FORWARD);

        // ----- Vision init -----
        initAprilTagWebcam1();

        telemetry.addData("Status", "Initialized (press START)");
        telemetry.addLine("D-Pad Down: log nearest tag RBE");
        telemetry.addLine("D-Pad Up (hold): aim/rotate toward last tag");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            // ------------------- Read gamepad for driving -------------------
            double axial   = -gamepad1.left_stick_y;  // forward/back
            double lateral =  gamepad1.left_stick_x;  // strafe
            double yaw     =  gamepad1.right_stick_x; // rotate (manual)

            // ------------------- D-Pad edge detection -------------------
            boolean currentDpadDown = gamepad1.dpad_down;
            boolean currentDpadUp   = gamepad1.dpad_up;

            // D-Pad DOWN: find nearest tag, log RBE, remember its ID
            if (currentDpadDown && !lastDpadDown) {
                AprilTagDetection nearest = getNearestTag();
                if (nearest != null && nearest.ftcPose != null) {
                    targetTagId = nearest.id; // remember this tag
                    telemetry.addLine(String.format(
                            "[Nearest Tag Stored] ID %d | RBE: %.1f in, %.1f°, %.1f°",
                            nearest.id, nearest.ftcPose.range,
                            nearest.ftcPose.bearing, nearest.ftcPose.elevation));
                } else {
                    telemetry.addLine("No AprilTag detected. Make sure the camera sees a field tag.");
                }
            }

            // D-Pad UP (hold): aim/rotate toward remembered tag using bearing
            if (currentDpadUp) {
                AprilTagDetection target = getCurrentDetectionOfTarget(targetTagId);
                if (target == null) {
                    // if we lost the specific tag, fall back to nearest so it's still useful
                    target = getNearestTag();
                }
                if (target != null && target.ftcPose != null) {
                    double bearing = target.ftcPose.bearing; // degrees; positive = one side, negative = the other
                    double absBearing = Math.abs(bearing);

                    if (absBearing > AIM_TOLERANCE_DEG) {
                        // Auto-scale turn power based on how far off we are
                        double scale = Math.min(absBearing, MAX_BEARING_FOR_SCALE) / MAX_BEARING_FOR_SCALE;
                        double turnPower = MIN_TURN_POWER + (MAX_TURN_POWER - MIN_TURN_POWER) * scale;

                        // Turn direction follows sign of bearing; flip with ROTATE_SIGN if needed
                        yaw = ROTATE_SIGN * Math.signum(bearing) * turnPower;

                        telemetry.addData("Aiming at Tag",
                                "ID %s | bearing=%.1f°, turn=%.2f",
                                (target.metadata != null ? target.metadata.name : String.valueOf(target.id)),
                                bearing, yaw);
                    } else {
                        // Close enough—don’t rotate
                        yaw = 0.0;
                        telemetry.addLine("Aimed: bearing within tolerance.");
                    }
                } else {
                    telemetry.addLine("Aiming: no tag visible.");
                }
            }

            // Update edge detection state
            lastDpadDown = currentDpadDown;
            lastDpadUp   = currentDpadUp;

            // ------------------- Compute mecanum powers -------------------
            double leftFrontPower  = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower   = axial - lateral + yaw;
            double rightBackPower  = axial + lateral - yaw;

            // Normalize to keep within [-1, 1]
            double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));
            if (max > 1.0) {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
            }

            // ------------------- Send power -------------------
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);

            // ------------------- Telemetry -------------------
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Target Tag", targetTagId == null ? "None" : targetTagId);
            telemetry.addData("Front L/R", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  L/R", "%4.2f, %4.2f", leftBackPower, rightBackPower);

            // Optional: show a quick count of current detections
            List<AprilTagDetection> dets = aprilTag.getDetections();
            telemetry.addData("# Tags Visible", dets.size());
            telemetry.update();
        }

        // Optional: free camera after OpMode ends
        if (visionPortal != null) {
            visionPortal.close();
        }
    }

    // ------------------- Helpers -------------------

    private void initAprilTagWebcam1() {
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();
        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"),
                aprilTag
        );
        // If you want DS preview: open 3-dots menu > Camera Stream
    }

    /** Return the visible AprilTag with the smallest range (closest). */
    private AprilTagDetection getNearestTag() {
        List<AprilTagDetection> detections = aprilTag.getDetections();
        AprilTagDetection best = null;
        double minRange = Double.MAX_VALUE;
        for (AprilTagDetection d : detections) {
            if (d != null && d.ftcPose != null) {
                if (d.ftcPose.range < minRange) {
                    minRange = d.ftcPose.range;
                    best = d;
                }
            }
        }
        return best;
    }

    /** If we have a stored tag ID, find that tag in current detections. */
    private AprilTagDetection getCurrentDetectionOfTarget(Integer tagId) {
        if (tagId == null) return null;
        List<AprilTagDetection> detections = aprilTag.getDetections();
        for (AprilTagDetection d : detections) {
            if (d != null && d.id == tagId && d.ftcPose != null) {
                return d;
            }
        }
        return null;
    }
}
