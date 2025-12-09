package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Launch Test Auto RPM", group="Linear OpMode")
public class launchtest extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx launchMotor = null;

    private boolean incDist = false;
    private boolean decDist = false;

    private double distance = 60;     // inches
    private double rpmTarget = 0;

    // Quadratic fit parameters from regression
    private final double A = -0.0000345;
    private final double B = 0.228;
    private final double C = -289.5;

    @Override
    public void runOpMode() {

        launchMotor = hardwareMap.get(DcMotorEx.class, "launch_motor");
        launchMotor.setDirection(DcMotor.Direction.FORWARD);
        launchMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Status", "Initialized!");
        telemetry.update();

        waitForStart();
        runtime.reset();

        float triggerPress = 0;

        while (opModeIsActive()) {

            triggerPress = gamepad1.right_trigger;

            boolean lastInc = incDist;
            boolean lastDec = decDist;
            incDist = gamepad1.dpad_up;
            decDist = gamepad1.dpad_down;

            // Adjust distance in 0.5 inch increments
            if (incDist && !lastInc) distance += 0.5;
            if (decDist && !lastDec) distance -= 0.5;

            if (distance < 10) distance = 10;
            if (distance > 150) distance = 150;

            // Compute RPM from quadratic model
            rpmTarget = distanceToRPM(distance);

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

            telemetry.addData("Distance (in)", "%.1f", distance);
            telemetry.addData("RPM Target", "%.1f", rpmTarget);
            telemetry.addData("RPM Actual", "%.1f", actualRPM);
            telemetry.update();
        }
    }

    // Invert quadratic curve: solve RPM from distance
    private double distanceToRPM(double d) {
        return((10.63*d) + 2197);
    }
}
