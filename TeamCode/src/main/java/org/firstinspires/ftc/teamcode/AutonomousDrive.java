package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Auto", group="Robot")
public class AutonomousDrive extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx frontLeftDrive = null;
    private DcMotorEx frontRightDrive = null;
    private DcMotorEx backLeftDrive = null;
    private DcMotorEx backRightDrive =  null;


    @Override
    public void runOpMode() {

        frontLeftDrive = hardwareMap.get(DcMotorEx.class, "front_left_drive");
        backLeftDrive = hardwareMap.get(DcMotorEx.class, "back_left_drive");
        frontRightDrive = hardwareMap.get(DcMotorEx.class, "front_right_drive");
        backRightDrive = hardwareMap.get(DcMotorEx.class, "back_right_drive");


        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        waitForStart();
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.3)) {
            frontLeftDrive.setPower(1.0);
            frontRightDrive.setPower(1.0);
            backLeftDrive.setPower(1.0);
            backRightDrive.setPower(1.0);
        }


    }
}
