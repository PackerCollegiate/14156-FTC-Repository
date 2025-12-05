/* Copyright (c) 2021 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="Launch", group="Linear OpMode")

public class launchtest extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx launchMotor = null;

//    private boolean increasepower = false;
//
//    private boolean decreasepower = false;

    private boolean increaseD = false;
    private boolean decreaseD = false;

    private double launchPower = 0.5;
    private double distance = 60;

    @Override
    public void runOpMode() {

        launchMotor = hardwareMap.get(DcMotorEx.class, "launch_motor");

        launchMotor.setDirection(DcMotor.Direction.FORWARD);

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        float launchPress = 0;
        while (opModeIsActive()) {

            launchPress = gamepad1.right_trigger;
//            boolean lastincrease = increasepower;
//            boolean lastdecrease = decreasepower;
//            increasepower = gamepad1.dpad_up;
//            decreasepower = gamepad1.dpad_down;

            boolean lastincreaseD = increaseD;
            boolean lastdecreaseD = decreaseD;
            increaseD = gamepad1.dpad_up;
            decreaseD = gamepad1.dpad_down;


//            if(increasepower && !lastincrease) {
//                launchPower = launchPower + 0.02;
//                if(launchPower >= 1) {
//                    launchPower = 1;
//                }
//            }
//
//            if(decreasepower && !lastdecrease) {
//                launchPower = launchPower - 0.02;
//                if(launchPower <= 0){
//                    launchPower = 0;
//                }
//            }

            if (increaseD && !lastincreaseD) {
                distance = distance + 1;
                if (distance <= 30) {
                    distance = 30;
                }
            }

            if (decreaseD && !lastdecreaseD) {
                distance = distance - 1;
            }
        }

        launchPower = 0.3069 * Math.pow(distance, 0.2096);

        double rpm = launchMotor.getVelocity() * 60;

        if (launchPress > 0.8) {
            launchMotor.setPower(launchPower);
        } else {
            launchMotor.setPower(0);
        }

        // Show the elapsed game time and wheel power.
        telemetry.addData("!Status", "Run Time: " + runtime.toString());
        telemetry.addData("Launch Distance", "%3.1f", distance);
        telemetry.addData("Launch Power", "%1.4f", launchPower);
        telemetry.addData("RPM","%4.1f", rpm);
        telemetry.update();
    }


    }
