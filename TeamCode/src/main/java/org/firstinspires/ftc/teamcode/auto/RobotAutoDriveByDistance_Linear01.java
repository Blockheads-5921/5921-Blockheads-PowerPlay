/* Copyright (c) 2017 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.common.HardwareDrive;

/**
 * This file illustrates the concept of driving a path based on time.
 * The code is structured as a LinearOpMode
 *
 * The code assumes that you do NOT have encoders on the wheels,
 *   otherwise you would use: RobotAutoDriveByEncoder;
 *
 *   The desired path in this example is:
 *   - Drive forward for 3 seconds
 *   - Spin right for 1.3 seconds
 *   - Drive Backward for 1 Second
 *
 *  The code is written in a simple form with no optimizations.
 *  However, there are several ways that this type of sequence could be streamlined,
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Autonomous(name="Robot: Auto Drive By Distance 01", group="Robot")
@Disabled
public class RobotAutoDriveByDistance_Linear01 extends LinearOpMode {

    HardwareDrive robot = new HardwareDrive();

    private ElapsedTime     runtime = new ElapsedTime();
    static final double     FORWARD_SPEED = 0.6;
    static final double     TURN_SPEED    = 0.5;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step 1:  Drive forward
//        robot.lf.setPower(FORWARD_SPEED);
//        robot.rf.setPower(-FORWARD_SPEED);
//        robot.lb.setPower(FORWARD_SPEED);
//        robot.rb.setPower(-FORWARD_SPEED);

        // Step 2:  Spin right
//        robot.lf.setPower(FORWARD_SPEED);
//        robot.rf.setPower(FORWARD_SPEED);
//        robot.lb.setPower(FORWARD_SPEED);
//        robot.rb.setPower(FORWARD_SPEED);


        RobotLog.d("5921","Step4");


        // Step 1:  Drive forward for 3 seconds

        robot.lf.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.lb.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.rf.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.rb.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.lf.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        robot.lb.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        robot.rf.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        robot.rb.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);


        robot.lf.setTargetPosition(5000);
        robot.rf.setTargetPosition(-5000);
        robot.lb.setTargetPosition(5000);
        robot.rb.setTargetPosition(-5000);

        robot.lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lf.setPower(0.5);
        robot.rf.setPower(0.5);
        robot.lb.setPower(0.5);
        robot.rb.setPower(0.5);


        while (opModeIsActive() &&
                // (runtime.seconds() < timeoutS) &&
                ( robot.lf.isBusy())) {
            telemetry.addData("Running to",  " %7d ", 5000);
            telemetry.addData("Currently at",  " at %7d", robot.lf.getCurrentPosition());
            telemetry.update();
            RobotLog.d("Encoders: %7d,%7d,%7d,%7d", robot.lf.getCurrentPosition(), robot.rf.getCurrentPosition(), robot.lb.getCurrentPosition(), robot.rb.getCurrentPosition());
        }


        // Step end:  Stop
        robot.lf.setPower(0);
        robot.rf.setPower(0);
        robot.lb.setPower(0);
        robot.rb.setPower(0);

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);
    }
}
