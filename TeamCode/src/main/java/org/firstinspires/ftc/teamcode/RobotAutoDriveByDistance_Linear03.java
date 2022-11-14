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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

/**
The Purpose of this Program is to run calibration on Forward, Reverse, Spin and Strafe
 */

@Autonomous(name="Robot: Auto Drive By Distance 05", group="Robot")
@Disabled
public class RobotAutoDriveByDistance_Linear03 extends LinearOpMode {

    HardwareDrive robot = new HardwareDrive();

    private ElapsedTime     runtime = new ElapsedTime();

// Unused, delete later?
//   static final double     FORWARD_SPEED = 0.6;
//   static final double     TURN_SPEED    = 0.5;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        RobotLog.d("5921","Step4");

    // Calibrate Forward and Reverse
//        //DriveForward(524*10,0.35); // 47.5 cnts/inch, old - 570, new 43.651 cnts/inch
        DriveForward(1045,0.35); // 1 pad
//        DriveForward(1017+1017,0.35); // 2 pad
//        DriveForward(1179+1017*2,0.35); // 3 pad
//        DriveForward(1179+1017*3,0.35); // 4 pad
//        DriveForward(1179+1017*4,0.35); // 5 pad
        DriveStop(0);
        sleep(3000);
        DriveReverse(1045,0.35);
        DriveStop(0);
        sleep(500);

    // Calibrate Right and Left
//        SpinRight(4*920,0.25); // 76.67 cnts/inch
//        DriveStop(0);
//        sleep(3000);
//        SpinLeft(4*920,0.25);
//        DriveStop(0);
//        sleep(500);

    // Calibrate StrafeRight and StrafeLeft
//        StrafeRight(595*10,0.25); // 45.83 cnts/inch, old-550, new 49.549 cnts/inch
//        DriveStop(0);
//        sleep(3000);
//        StrafeLeft(595*10,0.25);
//        DriveStop(0);
//        sleep(500);

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);
    }

    // StrafeRight Function
    private void StrafeRight(int straferight_encoder_pulses, double drive_power) {
        robot.lf.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.lb.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.rf.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.rb.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.lf.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        robot.lb.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        robot.rf.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        robot.rb.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        robot.lf.setTargetPosition(-straferight_encoder_pulses);
        robot.rf.setTargetPosition(-straferight_encoder_pulses);
        robot.lb.setTargetPosition(straferight_encoder_pulses);
        robot.rb.setTargetPosition(straferight_encoder_pulses);

        robot.lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lf.setPower(drive_power);
        robot.rf.setPower(drive_power);
        robot.lb.setPower(drive_power);
        robot.rb.setPower(drive_power);


        while (opModeIsActive() &&
                // (runtime.seconds() < timeoutS) &&
                (robot.lf.isBusy())) {
            telemetry.addData("Running to", " %7d ", straferight_encoder_pulses);
            telemetry.addData("Currently at", " at %7d", robot.lf.getCurrentPosition());
            telemetry.update();
            RobotLog.d("StrafeRight: Encoders: %7d,%7d,%7d,%7d", robot.lf.getCurrentPosition(), robot.rf.getCurrentPosition(), robot.lb.getCurrentPosition(), robot.rb.getCurrentPosition());
        }
    }

    // StrafeLeft Function
    private void StrafeLeft(int stafeleft_encoder_pulses, double drive_power) {

        robot.lf.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.lb.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.rf.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.rb.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.lf.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        robot.lb.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        robot.rf.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        robot.rb.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        robot.lf.setTargetPosition(stafeleft_encoder_pulses);
        robot.rf.setTargetPosition(stafeleft_encoder_pulses);
        robot.lb.setTargetPosition(-stafeleft_encoder_pulses);
        robot.rb.setTargetPosition(-stafeleft_encoder_pulses);

        robot.lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lf.setPower(drive_power);
        robot.rf.setPower(drive_power);
        robot.lb.setPower(drive_power);
        robot.rb.setPower(drive_power);

        while (opModeIsActive() &&
                // (runtime.seconds() < timeoutS) &&
                (robot.lf.isBusy())) {
            telemetry.addData("Running to", " %7d ", stafeleft_encoder_pulses);
            telemetry.addData("Currently at", " at %7d", robot.lf.getCurrentPosition());
            telemetry.update();
            RobotLog.d("StrafeLeft: Encoders: %7d,%7d,%7d,%7d", robot.lf.getCurrentPosition(), robot.rf.getCurrentPosition(), robot.lb.getCurrentPosition(), robot.rb.getCurrentPosition());
        }
    }

    // SpinLeft Function
    private void SpinLeft(int spinleft_encoder_pulses, double drive_power) {

        robot.lf.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.lb.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.rf.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.rb.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.lf.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        robot.lb.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        robot.rf.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        robot.rb.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        robot.lf.setTargetPosition(-spinleft_encoder_pulses);
        robot.rf.setTargetPosition(-spinleft_encoder_pulses);
        robot.lb.setTargetPosition(-spinleft_encoder_pulses);
        robot.rb.setTargetPosition(-spinleft_encoder_pulses);

        robot.lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lf.setPower(drive_power);
        robot.rf.setPower(drive_power);
        robot.lb.setPower(drive_power);
        robot.rb.setPower(drive_power);

        while (opModeIsActive() &&
                // (runtime.seconds() < timeoutS) &&
                (robot.lf.isBusy())) {
            telemetry.addData("Running to", " %7d ", spinleft_encoder_pulses);
            telemetry.addData("Currently at", " at %7d", robot.lf.getCurrentPosition());
            telemetry.update();
            RobotLog.d("SpinLeft: Encoders: %7d,%7d,%7d,%7d", robot.lf.getCurrentPosition(), robot.rf.getCurrentPosition(), robot.lb.getCurrentPosition(), robot.rb.getCurrentPosition());
        }
    }

    // SpinRight Function
    private void SpinRight(int spinright_encoder_pulses, double drive_power) {
        robot.lf.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.lb.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.rf.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.rb.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.lf.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        robot.lb.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        robot.rf.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        robot.rb.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        robot.lf.setTargetPosition(spinright_encoder_pulses);
        robot.rf.setTargetPosition(spinright_encoder_pulses);
        robot.lb.setTargetPosition(spinright_encoder_pulses);
        robot.rb.setTargetPosition(spinright_encoder_pulses);

        robot.lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lf.setPower(drive_power);
        robot.rf.setPower(drive_power);
        robot.lb.setPower(drive_power);
        robot.rb.setPower(drive_power);

        while (opModeIsActive() &&
                // (runtime.seconds() < timeoutS) &&
                (robot.lf.isBusy())) {
            telemetry.addData("Running to", " %7d ", spinright_encoder_pulses);
            telemetry.addData("Currently at", " at %7d", robot.lf.getCurrentPosition());
            telemetry.update();
            RobotLog.d("SpinRight: Encoders: %7d,%7d,%7d,%7d", robot.lf.getCurrentPosition(), robot.rf.getCurrentPosition(), robot.lb.getCurrentPosition(), robot.rb.getCurrentPosition());
        }
    }

    // DriveForward Function
    private void DriveForward(int forward_encoder_pulses, double drive_power) {
        robot.lf.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.lb.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.rf.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.rb.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.lf.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        robot.lb.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        robot.rf.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        robot.rb.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        robot.lf.setTargetPosition(-forward_encoder_pulses);
        robot.rf.setTargetPosition(+forward_encoder_pulses);
        robot.lb.setTargetPosition(-forward_encoder_pulses);
        robot.rb.setTargetPosition(+forward_encoder_pulses);

        robot.lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lf.setPower(drive_power);
        robot.rf.setPower(drive_power);
        robot.lb.setPower(drive_power);
        robot.rb.setPower(drive_power);

        while (opModeIsActive() &&
                // (runtime.seconds() < timeoutS) &&
                (robot.lf.isBusy())) {
            telemetry.addData("Running to", " %7d ", forward_encoder_pulses);
            telemetry.addData("Currently at", " at %7d", robot.lf.getCurrentPosition());
            telemetry.update();
            RobotLog.d("Forward: Encoders: %7d,%7d,%7d,%7d", robot.lf.getCurrentPosition(), robot.rf.getCurrentPosition(), robot.lb.getCurrentPosition(), robot.rb.getCurrentPosition());
        }
    }

    // DriveReverse Function
    private void DriveReverse(int reverse_encoder_pulses, double drive_power){
        robot.lf.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.lb.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.rf.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.rb.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.lf.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        robot.lb.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        robot.rf.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        robot.rb.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        robot.lf.setTargetPosition(reverse_encoder_pulses);
        robot.rf.setTargetPosition(-reverse_encoder_pulses);
        robot.lb.setTargetPosition(reverse_encoder_pulses);
        robot.rb.setTargetPosition(-reverse_encoder_pulses);

        robot.lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lf.setPower(drive_power);
        robot.rf.setPower(drive_power);
        robot.lb.setPower(drive_power);
        robot.rb.setPower(drive_power);

        while (opModeIsActive() &&
                // (runtime.seconds() < timeoutS) &&
                (robot.lf.isBusy())) {
            telemetry.addData("Running to", " %7d ", reverse_encoder_pulses);
            telemetry.addData("Currently at", " at %7d", robot.lf.getCurrentPosition());
            telemetry.update();
            RobotLog.d("Reverse: Encoders: %7d,%7d,%7d,%7d", robot.lf.getCurrentPosition(), robot.rf.getCurrentPosition(), robot.lb.getCurrentPosition(), robot.rb.getCurrentPosition());
        }
    }

    // DriveStop Function
    private void DriveStop(int i) {
        robot.lf.setPower(i);
        robot.rf.setPower(i);
        robot.lb.setPower(i);
        robot.rb.setPower(i);
    }

}
