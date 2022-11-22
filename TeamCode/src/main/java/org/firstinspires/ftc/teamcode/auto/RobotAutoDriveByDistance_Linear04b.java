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
The Purpose of this Program is to run calibration on Forward, Reverse, Spin and Strafe
 */

@Autonomous(name="Robot: Auto Drive By Distance 04b", group="Robot")
@Disabled
public class RobotAutoDriveByDistance_Linear04b extends LinearOpMode {

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

        //VALID COUNTS PER 90 DEGREES ROTATION as of 10/31/2022: 4*920 cnts/90 degrees
        //VALID COUNTS PER INCH for strafing as of 10/31/2022: 49.549 cnts/inch
        //VALID COUNTS PER INCH for normal driving as of 10/31/22: 43.651 cnts/inch


        //PUT AUTONOMOUS SCRIPT HERE
        int drive_power_all = 10;       // 0 to 100
        int wait_after_drive = 300;     // millisec


        //SCRIPT FOR STARTING AT A2 or F5. Not sure about the 100% power, the robot seems to do a fair amount of coasting.
        SpinLeft(290,drive_power_all); //face towards cones
        DriveStop(0);
        sleep(wait_after_drive);
        DriveForward(1048,drive_power_all); //move robot to pad A3, we're basing all operations on row 3
        DriveStop(0);
        sleep(wait_after_drive);
        for (int i=1; i<4; i++){ //5 is arbitary, adjust depending on how fast the robot is
            //PickUpCone(); //pick up cone
            StrafeRight(1571,drive_power_all); //move to high pole

            DriveStop(0);
            sleep(wait_after_drive);

            //DepositCone(3); //drop cone on high pole (height 3)
            StrafeLeft(1571,drive_power_all); //strafe back to cone area
            DriveStop(0);
            sleep(wait_after_drive);
        }
        DriveReverse(2095,drive_power_all); //go to our terminal
        DriveStop(0);
        sleep(wait_after_drive);
        /*SCRIPT FOR STARTING AT A5 or F2
        SpinRight(290,100); //face towards cones
        DriveForward(1048,100); //move robot to pad F3, we're basing all operations on row 3
        for (int i=0; i<5; i++){ //5 is arbitary, adjust depending on how fast the robot is
            PickUpCone(); //pick up cone
            StrafeLeft(1571,100); //move to high pole
            DepositCone(3); //drop cone on high pole (height 3)
            StrafeRight(1571,100); //strafe back to cone area
        }
        DriveReverse(2095,100); //go to our terminal
         */

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

//    // Lifter function
//    private void DepositCone(int junctionLevel){
//        //assumes lift is at bottom and claw is closed
//        switch (junctionLevel) {
//            case 1:
//                targetPos = null; //fill these out, they're for how high to raise the lift. IDK the values myself.
//                break;
//            case 2:
//                targetPos = null; //so for example this value for targetPos would cause the elevator to go higher than the previous
//                break;
//            case 3:
//                targetPos = null; // and this would be still higher
//                break;
//        }
//        //raise arm
//        robot.lift.setTargetPosition(targetPos); //does not work now because targetPos is null
//        robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.lift.setPower(0.35);
//
//        while (opModeIsActive() && (robot.lift.isBusy())) {
//            telemetry.addData("Lifter lifting...");
//        }
//        
//        //release cone
//        robot.gripper.setTargetPosition(openPosition); //this is a placeholder
//        robot.gripper.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.gripper.setPower(0.35);
//
//        //lower arm
//        robot.lift.setTargetPosition(Constants.elevatorPositionDown - 20);
//        robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION); //might need changing?
//        robot.lift.setPower(0.35);
//    }
//
//    //Pick up cone function
//    private void PickUpCone(){
//        robot.gripper.setTargetPosition(closedPosition); //this is a placeholder
//        robot.gripper.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.gripper.setPower(0.35);
//    }

    // DriveStop Function
    private void DriveStop(int power) {
        robot.lf.setPower(power);
        robot.rf.setPower(power);
        robot.lb.setPower(power);
        robot.rb.setPower(power);
    }

}
