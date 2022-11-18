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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.common.HardwareDrive;


@Autonomous(name="Robot: PowerPlayAutonomous", group="Robot")
//@Disabled
public class PowerPlayAutonomous extends LinearOpMode {

    HardwareDrive robot = new HardwareDrive();

    private final ElapsedTime
    runtime = new ElapsedTime();

// Unused, delete later?
//   static final double     FORWARD_SPEED = 0.6;
//   static final double     TURN_SPEED    = 0.5;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap); // initiates the hardware Map

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

        //SCRIPT FOR STARTING AT A2 or F5
        double autoPower = 0.12;
        int sleepTime = 1000;
        SpinLeft(920,autoPower); //face towards cones
        DriveStop(0);
        sleep(sleepTime);
        DriveForward(1048,autoPower); //move robot to pad A3, we're basing all operations on row 3
        DriveStop(0);
        sleep(sleepTime);
        for (int i=1; i<4; i++){ //repeat 5 times; 5 is arbitrary, adjust depending on how fast the robot is
            //PickUpCone(); //pick up cone
            StrafeRight(1571,autoPower); //move to high pole
            DriveStop(0);
            sleep(sleepTime);
            //DepositCone(3); //drop cone on high pole (height 3)
            StrafeLeft(1571,autoPower); //strafe back to cone area
            DriveStop(0);
            sleep(sleepTime);
        }
        DriveReverse(2095,autoPower); //go to our terminal
        DriveStop(0);

        /*SCRIPT FOR STARTING AT A5 or F2
        SpinRight(290,100); //face towards cones
        DriveForward(1048,100); //move robot to pad F3, we're basing all operations on row 3
        for (int i=0; i<5; i++){ //5 is arbitary, adjust depending on how fast the robot is
            PickUpCone(); //pick up cone
            StrafeLeft(1571,100); //move to high pole
            DepositCone(3); //drop cone on high pole (height 3)
            StrafeRight(1571,100); //strafe back to cone area
        }
        DriveReverse(2095,100); //go to our terminal Trentan made this
         */

    }

    // StrafeRight Function
    private void StrafeRight(int straferight_encoder_pulses, double drivePower) {
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
        robot.lf.setPower(drivePower);
        robot.rf.setPower(drivePower);
        robot.lb.setPower(drivePower);
        robot.rb.setPower(drivePower);


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
    private void StrafeLeft(int strafeleftEncoderPulses, double drivePower) {

        robot.lf.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.lb.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.rf.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.rb.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.lf.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        robot.lb.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        robot.rf.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        robot.rb.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        robot.lf.setTargetPosition(strafeleftEncoderPulses);
        robot.rf.setTargetPosition(strafeleftEncoderPulses);
        robot.lb.setTargetPosition(-strafeleftEncoderPulses);
        robot.rb.setTargetPosition(-strafeleftEncoderPulses);

        robot.lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lf.setPower(drivePower);
        robot.rf.setPower(drivePower);
        robot.lb.setPower(drivePower);
        robot.rb.setPower(drivePower);

        while (opModeIsActive() &&
                // (runtime.seconds() < timeoutS) &&
                (robot.lf.isBusy())) {
            telemetry.addData("Running to", " %7d ", strafeleftEncoderPulses);
            telemetry.addData("Currently at", " at %7d", robot.lf.getCurrentPosition());
            telemetry.update();
            RobotLog.d("StrafeLeft: Encoders: %7d,%7d,%7d,%7d", robot.lf.getCurrentPosition(), robot.rf.getCurrentPosition(), robot.lb.getCurrentPosition(), robot.rb.getCurrentPosition());
        }
    }

    // SpinLeft Function
    private void SpinLeft(int spinleftEncoderPulses, double drivePower) {

        robot.lf.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.lb.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.rf.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.rb.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.lf.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        robot.lb.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        robot.rf.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        robot.rb.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        robot.lf.setTargetPosition(spinleftEncoderPulses);
        robot.rf.setTargetPosition(spinleftEncoderPulses);
        robot.lb.setTargetPosition(spinleftEncoderPulses);
        robot.rb.setTargetPosition(spinleftEncoderPulses);

        robot.lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lf.setPower(drivePower);
        robot.rf.setPower(drivePower);
        robot.lb.setPower(drivePower);
        robot.rb.setPower(drivePower);

        while (opModeIsActive() &&
                // (runtime.seconds() < timeoutS) &&
                (robot.lf.isBusy())) {
            telemetry.addData("Running to", " %7d ", spinleftEncoderPulses);
            telemetry.addData("Currently at", " at %7d", robot.lf.getCurrentPosition());
            telemetry.update();
            RobotLog.d("SpinLeft: Encoders: %7d,%7d,%7d,%7d", robot.lf.getCurrentPosition(), robot.rf.getCurrentPosition(), robot.lb.getCurrentPosition(), robot.rb.getCurrentPosition());
        }
    }

    // SpinRight Function
    private void SpinRight(int spinrightEncoderPulses, double drivePower) {
        robot.lf.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.lb.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.rf.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.rb.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.lf.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        robot.lb.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        robot.rf.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        robot.rb.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        robot.lf.setTargetPosition(-spinrightEncoderPulses);
        robot.rf.setTargetPosition(-spinrightEncoderPulses);
        robot.lb.setTargetPosition(-spinrightEncoderPulses);
        robot.rb.setTargetPosition(-spinrightEncoderPulses);

        robot.lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lf.setPower(drivePower);
        robot.rf.setPower(drivePower);
        robot.lb.setPower(drivePower);
        robot.rb.setPower(drivePower);

        while (opModeIsActive() &&
                // (runtime.seconds() < timeoutS) &&
                (robot.lf.isBusy())) {
            telemetry.addData("Running to", " %7d ", spinrightEncoderPulses);
            telemetry.addData("Currently at", " at %7d", robot.lf.getCurrentPosition());
            telemetry.update();
            RobotLog.d("SpinRight: Encoders: %7d,%7d,%7d,%7d", robot.lf.getCurrentPosition(), robot.rf.getCurrentPosition(), robot.lb.getCurrentPosition(), robot.rb.getCurrentPosition());
        }
    }

    // DriveForward Function
    private void DriveForward(int forwardEncoderPulses, double drivePower) {
        robot.lf.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.lb.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.rf.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.rb.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.lf.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        robot.lb.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        robot.rf.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        robot.rb.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        robot.lf.setTargetPosition(-forwardEncoderPulses);
        robot.rf.setTargetPosition(+forwardEncoderPulses);
        robot.lb.setTargetPosition(-forwardEncoderPulses);
        robot.rb.setTargetPosition(+forwardEncoderPulses);

        robot.lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lf.setPower(drivePower);
        robot.rf.setPower(drivePower);
        robot.lb.setPower(drivePower);
        robot.rb.setPower(drivePower);

        while (opModeIsActive() &&
                // (runtime.seconds() < timeoutS) &&
                (robot.lf.isBusy())) {
            telemetry.addData("Running to", " %7d ", forwardEncoderPulses);
            telemetry.addData("Currently at", " at %7d", robot.lf.getCurrentPosition());
            telemetry.update();
            RobotLog.d("Forward: Encoders: %7d,%7d,%7d,%7d", robot.lf.getCurrentPosition(), robot.rf.getCurrentPosition(), robot.lb.getCurrentPosition(), robot.rb.getCurrentPosition());
        }
    }

    // DriveReverse Function
    private void DriveReverse(int reverseEncoderPulses, double drivePower){
        robot.lf.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.lb.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.rf.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.rb.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.lf.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        robot.lb.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        robot.rf.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        robot.rb.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        robot.lf.setTargetPosition(reverseEncoderPulses);
        robot.rf.setTargetPosition(-reverseEncoderPulses);
        robot.lb.setTargetPosition(reverseEncoderPulses);
        robot.rb.setTargetPosition(-reverseEncoderPulses);

        robot.lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lf.setPower(drivePower);
        robot.rf.setPower(drivePower);
        robot.lb.setPower(drivePower);
        robot.rb.setPower(drivePower);

        while (opModeIsActive() &&
                // (runtime.seconds() < timeoutS) &&
                (robot.lf.isBusy())) {
            telemetry.addData("Running to", " %7d ", reverseEncoderPulses);
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
//        //assumes gripper is open and arm is down; should be this way
//        robot.gripper.setTargetPosition(closedPosition); //this is a placeholder
//        robot.gripper.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.gripper.setPower(0.35);
//    }

    // DriveStop Function
    private void DriveStop(double i) {
        if (i == 0){
            robot.lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        else{
            robot.lf.setPower(i);
            robot.rf.setPower(i);
            robot.lb.setPower(i);
            robot.rb.setPower(i);
        }
    }

}
