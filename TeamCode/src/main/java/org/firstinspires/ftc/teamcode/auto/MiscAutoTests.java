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
import com.qualcomm.robotcore.hardware.CRServo;
import org.firstinspires.ftc.teamcode.common.HardwareDrive;
import org.firstinspires.ftc.teamcode.common.Constants;

import java.util.Set;


@Autonomous(name="Robot: Misc Auto Tests", group="Robot")
//@Disabled
public class MiscAutoTests extends LinearOpMode {

    Constants constants = new Constants();
    HardwareDrive robot = new HardwareDrive();
    private CRServo serv0;
    private final ElapsedTime
    runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        serv0 = hardwareMap.get(CRServo.class, "serv0");

        robot.lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        robot.lf.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.lb.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.rf.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.rb.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        robot.lf.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        robot.lb.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        robot.rf.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        robot.rb.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Status", "Ready to run");
        telemetry.addData("Starting encoder values: ", "lf %d lb %d rf %d rb %d",
                robot.lf.getCurrentPosition(), robot.lb.getCurrentPosition(), robot.rf.getCurrentPosition(), robot.rb.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        RobotLog.d("5921", "Step4");


        // VALID COUNTS PER 90 DEGREES ROTATION as of 10/31/2022: 4*920 cnts/90 degrees
        // VALID COUNTS PER INCH for strafing as of 10/31/2022: 49.549 cnts/inch
        // VALID COUNTS PER INCH for normal driving as of 10/31/22: 43.651 cnts/inch


        // PUT AUTONOMOUS SCRIPT HERE

        // SCRIPT FOR STARTING AT F5

        double autoPower = 0.20;

        double dirX = 0;
        double dirY = -1;
        double dirR = 0.75;
        double xIncOrDec = 0.01;

        serv0.setPower(-0.1);
        sleep(200);

        robot.lift.setTargetPosition(Constants.elevatorPositionMid);
        robot.lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lift.setPower(0.5);

        // sean simulator
        for (int i = 0; i < 150; i ++) {
            telemetry.addData("Iteration ", i);
            dirY += 0.01;
            // make x start decreasing once it's hit 1
            if (dirX >= 1.00) { xIncOrDec *= -1; telemetry.addLine("Turning X back down");}
            dirX += xIncOrDec;
            // Positive x goes to the robot's right, positive y goes forward
            if (i == 120) { dirY += 0.56; dirX += 0.2; telemetry.addLine("Boosting linear directions");}
            TOpStyleDrive(dirX, dirY, dirR, autoPower);
            telemetry.addData("Values", "dirX: %f dirY: %f dirR: %f xIncOrDec: %f", dirX, dirY, dirR, xIncOrDec);
            telemetry.update();
            sleep(20);
        }
        SetBrakes(true);

        serv0.setPower(0.18); // Drop cone

        /*
        telemetry.addLine("Setting target positions...");
        telemetry.update();
        robot.lf.setTargetPosition(-2125);
        robot.lf.setTargetPosition(-220);
        robot.lf.setTargetPosition(-2613);
        robot.lf.setTargetPosition(-727);

        telemetry.addLine("Setting mode to RUN_TO_POSITION...");
        telemetry.update();
        robot.lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Now that we're facing the right direction, we can run to position to correct for any remaining error
        // surely there's a better way to do this... i wish this were python
        int[] posErrors = {-2125 - robot.lf.getCurrentPosition(), -229 - robot.lb.getCurrentPosition(),
                -2613 - robot.rf.getCurrentPosition(), -727 - robot.rf.getCurrentPosition()};

        int maxDiff = 0;
        for (int cNum: posErrors) {
            if (cNum > maxDiff) {maxDiff = cNum;}
        }
        telemetry.addLine("Setting power...");
        telemetry.update();
        if (maxDiff > 50){
            robot.lf.setPower((posErrors[0]/maxDiff)*autoPower);
            robot.lb.setPower((posErrors[1]/maxDiff)*autoPower);
            robot.lb.setPower((posErrors[2]/maxDiff)*autoPower);
            robot.lb.setPower((posErrors[3]/maxDiff)*autoPower);

        }
        /*
        telemetry.addData("Ending encoder values: ", "lf %d lb %d rf %d rb %d",
                robot.lf.getCurrentPosition(), robot.lb.getCurrentPosition(), robot.rf.getCurrentPosition(), robot.rb.getCurrentPosition());
        telemetry.update();
        sleep(30000);
         */
    }

    private void TOpStyleDrive(double directionX, double directionY, double directionR, double powerCoef){
        directionY *= -1;
        directionR *= -1;
        robot.lf.setPower((directionY + directionR - directionX) * powerCoef);
        robot.rf.setPower((-directionY + directionR - directionX) * powerCoef);
        robot.lb.setPower((directionY + directionR + directionX) * powerCoef);
        robot.rb.setPower((-directionY + directionR + directionX) * powerCoef);
    }

    private void SetBrakes(boolean brakesOn) {
        if (brakesOn){
            robot.lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            robot.lf.setPower(0);
            robot.rf.setPower(0);
            robot.lb.setPower(0);
            robot.rb.setPower(0);

            robot.lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        else{
            robot.lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            robot.rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            robot.lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            robot.rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
    }



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

        robot.lf.setVelocity(drivePower);
        robot.rf.setVelocity(drivePower);
        robot.lb.setVelocity(drivePower);
        robot.rb.setVelocity(drivePower);

        while (opModeIsActive() &&
                // (runtime.seconds() < timeoutS) &&
                (robot.lf.isBusy())) {
            telemetry.addData("Running to", " %7d ", forwardEncoderPulses);
            telemetry.addData("Currently at", " at %7d", robot.lf.getCurrentPosition());
            telemetry.update();
            RobotLog.d("Forward: Encoders: %7d,%7d,%7d,%7d", robot.lf.getCurrentPosition(), robot.rf.getCurrentPosition(), robot.lb.getCurrentPosition(), robot.rb.getCurrentPosition());
        }
    }

    private void StrafeRight(int straferightEncoderPulses, double drivePower) {
        robot.lf.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.lb.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.rf.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.rb.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.lf.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        robot.lb.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        robot.rf.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        robot.rb.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        robot.lf.setTargetPosition(-straferightEncoderPulses);
        robot.rf.setTargetPosition(-straferightEncoderPulses);
        robot.lb.setTargetPosition(straferightEncoderPulses);
        robot.rb.setTargetPosition(straferightEncoderPulses);

        robot.lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lf.setPower(drivePower);
        robot.rf.setPower(drivePower);
        robot.lb.setPower(drivePower);
        robot.rb.setPower(drivePower);

        // update the telemetry monitor
        while (opModeIsActive() && (robot.lf.isBusy())) {
            telemetry.addData("Running to", " %7d ", straferightEncoderPulses);
            telemetry.addData("Currently at", " at %7d", robot.lf.getCurrentPosition());
            telemetry.update();
            RobotLog.d("StrafeRight: Encoders: %7d,%7d,%7d,%7d", robot.lf.getCurrentPosition(), robot.rf.getCurrentPosition(), robot.lb.getCurrentPosition(), robot.rb.getCurrentPosition());
        }
    }

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

    private void DepositCone(int junctionLevel){
        //assumes lift is at bottom
        int targetPos = 0;
        switch (junctionLevel) {
            case 1:
                targetPos = Constants.elevatorPositionLow;
                break;
            case 2:
                targetPos = Constants.elevatorPositionMid;
                break;
            case 3:
                targetPos = Constants.elevatorPositionTop;
                break;
        }
        //raise arm
        robot.lift.setTargetPosition(targetPos);
        robot.lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lift.setPower(1.00);
        sleep(2300);
        robot.lift.setPower(0); //Brake arm, maybe unnecessary?
        //Drive forward
        SetBrakes(false);
        DriveForward(120,0.15);
        //Release cone
        serv0.setPower(0.18);
        //Back up
        DriveReverse(120,0.20);
        sleep(250);
        //lower arm
        robot.lift.setTargetPosition(Constants.elevatorPositionBottom);
        robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lift.setPower(0.75);
        sleep(750);
        SetBrakes(true);
    }
}