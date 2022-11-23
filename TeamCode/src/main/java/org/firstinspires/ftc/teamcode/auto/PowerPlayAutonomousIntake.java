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
import org.firstinspires.ftc.teamcode.common.Constants;
import org.firstinspires.ftc.teamcode.common.HardwareDrive;

@Autonomous(name="Robot: PowerPlayAutonomousIntake", group="Robot")
//@Disabled
public class PowerPlayAutonomousIntake extends LinearOpMode {

    Constants constants = new Constants();
    HardwareDrive robot = new HardwareDrive();
    private CRServo serv0;
    private final ElapsedTime
    runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap); // initiates the hardware Map
        serv0 = hardwareMap.get(CRServo.class, "serv0");

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
        double autoPower = 0.30;
        int sleepTime = 1500;
        Drive("Normal",3,autoPower); //Center robot
        sleep(sleepTime);
        DriveStop(0);
        Drive("Rotate",-1,autoPower); //face towards cones
        sleep(sleepTime);
        DriveStop(0);
        Drive("Normal",24,autoPower); //move robot to pad A3, we're basing all operations on row 3
        sleep(sleepTime);
        DriveStop(0);
        for (int i=1; i<3; i++){ //repeat an arbitrary number of times, adjust depending on how fast the robot is
            serv0.setPower(0.22); //Grab cone
            Drive("Strafe",36,autoPower); //move to high pole
            DriveStop(0);
            sleep(sleepTime);
            DepositCone(3); //drop cone on high pole (height 3)
            Drive("Strafe",-36,autoPower); //strafe back to cone area
            sleep(sleepTime);
            DriveStop(0);
        }
        Drive("Normal",-48,autoPower); //go to our terminal
        DriveStop(0);

    }

    //Move function
    private void Drive(String movement_type, int inches_or_right_angles, double drive_power) {
        //Moves the robot in the specified movement type. Positive Normal drives forwards, 
        //positive Strafe drives left, and positive Rotate spins the robot clockwise.
        int[] motorDirCoefs = {0,0,0,0};
        int encoder_pulses = 0;
        switch(movement_type) {
            case "Normal":
                motorDirCoefs = new int[] {-1, 1, -1, 1};
                encoder_pulses = 44*inches_or_right_angles;
                break;
            case "Strafe":
                motorDirCoefs = new int[] {1,1,1,1};
                encoder_pulses = 50*inches_or_right_angles;
                break;
            case "Rotate":
                motorDirCoefs = new int[]{-1,-1,-1,-1};
                encoder_pulses = 920*inches_or_right_angles;
                break;
            
        }
        robot.lf.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.lb.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.rf.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.rb.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        robot.lf.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        robot.lb.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        robot.rf.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        robot.rb.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        robot.lf.setTargetPosition(encoder_pulses*motorDirCoefs[0]);
        robot.lb.setTargetPosition(encoder_pulses*motorDirCoefs[1]);
        robot.rf.setTargetPosition(encoder_pulses*motorDirCoefs[2]);
        robot.rb.setTargetPosition(encoder_pulses*motorDirCoefs[3]);

        robot.lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.lf.setPower(drive_power);
        robot.rf.setPower(drive_power);
        robot.lb.setPower(drive_power);
        robot.rb.setPower(drive_power);
    }


    // Lifter function
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
        robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lift.setPower(0.35);
        //Drive forwards and drop cone
        Drive("Normal",2,0.35);
        serv0.setPower(-0.1);
        Drive("Normal",-2,0.35);
        //lower arm
        robot.lift.setTargetPosition(Constants.elevatorPositionBottom);
        robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lift.setPower(0.35);
    }


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
