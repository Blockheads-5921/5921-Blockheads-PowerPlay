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
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.common.auto.AutoMethods;


@Autonomous(name="Robot: RefactoringTestsA5", group="Robot")
public class RefactoringTestsA5 extends AutoMethods {
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        serv0 = hardwareMap.get(CRServo.class, "serv0");

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        RobotLog.d("5921","Step4");

        // VALID COUNTS PER 90 DEGREES ROTATION as of 10/31/2022: 4*920 cnts/90 degrees
        // VALID COUNTS PER INCH for strafing as of 10/31/2022: 49.549 cnts/inch
        // VALID COUNTS PER INCH for normal driving as of 10/31/22: 43.651 cnts/inch


        // PUT AUTONOMOUS SCRIPT HERE

        // SCRIPT FOR STARTING AT A5

        double autoPower = 0.40;
        int imageNo = getConeImage();
        switch (imageNo) {
            case 1:
                StrafeLeft(1200, autoPower);
                break;
            case 2:
                break;
            case 3:
                StrafeRight(1200, autoPower);
                break;
        }
        DriveForward(1200, autoPower);

        /*
        double autoPower = 0.40;
        int sleepTime = 1;
        serv0.setPower(-0.1);
        sleep(sleepTime);
        DriveForward(200, autoPower);
        sleep(sleepTime);
        SpinRight(920, autoPower); //face towards cones
        sleep(sleepTime);
        SetBrakes(true);
        DriveForward(1000, autoPower); //move robot to pad A4, we're basing all operations on row 3
        sleep(sleepTime);
        SetBrakes(true);
        for (int i = 0; i < 2; i++){ //go back and forth between substation and high junction
            StrafeLeft(1775, autoPower); //move to high pole
            sleep(sleepTime);
            SetBrakes(true);
            DepositCone(3); //drop cone on high pole (height 3)
            StrafeRight(1775, autoPower); // Strafe back to A4
            sleep(sleepTime);
            SetBrakes(true);
            DriveForward(350, autoPower); //Go forward to pick up cone.
            sleep(sleepTime);
            SetBrakes(true);
            serv0.setPower(-0.1); //Pick up cone
            sleep(500);
            DriveReverse(350, autoPower); //Go back after picking up cone. We're now centered at A4 again.
            sleep(sleepTime);
            SetBrakes(true);
            sleep(200);
        }
        DriveForward(300, autoPower); //go to our substation
        sleep(sleepTime);
        SetBrakes(true);

         */

    }
}