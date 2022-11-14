package org.firstinspires.ftc.teamcode;

import android.view.View;

// wat

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Base Drive03_TwoDrivers", group = "Drive")
//@Disabled
public class BaseDrive03_TwoDrivers extends LinearOpMode {
    /* Declare OpMode members. */
    HardwareDrive robot = new HardwareDrive();
    //Constants constants = new Constants();
    private CRServo serv0;
    private ElapsedTime runtime = new ElapsedTime();

    Button lifterButton = new Button();
    Button lifterBottomButton = new Button();

    private boolean toggleButton = true;

    /**
     * The relativeLayout field is used to aid in providing interesting visual feedback
     * in this sample application; you probably *don't* need this when you use a color sensor on your
     * robot. Note that you won't see anything change on the Driver Station, only on the Robot Controller.
     */
    View relativeLayout;

    @Override
    public void runOpMode() {
        serv0 = hardwareMap.get(CRServo.class, "serv0");
        robot.init(hardwareMap);
        telemetry.addData("Say", "Hello Driver");
        runtime.reset();
        robot.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        while (opModeIsActive()) {
            loop1();
        }

    }

    private void loop1() {
        UpdateButton();
        UpdatePlayer1();
        UpdatePlayer2();
        UpdateTelemetry();

            }

    private void UpdatePlayer1() {
        double drivePower = DriveTrainSpeed();
        DriveTrainBase(drivePower);
        DriveTrainSpeed();
        DriveMicroAdjust(0.4);
    }

    private void UpdatePlayer2() {

//      Currently using manual lifter for Player2, debug Lifter(), then uncomment Lifter() below.
//      Lifter() function, currently is not advised, -commented out below
//      Using .power approach vs. .encoder values
//        Lifter();


    }

    private void UpdateTelemetry() {

        telemetry.addData("g1.X", gamepad1.left_stick_x);
        telemetry.addData("g1.Y", -gamepad1.left_stick_y);
        telemetry.addData("g1.R", gamepad1.right_stick_x);
        telemetry.addData("Arm Position", robot.lift.getCurrentPosition());
        telemetry.addData("g2.L", gamepad2.right_stick_y);
        telemetry.update();
    }

    private void UpdateButton() {

        // This code operates the gripper, consider placing this into a function

        if (gamepad2.left_trigger > 0.01) {
            serv0.setPower(0.22 * gamepad2.left_trigger - 0);}
                else if  (gamepad2.right_trigger > 0.01) {
            serv0.setPower(-0.1 * gamepad2.right_trigger + 0);
        }

    }

    private void DriveTrainBase(double drivePower) {
        double directionX = Math.pow(gamepad1.left_stick_x, 1); //Strafe
        double directionY = Math.pow(gamepad1.left_stick_y, 1); //Forward
        double directionR = -Math.pow(gamepad1.right_stick_x, 1); //Turn
        double directionL = Math.pow(gamepad2.right_stick_y, 1); //Lift


        robot.lf.setPower((directionY + directionR - directionX) * drivePower);
        robot.rf.setPower((-directionY + directionR - directionX) * drivePower);
        robot.lb.setPower((directionY + directionR + directionX) * drivePower);
        robot.rb.setPower((-directionY + directionR + directionX) * drivePower);
 //       robot.lift.setPower((directionL) * 0.35);

 // trying to prevent the lift from stalling at the top, see if statement below
        if (robot.lift.getCurrentPosition() > -2900){
        robot.lift.setPower((directionL) * 0.40);}
        else {
            robot.lift.setPower((directionL) * 0.01);}

//        if (gamepad2.right_stick_y > -0.75) {
//            robot.lift.setPower((directionL) * 0.05); // give lift some constant power to prevent fast drops
//        }
//        else if (gamepad2.right_stick_y > -0.5) {
//            robot.lift.setPower((directionL) * 0.01); // give lift some constant power to prevent fast drops
//        }
    }


    private void DriveMicroAdjust(double power) {
        if (gamepad1.dpad_up) {
            robot.lf.setPower(-power);
            robot.rf.setPower(+power);
            robot.lb.setPower(-power);
            robot.rb.setPower(+power);
        } else if (gamepad1.dpad_down) {
            robot.lf.setPower(+power);
            robot.rf.setPower(-power);
            robot.lb.setPower(+power);
            robot.rb.setPower(-power);
        } else if (gamepad1.dpad_right) {
            robot.lf.setPower(power);
            robot.rf.setPower(power);
            robot.lb.setPower(power);
            robot.rb.setPower(power);
        } else if (gamepad1.dpad_left) {
            robot.lf.setPower(-power);
            robot.rf.setPower(-power);
            robot.lb.setPower(-power);
            robot.rb.setPower(-power);
        }

        if (gamepad1.left_trigger == 1) {
            robot.lf.setPower(-power);
            robot.rf.setPower(power);
            robot.lb.setPower(-power);
            robot.rb.setPower(power);
        } else if (gamepad1.right_trigger == 1) {
            robot.lf.setPower(power);
            robot.rf.setPower(-power);
            robot.lb.setPower(power);
            robot.rb.setPower(-power);
        }
    }

    double DriveTrainSpeed() {
        double drivePower = 1;

        if (gamepad1.right_bumper)
            drivePower = 1;
        else if (gamepad1.left_bumper)
            drivePower = 0.25;

        return drivePower;
    }

    // uncomment start
    // rebuild all of Lifter function, currently just using manual mode above, in drivetrain
    private void Lifter() {
        int position = robot.lift.getCurrentPosition();
        if (lifterButton.is(Button.State.TAP)) {
             if (position >= (Constants.elevatorPositionTop + 20)) {
                robot.lift.setTargetPosition(Constants.elevatorPositionDown - 20);
                robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.lift.setPower(0.35);
                while (opModeIsActive() &&
                        // (runtime.seconds() < timeoutS) &&
                        (robot.lift.isBusy())) {

                    // Display it for the driver.
                    telemetry.addData("Running to", " %7d ", Constants.elevatorPositionDown - 50);
                    telemetry.addData("Currently at", " at %7d", robot.lift.getCurrentPosition());
                    telemetry.update();
                }


            } else {
                robot.lift.setTargetPosition(Constants.elevatorPositionTop);
                robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.lift.setPower(0.35);
            }
        }

        if (lifterBottomButton.is(Button.State.TAP)) {
            if (position >= (Constants.elevatorPositionBottom + 400)) {
                robot.lift.setTargetPosition(Constants.elevatorPositionDown + 390);
                robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.lift.setPower(0.3);
            } else {
                robot.lift.setTargetPosition(Constants.elevatorPositionBottom);
                robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.lift.setPower(0.3);
            }
        }

        // next two if statements, Code below is very buggy, delete and rebuild
//        if (gamepad2.left_bumper) {
//            robot.lift.setTargetPosition(position + 50);
//            robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            robot.lift.setPower(0.35);
//        }
//        if (gamepad2.right_bumper) {
//            robot.lift.setTargetPosition(position - 50);
//            robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            robot.lift.setPower(0.35);
//        }
    }

}
