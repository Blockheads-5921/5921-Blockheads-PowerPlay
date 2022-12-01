package org.firstinspires.ftc.teamcode.driveModes;

import android.view.View;

// wat

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.Button;
import org.firstinspires.ftc.teamcode.common.Constants;
import org.firstinspires.ftc.teamcode.common.HardwareDrive;

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

    //might not need View relativelayout;, according to code this was built off of.
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
        UpdateTelemetry();

            }

    private void UpdatePlayer1() {
        double drivePower = DriveTrainSpeed();
        DriveTrainBase(drivePower);
        DriveMicroAdjust(0.4);
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
        double liftPower = Math.pow(gamepad2.right_stick_y, 1); //Lift
        int liftPos = robot.lift.getCurrentPosition();


        robot.lf.setPower((directionY + directionR - directionX) * drivePower);
        robot.rf.setPower((-directionY + directionR - directionX) * drivePower);
        robot.lb.setPower((directionY + directionR + directionX) * drivePower);
        robot.rb.setPower((-directionY + directionR + directionX) * drivePower);

        if (liftPos < Constants.elevatorPositionTop) {
            // Make sure we're not letting lift over-extend...
            robot.lift.setPower((liftPower) * 0.1);
        } else if (liftPos > Constants.elevatorPositionBottom) {
            // or over-retract.
            robot.lift.setPower((liftPower) * 0.01);
        } else {
            robot.lift.setPower((liftPower) * 0.60);
        }

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
        double drivePower = 0.25;

        if (gamepad1.right_bumper)
            drivePower = 1;
        else if (gamepad1.left_bumper)
            drivePower = 0.25;

        return drivePower;
    }

}
