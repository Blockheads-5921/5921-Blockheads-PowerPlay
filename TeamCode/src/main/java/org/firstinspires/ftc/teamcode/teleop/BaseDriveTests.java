package org.firstinspires.ftc.teamcode.teleop;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.Button;
import org.firstinspires.ftc.teamcode.common.Constants;
import org.firstinspires.ftc.teamcode.common.HardwareDrive;
import org.firstinspires.ftc.teamcode.common.Utility;

@TeleOp(name = "Base Drive Tests", group = "Drive")
//@Disabled
public class BaseDriveTests extends LinearOpMode {
    /* Declare OpMode members. */
    HardwareDrive robot = new HardwareDrive();
    private Constants constants = new Constants();
    private CRServo serv0;
    private CRServo serv1;
    private ElapsedTime runtime = new ElapsedTime();
    private Button lifterButton = new Button();
    private Button lifterBottomButton = new Button();
    private boolean toggleButton = true;
    int lTgtPos = 0;


    @Override
    public void runOpMode() {
        serv0 = hardwareMap.get(CRServo.class, "serv0");
        serv1 = hardwareMap.get(CRServo.class, "serv1");
        robot.init(hardwareMap);
        telemetry.addData("Say", "Hello Driver");
        runtime.reset();
        robot.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();
        while (opModeIsActive()) loop1();
    }
    private void loop1() {

        if (gamepad2.y) {
            lTgtPos = Constants.elevatorPositionTop;
        } else if (gamepad2.x) {
            lTgtPos = Constants.elevatorPositionMid;
        } else if (gamepad2.a) {
            lTgtPos = Constants.elevatorPositionLow;
        } else if (gamepad2.b) {
            lTgtPos = Constants.elevatorPositionBottom - 200;
        } else if (gamepad2.dpad_down || gamepad2.dpad_left || gamepad2.dpad_right || gamepad2.dpad_up) {
            lTgtPos = Constants.elevatorPositionBottom;
        }

        telemetry.addData("Difference between target and current lift positions", lTgtPos - robot.lift.getCurrentPosition());

        // The absolute sum of the positions of the motors of the left side minus the same for the left side.
        // Proportional to the amount turned.

        int liftTgtOffset = lTgtPos - robot.lift.getCurrentPosition(); // if negative, lift is higher than target position

        if (liftTgtOffset > 100) { // lift is too high
            robot.lift.setPower(0.6);
            telemetry.addLine("Lift too high");
        } else if (liftTgtOffset < -100) { // lift is too low
            robot.lift.setPower(-0.6);
            telemetry.addLine("Lift too low");
        } else {
            robot.lift.setPower(-0.001);
            telemetry.addLine("Lift position OK");
        }

        if (gamepad2.left_trigger > 0.01) {serv1.setPower(gamepad2.left_trigger);}

        double drivePower = 0.4;
        if (gamepad1.right_bumper) {drivePower = 1;}
        else if (gamepad1.left_bumper) {drivePower = 0.25;}
        DriveTrainBase(drivePower, lTgtPos);
        DriveMicroAdjust(0.4);
        UpdateTelemetry();
    }

    @Utility.Encapsulate
    private void DriveTrainBase(double drivePower, int lTgtPos) {
        double directionX = Math.pow(gamepad1.left_stick_x, 1); // Strafe
        double directionY = Math.pow(gamepad1.left_stick_y, 1); // Forward
        double directionR = -Math.pow(gamepad1.right_stick_x, 1); // Turn
        // double liftPower = Math.pow(gamepad2.right_stick_y, 1); // Lift

        //dead zones
        if (gamepad1.left_stick_x < 0.2 && gamepad1.left_stick_x > -0.2) {directionX = 0;}
        if (gamepad1.left_stick_y < 0.2 && gamepad1.left_stick_y > -0.2) {directionY = 0;}

        robot.lf.setPower((directionY + directionR - directionX) * drivePower);
        robot.rf.setPower((-directionY + directionR - directionX) * drivePower);
        robot.lb.setPower((directionY + directionR + directionX) * drivePower);
        robot.rb.setPower((-directionY + directionR + directionX) * drivePower);

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

    private void UpdateTelemetry() {
        telemetry.addData("g1.X", gamepad1.left_stick_x);
        telemetry.addData("g1.Y", -gamepad1.left_stick_y);
        telemetry.addData("g1.R", gamepad1.right_stick_x);
        telemetry.addData("Arm Position", robot.lift.getCurrentPosition());
        telemetry.addData("g2.L", gamepad2.right_stick_y);
        telemetry.update();
    }
}
