package org.firstinspires.ftc.teamcode.driveModes;
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
    private ElapsedTime runtime = new ElapsedTime();
    private Button lifterButton = new Button();
    private Button lifterBottomButton = new Button();
    private boolean toggleButton = true;
    int lTgtPos = 0;
    int robotAngle;
    int xPower;
    int yPower;
    int rPower;


    @Override
    public void runOpMode() {
        serv0 = hardwareMap.get(CRServo.class, "serv0");
        robot.init(hardwareMap);
        telemetry.addData("Say", "Hello Driver");
        runtime.reset();
        robot.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();
        while (opModeIsActive()) loop1();
    }
    private void loop1() {
        // Lift+intake stuff
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

        int liftTgtOffset = lTgtPos - robot.lift.getCurrentPosition(); // if negative, lift is higher than target position

        if (liftTgtOffset > 100) { // lift is too high
            robot.lift.setPower(0.75);
            telemetry.addLine("Lift too high");
        } else if (liftTgtOffset < -100) { // lift is too low
            robot.lift.setPower(-0.90);
            telemetry.addLine("Lift too low");
        } else {
            robot.lift.setPower(-0.001);
            telemetry.addLine("Lift position OK");
        }

        if (gamepad2.left_trigger > 0.01) serv0.setPower(0.22 * gamepad2.left_trigger - 0);
        else if  (gamepad2.right_trigger > 0.01) serv0.setPower(-0.1 * gamepad2.right_trigger + 0);

        // Drivetrain code

        // The absolute sum of the positions of the motors of the left side minus the same for the left side.
        // Proportional to the amount turned.
        int diffsAbsSidesPosSums = Math.abs(robot.lf.getCurrentPosition() + robot.lb.getCurrentPosition())
                - Math.abs(robot.rf.getCurrentPosition() + robot.rb.getCurrentPosition());

        telemetry.addData("Abs sum of left motor positions minus abs sum of right motor positions",diffsAbsSidesPosSums);

        robotAngle = diffsAbsSidesPosSums/33; // Probably will need some tuning

        while (Math.abs(robotAngle)>180) { // This may get inefficient as many turns are made
            if (robotAngle < -180) {robotAngle -= -180;}
            else if (robotAngle > 180) {robotAngle += 180;}
        }

        if (robotAngle > 0) {yPower = robotAngle/-90 + 1;} // Calculate y power
        else {yPower = robotAngle/90 + 1;}

        if(robotAngle > 90) {xPower = robotAngle/90 -2;} // Calculate x power
        else if (robotAngle < -90) {xPower = robotAngle/90+2;}
        else {xPower = robotAngle/90;}

        double drivePower = 0.25;
        if (gamepad1.right_bumper) drivePower = 1;
        else if (gamepad1.left_bumper) drivePower = 0.25;

        DriveTrainBase(xPower,yPower,-gamepad1.right_stick_x,drivePower);
        DriveMicroAdjust(0.4);
        UpdateTelemetry();
    }

    @Utility.Encapsulate
    private void DriveTrainBase(double xPower, double yPower, double rPower, double powerCoef) {

        robot.lf.setPower((yPower + rPower - xPower) * powerCoef);
        robot.rf.setPower((-yPower + rPower - xPower) * powerCoef);
        robot.lb.setPower((yPower + rPower + xPower) * powerCoef);
        robot.rb.setPower((-yPower + rPower + xPower) * powerCoef);

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
