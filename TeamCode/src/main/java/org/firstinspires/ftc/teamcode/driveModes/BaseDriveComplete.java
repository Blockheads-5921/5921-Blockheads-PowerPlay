package org.firstinspires.ftc.teamcode.driveModes;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.checkerframework.checker.signedness.qual.Constant;
import org.firstinspires.ftc.teamcode.common.Button;
import org.firstinspires.ftc.teamcode.common.Constants;
import org.firstinspires.ftc.teamcode.common.HardwareDrive;
import org.firstinspires.ftc.teamcode.common.Utility;

@TeleOp(name = "Base Drive Complete", group = "Drive")
//@Disabled
public class BaseDriveComplete extends LinearOpMode {
    /* Declare OpMode members. */
    HardwareDrive robot = new HardwareDrive();
    private final Constants constants = new Constants();
    private CRServo serv0;
    private final ElapsedTime runtime = new ElapsedTime();
    private final Button lifterButton = new Button();
    private final Button lifterBottomButton = new Button();
    private final boolean toggleButton = true;

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
        UpdateGripper();
        UpdatePlayers();
        UpdateTelemetry();
    }

    private void UpdatePlayers() {
        double drivePower = 0.4;
        if (gamepad1.right_bumper) drivePower = 1;
        DriveTrainBase(drivePower);
        DriveMicroAdjust(0.1);

    }
    @Utility.Encapsulate
    private void DriveTrainBase(double drivePower) {
        double directionX = Math.pow(gamepad1.left_stick_x, 1); // Strafe
        double directionY = Math.pow(gamepad1.left_stick_y, 1); // Forward
        double directionR = -Math.pow(gamepad1.right_stick_x, 1); // Turn
        double liftPower = Math.pow(gamepad2.right_stick_y, 1); // Lift
        int liftPos = robot.lift.getCurrentPosition();

        robot.lf.setPower((directionY + directionR - directionX) * drivePower);
        robot.rf.setPower((-directionY + directionR - directionX) * drivePower);
        robot.lb.setPower((directionY + directionR + directionX) * drivePower);
        robot.rb.setPower((-directionY + directionR + directionX) * drivePower);

        /*
                if (gamepad2.y == true) {MoveLiftTo(3); telemetry.addLine("Button y pressed");}
        if (gamepad2.x == true) {MoveLiftTo(2);}
        if (gamepad2.a == true) {MoveLiftTo(1);}
        if (gamepad2.b == true) {MoveLiftTo(0);}
        if (gamepad2.dpad_down || gamepad2.dpad_left || gamepad2.dpad_right || gamepad2.dpad_up) {MoveLiftTo(-1);}
         */

        // Make sure we're not letting lift over-extend...
        if (liftPos < Constants.elevatorPositionTop && gamepad2.right_stick_y < 0) {robot.lift.setPower((liftPower) * 0.1 - -0.001);} // YOU WILL DELETE MY CURLY BRACKETS OVER MY DEAD BODY!!!
        // or over-retract
        else if (liftPos > Constants.elevatorPositionBottom && gamepad2.right_stick_y > 0) {robot.lift.setPower((liftPower) * 0.01);}
        else {robot.lift.setPower((liftPower - 0.001) * 0.90);}

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

    private void UpdateGripper() {
        if (gamepad2.left_trigger > 0.01) serv0.setPower(0.18 * gamepad2.left_trigger - 0);
        else if  (gamepad2.right_trigger > 0.01) serv0.setPower(-0.1 * gamepad2.right_trigger + 0);
    }
    private void UpdateTelemetry() {
        telemetry.addData("g1.X", gamepad1.left_stick_x);
        telemetry.addData("g1.Y", -gamepad1.left_stick_y);
        telemetry.addData("g1.R", gamepad1.right_stick_x);
        telemetry.addData("Arm Position", robot.lift.getCurrentPosition());
        telemetry.addData("g2.L", gamepad2.right_stick_y);
        if (gamepad1.a && gamepad2.a) {
            telemetry.addLine("Players high fived!");
        }
        telemetry.update();
    }
}
