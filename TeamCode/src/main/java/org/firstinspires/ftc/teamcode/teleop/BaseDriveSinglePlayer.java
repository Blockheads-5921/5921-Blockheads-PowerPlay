package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.common.Constants;
import org.firstinspires.ftc.teamcode.common.HardwareDrive;
import org.firstinspires.ftc.teamcode.common.Utility;

import java.util.Locale;

@TeleOp(name = "Base Drive Single Player", group = "Drive")
public class BaseDriveSinglePlayer extends LinearOpMode {

    private final HardwareDrive robot = new HardwareDrive();
    private final ElapsedTime runtime = new ElapsedTime();

    private double drivePower = 0.40;

    @Override
    public void runOpMode() {
        composeTelemetry();
        robot.init(hardwareMap);
        telemetry.addData("Say", "Hello Driver");
        runtime.reset();
        robot.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        robot.imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        while (opModeIsActive()) {
            if (gamepad1.right_bumper) drivePower = 1;
            DriveTrainBase(drivePower);
            DriveMicroAdjust();
            UpdateGripper();
            UpdateTelemetry();
        }
    }

    @Utility.Encapsulate
    private void DriveTrainBase(double drivePower) {
        double directionX = Math.pow(gamepad1.left_stick_x, 1);
        double directionY = Math.pow(gamepad1.left_stick_y, 1);
        double directionR = -Math.pow(gamepad1.right_stick_x, 1);

        if (gamepad1.left_stick_x < 0.2 && gamepad1.left_stick_x > -0.2) directionX = 0;
        if (gamepad1.left_stick_y < 0.2 && gamepad1.left_stick_y > -0.2) directionY = 0;
        int liftPos = robot.lift.getCurrentPosition();

        robot.lf.setPower((directionY + directionR - directionX) * drivePower);
        robot.rf.setPower((-directionY + directionR - directionX) * drivePower);
        robot.lb.setPower((directionY + directionR + directionX) * drivePower);
        robot.rb.setPower((-directionY + directionR + directionX) * drivePower);

        if (liftPos < Constants.elevatorPositionTop && gamepad1.left_bumper) {
            robot.lift.setPower((0.5 * 0.1 + 0.001));
        }
        else if (liftPos > Constants.elevatorPositionBottom && gamepad1.left_trigger == 1) {
            robot.lift.setPower((0.5) * 0.01);
        } else robot.lift.setPower((0.5 - 0.001) * 1);
    }
    private void DriveMicroAdjust() {
        if (gamepad1.dpad_up) {
            robot.lf.setPower(-0.4);
            robot.rf.setPower(+0.4);
            robot.lb.setPower(-0.4);
            robot.rb.setPower(+0.4);
        } else if (gamepad1.dpad_down) {
            robot.lf.setPower(+0.4);
            robot.rf.setPower(-0.4);
            robot.lb.setPower(+0.4);
            robot.rb.setPower(-0.4);
        } else if (gamepad1.dpad_right) {
            robot.lf.setPower(0.4);
            robot.rf.setPower(0.4);
            robot.lb.setPower(0.4);
            robot.rb.setPower(0.4);
        } else if (gamepad1.dpad_left) {
            robot.lf.setPower(-0.4);
            robot.rf.setPower(-0.4);
            robot.lb.setPower(-0.4);
            robot.rb.setPower(-0.4);
        }
    }
    public void UpdateGripper() {
        if (gamepad1.right_trigger >= 0.9) robot.serv0.setPower(0.22 * 0.5);
        else robot.serv0.setPower(-0.16 * 0.5);
    }

    @Utility.Encapsulate
    public void UpdateTelemetry() {
        telemetry.addData("g1.X", gamepad1.left_stick_x);
        telemetry.addData("g1.Y", -gamepad1.left_stick_y);
        telemetry.addData("g1.R", gamepad1.right_stick_x);
        telemetry.addData("Arm Position", robot.lift.getCurrentPosition());
        telemetry.update();
    }
    public void composeTelemetry() {
        telemetry.addAction(() -> {
            robot.angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            robot.gravity = robot.imu.getGravity();
        });

        telemetry.addLine().addData("status", () -> robot.imu.getSystemStatus().toShortString()).addData("calib", () -> robot.imu.getCalibrationStatus().toString());
        telemetry.addLine().addData("heading", () -> formatAngle(robot.angles.angleUnit, robot.angles.firstAngle)).addData("roll", () -> formatAngle(robot.angles.angleUnit, robot.angles.secondAngle)).addData("pitch", () -> formatAngle(robot.angles.angleUnit, robot.angles.thirdAngle));
        telemetry.addLine().addData("grvty", () -> robot.gravity.toString()).addData("mag", () -> String.format(Locale.getDefault(), "%.3f", Math.sqrt(robot.gravity.xAccel * robot.gravity.xAccel + robot.gravity.yAccel * robot.gravity.yAccel + robot.gravity.zAccel * robot.gravity.zAccel)));
    }

    String formatDegrees(double degrees) { return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees)); }
    String formatAngle(AngleUnit angleUnit, double angle) { return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle)); }
}