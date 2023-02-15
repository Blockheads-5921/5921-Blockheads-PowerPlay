package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.drive.Drive;
import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.opencv.core.*;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.FtcDashboard;

import org.checkerframework.checker.signedness.qual.Constant;
import org.firstinspires.ftc.teamcode.auto.BasicPipeline;
import org.firstinspires.ftc.teamcode.common.Button;
import org.firstinspires.ftc.teamcode.common.Constants;
import org.firstinspires.ftc.teamcode.common.HardwareDrive;
import org.firstinspires.ftc.teamcode.common.Utility;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@TeleOp(name = "Base Drive Complete", group = "Drive")
//@Disabled
public class BaseDriveComplete extends LinearOpMode {
    /* Declare OpMode members. */
    HardwareDrive robot = new HardwareDrive();
    private final Constants constants = new Constants();
    private CRServo serv0;
    private final ElapsedTime runtime = new ElapsedTime();
    private FtcDashboard dashboard = FtcDashboard.getInstance();
    BasicPipeline basicPipeline = new BasicPipeline();
    Point junctionLocation = new Point();
    OpenCvCamera camera;

    @Override
    public void runOpMode() {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, dashboard.getTelemetry());
        runtime.reset();
        serv0 = hardwareMap.get(CRServo.class, "serv0");
        robot.init(hardwareMap);
        robot.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "log920"), cameraMonitorViewId);

        camera.setPipeline(basicPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
            }
        });

        waitForStart();
        while (opModeIsActive()) loop1();
    }

    private void loop1() {
        double drivePower = 0.4;
        if (gamepad1.right_bumper) drivePower = 1;

        int liftPos = robot.lift.getCurrentPosition();

        robot.lf.setPower((gamepad1.left_stick_y + -gamepad1.right_stick_x - gamepad1.left_stick_x) * drivePower);
        robot.rf.setPower((-gamepad1.left_stick_y + -gamepad1.right_stick_x - gamepad1.left_stick_x) * drivePower);
        robot.lb.setPower((gamepad1.left_stick_y + -gamepad1.right_stick_x + gamepad1.left_stick_x) * drivePower);
        robot.rb.setPower((-gamepad1.left_stick_y + -gamepad1.right_stick_x + gamepad1.left_stick_x) * drivePower);

        // Make sure we're not letting lift over-extend...kenny wuz here
        if (liftPos < Constants.elevatorPositionTop && gamepad2.right_stick_y < 0) {
            robot.lift.setPower((gamepad2.left_stick_y) * 0.1 - -0.001);
        }
        // or over-retract
        else if (liftPos > Constants.elevatorPositionBottom && gamepad2.right_stick_y > 0) {
            robot.lift.setPower((gamepad2.left_stick_y) * 0.01);
        } else {
            robot.lift.setPower((gamepad2.left_stick_y - 0.001) * 1.00);
        } // all this works because both the lift and left_stick_y are inverted

        if (gamepad2.left_trigger > 0.01) {
            serv0.setPower(0.25 * gamepad2.left_trigger - 0);
        } else if (gamepad2.right_trigger > 0.01) {
            serv0.setPower(-0.15 * gamepad2.right_trigger + 0);
        }

        if (gamepad2.right_stick_y > 0 & gamepad2.left_trigger == 0 && gamepad2.right_trigger == 0)
            DriveMicroAdjust(0.2);
        if (gamepad1.left_bumper) {
            junctionLocation = basicPipeline.getJunctionPoint();
            StrafeRight((int) junctionLocation.x - 300, 25);
        }
        UpdateTelemetry();
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
        if (gamepad1.a && gamepad2.a) {
            telemetry.addLine("Players high fived!");
        }
        telemetry.update();
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
    }

}
