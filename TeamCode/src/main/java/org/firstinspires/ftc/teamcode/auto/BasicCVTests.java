package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.auto.BasicPipeline;
import org.firstinspires.ftc.teamcode.common.Constants;
import org.firstinspires.ftc.teamcode.common.HardwareDrive;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import org.opencv.core.*;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

@Autonomous(name = "Basic CV Tests", group = "Robot")
public class BasicCVTests extends LinearOpMode {
    OpenCvCamera camera;
    BasicPipeline basicPipeline = new BasicPipeline();

    Constants constants = new Constants();
    HardwareDrive robot = new HardwareDrive();
    private CRServo serv0;
    private final ElapsedTime runtime = new ElapsedTime();



    @Override
    public void runOpMode() {

        robot.init(hardwareMap);
        serv0 = hardwareMap.get(CRServo.class, "serv0");

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "log920"), cameraMonitorViewId);
        basicPipeline = new BasicPipeline();

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

        Point junctionLocation = new Point();

        while(!isStarted() && !isStopRequested()) {
            telemetry.addData("Number of contours found: ", basicPipeline.getContourQuantity());

            junctionLocation = basicPipeline.getJunctionPoint();

            telemetry.addData("x of biggest contour: ", junctionLocation.x);
            telemetry.addData("y of biggest contour: ", junctionLocation.y);
            telemetry.addData("Size of biggest contour: ", basicPipeline.getBigJunctionSizeAttr());

            telemetry.update();
        }

        telemetry.addLine("Position robot now!");
        telemetry.update();
        sleep(4000);

        junctionLocation = basicPipeline.getJunctionPoint();
        telemetry.addData("Adjusting position; Last junction x was", junctionLocation.x);
        telemetry.update();
        StrafeRight((int) junctionLocation.x-300, 25);
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
        }
    }
}