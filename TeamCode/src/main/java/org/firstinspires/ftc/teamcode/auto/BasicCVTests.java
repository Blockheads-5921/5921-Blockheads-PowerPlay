package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
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

import java.util.ArrayList;

@Autonomous(name = "ooga booga cv", group = "Robot")
public class BasicCVTests extends LinearOpMode {
    OpenCvCamera camera;
    BasicPipeline basicPipeline = new BasicPipeline();

    // AprilTagDetection tagOfInterest = null;

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

        telemetry.setMsTransmissionInterval(50);

        int[] HSVlow = new int[] {0,0,0};
        int[] HSVhigh = new int[] {360,0,0};

        int HSVMode = 0;

        int minOrMax = 0;

        while (!isStopRequested()) {
            telemetry.addLine("press Y to adjust hue, X to adjust saturation, A to adjust value.");
            telemetry.addLine("Use left joystick to actually adjust, and press B to switch between min and max values");

            // get hsv mode
            if (gamepad1.y) {
                HSVMode = 0;
            } else if (gamepad1.x) {
                HSVMode = 1;
            } else if (gamepad1.a) {
                HSVMode = 2;
            }

            // get min or max
            if (gamepad1.b) {
                if (minOrMax == 0) {minOrMax = 1;}
                else {minOrMax = 0;}
            }

            // actually adjust
            if (gamepad1.left_stick_y < 0.5) {
                if (minOrMax == 0) { // not gonna figure out 2d arrays now
                    HSVlow[HSVMode] += 5;
                    basicPipeline.setScalarValues(0, HSVlow[0], HSVlow[1], HSVlow[2]);
                } else if (minOrMax == 1) {
                    HSVhigh[HSVMode] += 5;
                    basicPipeline.setScalarValues(1, HSVhigh[0], HSVhigh[1], HSVhigh[2]);
                }
            } else if (gamepad1.left_stick_y > 0.5) {
                if (minOrMax == 0) { // i realise how horrible this is
                    HSVlow[HSVMode] -= 5;
                    basicPipeline.setScalarValues(0, HSVlow[0], HSVlow[1], HSVlow[2]);
                } else if (minOrMax == 1) {
                    HSVhigh[HSVMode] -= 5;
                    basicPipeline.setScalarValues(1, HSVhigh[0], HSVhigh[1], HSVhigh[2]);
                }
            } // i solemnly swear upon vim never to do this again

            telemetry.addData("Low scalar: ", HSVlow);
            telemetry.addData("High scalar: ", HSVhigh);
            telemetry.update();
        }
    }
}