package org.firstinspires.ftc.teamcode.common;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static java.lang.Thread.sleep;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

// Import vuforia related stuff
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import org.firstinspires.ftc.teamcode.common.HardwareDrive;
import org.firstinspires.ftc.teamcode.common.Constants;

import java.util.List;


public abstract class AutoMethods extends LinearOpMode {
    // vuforia stuff
    private static final String TFOD_MODEL_ASSET = "PowerPlay.tflite";
    private static final String[] LABELS = {
            "1 Bolt",
            "2 Bulb",
            "3 Panel"
    };
    private static final String VUFORIA_KEY =
            "AaKd3K3/////AAABmbEXeRKKtUzcqa1Tm5CDOkdJ5/gdsHlZH8to976NByBP9kxz11aApGntLM40oaXUm4wvKkzhoz8wmwEOHWBce+Mx3M1l/8uQ3Ys4BZoWEKt+b+LCVCOg8oxNplp3WJFw0jgVLetgTfL/NFtI4jlygxlPkgaHlRKFWyJiZ5nX84+brBTBCL1rekx78AsFElJbTgeI+GNr8GNjnIuXhRQ+WWDiriWD04abmrmyh4HoVVTmElg+/9zdKW9seASyq/4/Z4kQt9ViprVbSD2MjxgBetXJ4PTWm4KAzkJE/gejgoiCSPGKecdCav+un5wFPRw5Y/AogVpBHDWeHmXHkgfG+NiMwK/2Ebyu3/PsLdyNDJxd";
    // possibly illegal
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;


    public HardwareDrive robot;
    public HardwareMap hardWareMap;
    public ElapsedTime runtime = new ElapsedTime();
    public Constants constants = new Constants();
    public CRServo serv0;

    public void DepositCone(int junctionLevel) throws InterruptedException {
        //assumes lift is at bottom
        int targetPos = 0;
        switch (junctionLevel) {
            case 1:
                targetPos = Constants.elevatorPositionLow;
                break;
            case 2:
                targetPos = Constants.elevatorPositionMid;
                break;
            case 3:
                targetPos = Constants.elevatorPositionTop;
                break;
        }
        //raise arm
        robot.lift.setTargetPosition(targetPos);
        robot.lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lift.setPower(1.00);
        sleep(2200);
        robot.lift.setPower(0); //Brake arm, maybe unnecessary?
        //Drive forward
        SetBrakes(false);
        DriveForward(160,0.15);
        //Lower arm
        robot.lift.setTargetPosition(Constants.elevatorPositionAboveCone);
        robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lift.setPower(0.66);
        sleep(1500);
        //Release cone
        serv0.setPower(0.18);
        //Back up
        DriveReverse(160,0.7);
        sleep(250);
        //lower arm fully
        robot.lift.setTargetPosition(Constants.elevatorPositionBottom);
        robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lift.setPower(0.75);
        sleep(750);
        SetBrakes(true);
    }

    public int getConeImage() {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();
        initTfod();

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(1.0, 16.0 / 9.0); //zoom image, could be very useful
        }

        String recognizedImage = null;
        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                telemetry.addData("# Objects Detected", updatedRecognitions.size());

                // step through the list of recognitions and display image position/size information for each one
                // Note: "Image number" refers to the randomized image orientation/number
                for (Recognition recognition : updatedRecognitions) {
                    double col = (recognition.getLeft() + recognition.getRight()) / 2;
                    double row = (recognition.getTop() + recognition.getBottom()) / 2;
                    double width = Math.abs(recognition.getRight() - recognition.getLeft());
                    double height = Math.abs(recognition.getTop() - recognition.getBottom());
                    recognizedImage = recognition.getLabel(); //why does this work?

                    telemetry.addData("", " ");
                    telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
                    telemetry.addData("- Position (Row/Col)", "%.0f / %.0f", row, col);
                    telemetry.addData("- Size (Width/Height)", "%.0f / %.0f", width, height);
                    telemetry.update();
                }
            }
        }
        int picNum = -1;
        switch (recognizedImage) {
            case "1 Bolt":
                picNum = 1;
                break;
            case "2 Bulb":
                picNum = 2;
                break;
            case "3 Panels":
                picNum = 3;
                break;
        }
        return picNum;
    }

    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "log920");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.75f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 300;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);

        // Use loadModelFromAsset() if the TF Model is built in as an asset by Android Studio
        // Use loadModelFromFile() if you have downloaded a custom team model to the Robot Controller's FLASH.
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
        // tfod.loadModelFromFile(TFOD_MODEL_FILE, LABELS);
    }

    public void SetBrakes(boolean brakesOn) {
        if (brakesOn) {
            robot.lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        else {
            robot.lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            robot.rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            robot.lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            robot.rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
    }

    public void StrafeRight(int straferightEncoderPulses, double drivePower) {
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
        while (/*opModeIsActive() && */(robot.lf.isBusy())) {
            telemetry.addData("Running to", " %7d ", straferightEncoderPulses);
            telemetry.addData("Currently at", " at %7d", robot.lf.getCurrentPosition());
            telemetry.update();
            RobotLog.d("StrafeRight: Encoders: %7d,%7d,%7d,%7d", robot.lf.getCurrentPosition(), robot.rf.getCurrentPosition(), robot.lb.getCurrentPosition(), robot.rb.getCurrentPosition());
        }
    }

    public void StrafeLeft(int strafeleftEncoderPulses, double drivePower) {
        robot.lf.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.lb.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.rf.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.rb.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.lf.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        robot.lb.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        robot.rf.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        robot.rb.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        robot.lf.setTargetPosition(strafeleftEncoderPulses);
        robot.rf.setTargetPosition(strafeleftEncoderPulses);
        robot.lb.setTargetPosition(-strafeleftEncoderPulses);
        robot.rb.setTargetPosition(-strafeleftEncoderPulses);

        robot.lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lf.setPower(drivePower);
        robot.rf.setPower(drivePower);
        robot.lb.setPower(drivePower);
        robot.rb.setPower(drivePower);

        while (/*opModeIsActive() && */(robot.lf.isBusy())) {
            telemetry.addData("Running to", " %7d ", strafeleftEncoderPulses);
            telemetry.addData("Currently at", " at %7d", robot.lf.getCurrentPosition());
            telemetry.update();
            RobotLog.d("StrafeLeft: Encoders: %7d,%7d,%7d,%7d", robot.lf.getCurrentPosition(), robot.rf.getCurrentPosition(), robot.lb.getCurrentPosition(), robot.rb.getCurrentPosition());
        }
    }

    public void SpinLeft(int spinleftEncoderPulses, double drivePower) {

        robot.lf.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.lb.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.rf.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.rb.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.lf.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        robot.lb.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        robot.rf.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        robot.rb.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        robot.lf.setTargetPosition(spinleftEncoderPulses);
        robot.rf.setTargetPosition(spinleftEncoderPulses);
        robot.lb.setTargetPosition(spinleftEncoderPulses);
        robot.rb.setTargetPosition(spinleftEncoderPulses);

        robot.lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lf.setPower(drivePower);
        robot.rf.setPower(drivePower);
        robot.lb.setPower(drivePower);
        robot.rb.setPower(drivePower);

        while (/*opModeIsActive() && */(robot.lf.isBusy())) {
            telemetry.addData("Running to", " %7d ", spinleftEncoderPulses);
            telemetry.addData("Currently at", " at %7d", robot.lf.getCurrentPosition());
            telemetry.update();
            RobotLog.d("SpinLeft: Encoders: %7d,%7d,%7d,%7d", robot.lf.getCurrentPosition(), robot.rf.getCurrentPosition(), robot.lb.getCurrentPosition(), robot.rb.getCurrentPosition());
        }
    }

    public void SpinRight(int spinrightEncoderPulses, double drivePower) {
        robot.lf.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.lb.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.rf.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.rb.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.lf.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        robot.lb.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        robot.rf.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        robot.rb.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        robot.lf.setTargetPosition(-spinrightEncoderPulses);
        robot.rf.setTargetPosition(-spinrightEncoderPulses);
        robot.lb.setTargetPosition(-spinrightEncoderPulses);
        robot.rb.setTargetPosition(-spinrightEncoderPulses);

        robot.lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lf.setPower(drivePower);
        robot.rf.setPower(drivePower);
        robot.lb.setPower(drivePower);
        robot.rb.setPower(drivePower);

        while (/*opModeIsActive() && */(robot.lf.isBusy())) {
            telemetry.addData("Running to", " %7d ", spinrightEncoderPulses);
            telemetry.addData("Currently at", " at %7d", robot.lf.getCurrentPosition());
            telemetry.update();
            RobotLog.d("SpinRight: Encoders: %7d,%7d,%7d,%7d", robot.lf.getCurrentPosition(), robot.rf.getCurrentPosition(), robot.lb.getCurrentPosition(), robot.rb.getCurrentPosition());
        }
    }

    public void DriveForward(int forwardEncoderPulses, double drivePower) {
        robot.lf.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.lb.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.rf.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.rb.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.lf.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        robot.lb.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        robot.rf.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        robot.rb.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        robot.lf.setTargetPosition(-forwardEncoderPulses);
        robot.rf.setTargetPosition(+forwardEncoderPulses);
        robot.lb.setTargetPosition(-forwardEncoderPulses);
        robot.rb.setTargetPosition(+forwardEncoderPulses);

        robot.lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lf.setPower(drivePower);
        robot.rf.setPower(drivePower);
        robot.lb.setPower(drivePower);
        robot.rb.setPower(drivePower);

        while (/*opModeIsActive() && */(robot.lf.isBusy())) {
            telemetry.addData("Running to", " %7d ", forwardEncoderPulses);
            telemetry.addData("Currently at", " at %7d", robot.lf.getCurrentPosition());
            telemetry.update();
            RobotLog.d("Forward: Encoders: %7d,%7d,%7d,%7d", robot.lf.getCurrentPosition(), robot.rf.getCurrentPosition(), robot.lb.getCurrentPosition(), robot.rb.getCurrentPosition());
        }
    }

    public void DriveReverse(int reverseEncoderPulses, double drivePower){
        robot.lf.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.lb.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.rf.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.rb.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.lf.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        robot.lb.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        robot.rf.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        robot.rb.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        robot.lf.setTargetPosition(reverseEncoderPulses);
        robot.rf.setTargetPosition(-reverseEncoderPulses);
        robot.lb.setTargetPosition(reverseEncoderPulses);
        robot.rb.setTargetPosition(-reverseEncoderPulses);

        robot.lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lf.setPower(drivePower);
        robot.rf.setPower(drivePower);
        robot.lb.setPower(drivePower);
        robot.rb.setPower(drivePower);

        while (/*opModeIsActive() && */(robot.lf.isBusy())) {
            telemetry.addData("Running to", " %7d ", reverseEncoderPulses);
            telemetry.addData("Currently at", " at %7d", robot.lf.getCurrentPosition());
            telemetry.update();
            RobotLog.d("Reverse: Encoders: %7d,%7d,%7d,%7d", robot.lf.getCurrentPosition(), robot.rf.getCurrentPosition(), robot.lb.getCurrentPosition(), robot.rb.getCurrentPosition());
        }
    }
}
