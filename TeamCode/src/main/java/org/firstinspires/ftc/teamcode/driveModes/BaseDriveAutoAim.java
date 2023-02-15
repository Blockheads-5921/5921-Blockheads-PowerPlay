package org.firstinspires.ftc.teamcode.driveModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.common.Constants;
import org.firstinspires.ftc.teamcode.common.HardwareDrive;

import java.util.List;

@Config
@TeleOp(name="Base Drive Aim", group="Drive")
public class BaseDriveAutoAim extends LinearOpMode {
    HardwareDrive robot = new HardwareDrive();
    private CRServo serv0;
    private final ElapsedTime runtime = new ElapsedTime();
    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    private final String VUFORIA_KEY = "something";

    @Override
    public void runOpMode() {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, dashboard.getTelemetry());
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
        UpdateAutoAim();
    }

    private void DriveTrainBase(double drivePower) {
        double directionX = Math.pow(gamepad1.left_stick_x, 1); // Strafe
        double directionY = Math.pow(gamepad1.left_stick_y, 1); // Forward
        double directionR = -Math.pow(gamepad1.right_stick_x, 1); // Turn
        double liftPower = Math.pow(gamepad2.right_stick_y, 1); // Lift
        //dead zones
        if (gamepad1.left_stick_x < 0.2 && gamepad1.left_stick_x > -0.2) directionX = 0;
        if (gamepad1.left_stick_y < 0.2 && gamepad1.left_stick_y > -0.2) directionY = 0;
        int liftPos = robot.lift.getCurrentPosition();

        robot.lf.setPower((directionY + directionR - directionX) * drivePower);
        robot.rf.setPower((-directionY + directionR - directionX) * drivePower);
        robot.lb.setPower((directionY + directionR + directionX) * drivePower);
        robot.rb.setPower((-directionY + directionR + directionX) * drivePower);

        if (liftPos < Constants.elevatorPositionTop && (gamepad2.right_stick_y < 0)) {
            robot.lift.setPower((liftPower) * 0.01);
        } else if (liftPos > Constants.elevatorPositionBottom && (gamepad2.right_stick_y > 0)) {
            robot.lift.setPower((liftPower) * 0.01);
        } else robot.lift.setPower((liftPower - 0.001) * 0.80);
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
    private void RunAutoAim() {

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.75f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 300;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
//        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);

        if (tfod != null) {
            tfod.activate();
            assert tfod != null;
            tfod.setZoom(1.0, 16.0/9.0);
        }
        if (tfod != null) {
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                telemetry.addData("# Objects Detected", updatedRecognitions.size());

                for (Recognition recognition : updatedRecognitions) {
                    double col = (recognition.getLeft() + recognition.getRight()) / 2 ;
                    double row = (recognition.getTop()  + recognition.getBottom()) / 2 ;
                    double width  = Math.abs(recognition.getRight() - recognition.getLeft()) ;
                    double height = Math.abs(recognition.getTop()  - recognition.getBottom()) ;

                    telemetry.addData(""," ");
                    telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100 );
                    telemetry.addData("- Position (Row/Col)","%.0f / %.0f", row, col);
                    telemetry.addData("- Size (Width/Height)","%.0f / %.0f", width, height);
                }
                telemetry.update();
            }
        }


    }

    private void UpdatePlayers() {
        double drivePower = 0.35;
        if (gamepad1.right_bumper) drivePower = 1;

        DriveTrainBase(drivePower);
        DriveMicroAdjust(drivePower);
    }
    private void UpdateGripper() {
        if (gamepad2.left_trigger > 0.01) serv0.setPower(0.22 * gamepad2.left_trigger - 0);
        else if  (gamepad2.right_trigger > 0.01) serv0.setPower(-0.1 * gamepad2.right_trigger + 0);
    }
    private void UpdateTelemetry() {
        telemetry.addData("gamepad1 | left-stick x", gamepad1.left_stick_x);
        telemetry.addData("gamepad1 | left-stick y", -gamepad1.left_stick_y);
        telemetry.addData("gamepad1 | right-stick x", gamepad1.right_stick_x);
        telemetry.addData("gamepad2 | right-stick y", gamepad2.right_stick_y);
        telemetry.addData("arm position", robot.lift.getCurrentPosition());
        telemetry.addData("1/ amount turned", (Math.abs(robot.lf.getCurrentPosition() + robot.lb.getCurrentPosition())
                - Math.abs(robot.rf.getCurrentPosition() + robot.rb.getCurrentPosition())));
        telemetry.addData("gyro angle", "idk");
        telemetry.update();
    }
    private void UpdateAutoAim() {
        if (gamepad1.left_bumper) {
            RunAutoAim();
        }
    }
}
