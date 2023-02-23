package org.firstinspires.ftc.teamcode.common;

/* any packages we need to import for our code */

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;

public class HardwareDrive {
    /* declare the motors and servo and set them to null */
    public DcMotorEx lf = null;
    public DcMotorEx rf = null;
    public DcMotorEx lb = null;
    public DcMotorEx rb = null;
    public DcMotorEx lift = null;
    public CRServo serv0;

    /* declare our gyro (imu) and camera */
    public BNO055IMU imu;
    public OpenCvCamera camera;
    public Orientation angles;
    public Acceleration gravity;

    /**
     * The initialization method used in every driveMode and
     * opMode to access our motors and sensors. Put this inside runOpMode
     * and pass to it the hardwareMap abstract.
     *
     * @param hwMap This parameter should be passed the abstract hardwareMap member in every LinearOpMode.
     */
    public void init(HardwareMap hwMap) {
        /* map each motor to a variable we can use in our code */
        lf = hwMap.get(DcMotorEx.class, "left_front");
        rf = hwMap.get(DcMotorEx.class, "right_front");
        lb = hwMap.get(DcMotorEx.class, "left_back");
        rb = hwMap.get(DcMotorEx.class, "right_back");
        lift = hwMap.get(DcMotorEx.class, "lift");
        serv0 = hwMap.get(CRServo.class, "serv0");

        /* all of our gyro initialization stuff */
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        /* find our webcam and initialize it */
        int cameraMonitorViewID = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hwMap.get(WebcamName.class, "log920"), cameraMonitorViewID);

        /* make sure all of our motors are going the right way */
        lf.setDirection(DcMotorEx.Direction.REVERSE);
        lb.setDirection(DcMotorEx.Direction.REVERSE);
        rf.setDirection(DcMotorEx.Direction.FORWARD);
        rb.setDirection(DcMotorEx.Direction.FORWARD);
        lift.setDirection(DcMotorEx.Direction.REVERSE);

        /* stop and reset the encoder */
        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        /* run each motor using the encoder so we can get data */
        lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}

