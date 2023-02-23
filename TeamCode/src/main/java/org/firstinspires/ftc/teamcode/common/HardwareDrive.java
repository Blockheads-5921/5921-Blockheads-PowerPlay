package org.firstinspires.ftc.teamcode.common;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;


public class HardwareDrive {
    public DcMotorEx lf = null;
    public DcMotorEx rf = null;
    public DcMotorEx lb = null;
    public DcMotorEx rb = null;
    public DcMotorEx lift = null;

    public BNO055IMU imu;
    public CRServo serv0;
    public OpenCvCamera camera;
    public Orientation angles;
    public Acceleration gravity;

    public void init(HardwareMap hwMap) {
        lf = hwMap.get(DcMotorEx.class, "left_front");
        rf = hwMap.get(DcMotorEx.class, "right_front");
        lb = hwMap.get(DcMotorEx.class, "left_back");
        rb = hwMap.get(DcMotorEx.class, "right_back");
        lift = hwMap.get(DcMotorEx.class, "lift");
        serv0 = hwMap.get(CRServo.class, "serv0");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        int cameraMonitorViewID = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hwMap.get(WebcamName.class, "log920"), cameraMonitorViewID);

        lf.setDirection(DcMotorEx.Direction.REVERSE);
        lb.setDirection(DcMotorEx.Direction.REVERSE);
        rf.setDirection(DcMotorEx.Direction.FORWARD);
        rb.setDirection(DcMotorEx.Direction.FORWARD);
        lift.setDirection(DcMotorEx.Direction.REVERSE);

        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}

