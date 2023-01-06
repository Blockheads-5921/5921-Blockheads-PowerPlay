package org.firstinspires.ftc.teamcode.driveModes;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
//Gyro stuff
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import java.util.Locale;
//Our stuff
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
    //Gyro stuff
    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;

    int lTgtPos = 0;
    double robotAngle = 0;
    double joystickAngle = 0;
    double tgtAngle = 0;
    double xPower = 0;
    double yPower = 0;
    double rPower = 0;
    double dirOffset = 0;

    @Override
    public void runOpMode() {
        serv0 = hardwareMap.get(CRServo.class, "serv0");
        robot.init(hardwareMap);
        telemetry.addData("Say", "Hello Driver");
        runtime.reset();
        //Gyro stuff
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        // composeTelemetry();

        robot.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.lf.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.lb.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.rf.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.rb.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        robot.lf.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        robot.lb.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        robot.rf.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        robot.rb.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        waitForStart();
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
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

        /*
        //get side lengths of triangle
        double a = 1;
        double b = Math.pow((Math.pow(gamepad1.left_stick_x,2) + Math.pow(gamepad1.left_stick_y, 2)),0.5);
        double c = Math.pow((Math.pow(gamepad1.left_stick_x-1,2) + Math.pow(gamepad1.left_stick_y-1, 2)),0.5);

        telemetry.addData("Joystick pushed-ness: ", b);

        joystickAngle = Math.acos((Math.pow(a,2)+Math.pow(b,2)-Math.pow(c,2))/2*a*b); // find joystick angle
        if (gamepad1.left_stick_x < 0) {joystickAngle *= -1;} // because the above formula always outputs a positive angle

         */

        joystickAngle = Math.acos(gamepad1.left_stick_x/gamepad1.left_stick_y);
        double joystickAmount = Math.pow((Math.pow(gamepad1.left_stick_x,2) + Math.pow(gamepad1.left_stick_y, 2)),0.5);


        telemetry.addData("Angle of joystick: ", joystickAngle);

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        robotAngle = angles.firstAngle; // this is the angle between vertical, from driver's pov, and robot

        telemetry.addData("Angle of robot: ", robotAngle);

        tgtAngle = robotAngle - joystickAngle; //subtract angles

        telemetry.addData("tgtAngle: ", tgtAngle);

        if (tgtAngle > 0) {yPower = tgtAngle/-90 + 1;} // Calculate y power
        else {yPower = tgtAngle/90 + 1;}

        if(tgtAngle > 90) {xPower = tgtAngle/90 -2;} // Calculate x power
        else if (tgtAngle < -90) {xPower = tgtAngle/90+2;}
        else {xPower = tgtAngle/90;}

        double drivePower = 0.25;
        if (gamepad1.right_bumper) drivePower = 1;

        telemetry.addData("X power: ", xPower);
        telemetry.addData("Y power: ", yPower);

        DriveTrainBase(joystickAmount*xPower,joystickAmount*yPower,-gamepad1.right_stick_x,drivePower);
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
        telemetry.addLine("\n\n");
        telemetry.addData("g1.X", gamepad1.left_stick_x);
        telemetry.addData("g1.Y", -gamepad1.left_stick_y);
        telemetry.addData("g1.R", gamepad1.right_stick_x);
        telemetry.addData("Arm Position", robot.lift.getCurrentPosition());
        telemetry.addData("g2.L", gamepad2.right_stick_y);
        telemetry.update();
    }

    //Gyro stuff

    public void composeTelemetry() {

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() { @Override public void run()
        {
            // Acquiring the angles is relatively expensive; we don't want
            // to do that in each of the three items that need that info, as that's
            // three times the necessary expense.
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            gravity  = imu.getGravity();
        }
        });

        telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override public String value() {
                        return imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override public String value() {
                        return imu.getCalibrationStatus().toString();
                    }
                });

        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });

        telemetry.addLine()
                .addData("grav", new Func<String>() {
                    @Override public String value() {
                        return gravity.toString();
                    }
                })
                .addData("mag", new Func<String>() {
                    @Override public String value() {
                        return String.format(Locale.getDefault(), "%.3f",
                                Math.sqrt(gravity.xAccel*gravity.xAccel
                                        + gravity.yAccel*gravity.yAccel
                                        + gravity.zAccel*gravity.zAccel));
                    }
                });
    }
    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
}
