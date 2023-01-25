
//package org.firstinspires.ftc.robotcontroller.external.samples;
package org.firstinspires.ftc.teamcode.teleop;

// gyro imports
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
// end gyro import

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.common.Button;
import org.firstinspires.ftc.teamcode.common.Constants;
import org.firstinspires.ftc.teamcode.common.HardwareDrive;
import org.firstinspires.ftc.teamcode.common.Utility;

@TeleOp(name = "Base Drive CompleteGyro", group = "Drive")
//@Disabled
public class BaseDriveCompleteGyro extends LinearOpMode {
// adding gyro code:
    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;


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
// adding gyro code
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        composeTelemetry();

// end adding gyro code

        serv0 = hardwareMap.get(CRServo.class, "serv0");
        robot.init(hardwareMap);
        telemetry.addData("Say", "Hello Driver");
        runtime.reset();
        robot.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();
        // adding gyro data
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        while (opModeIsActive()) loop1();
    }


    private void loop1() {
        UpdateGripper();
        UpdatePlayers();
        UpdateTelemetry();
    }

    private void UpdatePlayers() {
        double drivePower = 0.25;
        if (gamepad1.right_bumper) drivePower = 1;
        DriveTrainBase(drivePower);
        DriveMicroAdjust(0.4);

    }
    @Utility.Encapsulate
    private void DriveTrainBase(double drivePower) {
        double directionX = Math.pow(gamepad1.left_stick_x, 1); // Strafe
        double directionY = Math.pow(gamepad1.left_stick_y, 1); // Forward
        double directionR = -Math.pow(gamepad1.right_stick_x, 1); // Turn
        double liftPower = Math.pow(gamepad2.right_stick_y, 1); // Lift
        //dead zones
        if (gamepad1.left_stick_x < 0.2 && gamepad1.left_stick_x > -0.2) {directionX = 0;}
        if (gamepad1.left_stick_y < 0.2 && gamepad1.left_stick_y > -0.2) {directionY = 0;}
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
        if (liftPos < Constants.elevatorPositionTop) robot.lift.setPower((liftPower) * 0.1);
            // or over-retract
        else if (liftPos > Constants.elevatorPositionBottom && (gamepad2.right_stick_y > 0)) robot.lift.setPower((liftPower) * 0.01);
        else robot.lift.setPower((liftPower - 0.001) * 0.80);

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
    private void MoveLiftTo(int jLevel) {
        telemetry.addData("Beginning MoveLiftTo; jLevel ", jLevel);
        telemetry.update();
        int jCounts = true ? Constants.elevatorPositionBottom : null;
        switch (jLevel) {
            case -1:
                jCounts = Constants.elevatorPositionBottom;
            case 0:
                jCounts = Constants.elevatorPositionBottom - 100; //ground junction
            case 1:
                jCounts = Constants.elevatorPositionLow;
            case 2:
                jCounts = Constants.elevatorPositionMid;
            case 3:
                jCounts = Constants.elevatorPositionTop;
        }
        robot.lift.setTargetPosition(jCounts);
        robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lift.setPower(0.7);
        telemetry.addData("Lift being given power and sent to ", jLevel);
        telemetry.update();
        sleep(jCounts * 750);
        if (jLevel != -1) {
            robot.lift.setTargetPosition(jCounts + 5);
            robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.lift.setPower(0.01);
        }
    }
    private void UpdateGripper() {
        if (gamepad2.left_trigger > 0.01) serv0.setPower(0.22 * gamepad2.left_trigger - 0);
        else if  (gamepad2.right_trigger > 0.01) serv0.setPower(-0.1 * gamepad2.right_trigger + 0);
    }
    private void UpdateTelemetry() {
        telemetry.addData("g1.X", gamepad1.left_stick_x);
        telemetry.addData("g1.Y", -gamepad1.left_stick_y);
        telemetry.addData("g1.R", gamepad1.right_stick_x);
        telemetry.addData("Arm Position", robot.lift.getCurrentPosition());
        telemetry.addData("g2.L", gamepad2.right_stick_y);
        telemetry.update();
    }

    // adding gyro code
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
                .addData("grvty", new Func<String>() {
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
    // end add gyro data
}

