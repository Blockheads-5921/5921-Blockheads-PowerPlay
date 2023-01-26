package org.firstinspires.ftc.teamcode.teleop;
import com.acmerobotics.roadrunner.drive.Drive;
import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.checkerframework.checker.signedness.qual.Constant;
import org.firstinspires.ftc.teamcode.common.Button;
import org.firstinspires.ftc.teamcode.common.Constants;
import org.firstinspires.ftc.teamcode.common.HardwareDrive;
import org.firstinspires.ftc.teamcode.common.Utility;

@TeleOp(name = "Base Drive Complete", group = "Drive")
//@Disabled
public class BaseDriveComplete extends LinearOpMode {
    /* Declare OpMode members. */
    HardwareDrive robot = new HardwareDrive();
    private final Constants constants = new Constants();
    private CRServo serv0;
    private final ElapsedTime runtime = new ElapsedTime();
    private final Button lifterButton = new Button();
    private final Button lifterBottomButton = new Button();
    private final boolean toggleButton = true;
    // get goofy sounds
    int vineBoomSoundID = hardwareMap.appContext.getResources().getIdentifier("vineboom", "raw", hardwareMap.appContext.getPackageName());
    int runningSoundID = hardwareMap.appContext.getResources().getIdentifier("running", "raw", hardwareMap.appContext.getPackageName());
    int slidingSoundID = hardwareMap.appContext.getResources().getIdentifier("slide", "raw", hardwareMap.appContext.getPackageName());
    boolean soundPlayed = true;

    @Override
    public void runOpMode() {
        runtime.reset();
        serv0 = hardwareMap.get(CRServo.class, "serv0");
        robot.init(hardwareMap);
        robot.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, runningSoundID);

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

        // Make sure we're not letting lift over-extend...
        if (liftPos < Constants.elevatorPositionTop && gamepad2.right_stick_y < 0) {robot.lift.setPower((gamepad2.left_stick_y) * 0.1 - -0.001);}
        // or over-retract
        else if (liftPos > Constants.elevatorPositionBottom && gamepad2.right_stick_y > 0) {robot.lift.setPower((gamepad2.left_stick_y) * 0.01);}
        else {robot.lift.setPower((gamepad2.left_stick_y - 0.001) * 0.90);} // all this works because both the lift and left_stick_y are inverted

        if (gamepad2.right_stick_y < 0 && !soundPlayed) {
            SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, slidingSoundID);
            soundPlayed = true;
        }

        if (gamepad2.left_trigger > 0.01) {
            serv0.setPower(0.18 * gamepad2.left_trigger - 0);
            if (!soundPlayed) {
                SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, vineBoomSoundID);
                soundPlayed = true;
            }
        } else if (gamepad2.right_trigger > 0.01) {serv0.setPower(-0.1 * gamepad2.right_trigger + 0);}

        if (gamepad2.right_stick_y > 0 & gamepad2.left_trigger == 0 && gamepad2.right_trigger == 0) {soundPlayed = false;} //consider different soundPlayeds for different sounds?

        DriveMicroAdjust(0.2);
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
}