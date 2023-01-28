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

@TeleOp(name = "HSV Adjustments", group = "Drive")
//@Disabled
public class HSVAdjuster extends LinearOpMode {
    /* Declare OpMode members. */
    HardwareDrive robot = new HardwareDrive();
    private final Constants constants = new Constants();
    private CRServo serv0;
    private final ElapsedTime runtime = new ElapsedTime();
    private final Button lifterButton = new Button();
    private final Button lifterBottomButton = new Button();
    private final boolean toggleButton = true;

    int[] HSVlow = new int[] {0,0,0};
    int[] HSVhigh = new int[] {360,0,0};

    int HSVMode = 0;

    int minOrMax = 0;

    @Override
    public void runOpMode() {
        runtime.reset();
        serv0 = hardwareMap.get(CRServo.class, "serv0");
        robot.init(hardwareMap);
        robot.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        waitForStart();

        while (opModeIsActive()) loop1();
    }
    private void loop1() {
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
            } else if (minOrMax == 1) {
                HSVhigh[HSVMode] += 5;
            }
        } else if (gamepad1.left_stick_y > 0.5) {
            if (minOrMax == 0) { // i realise how horrible this is
                HSVlow[HSVMode] -= 5;
            } else if (minOrMax == 1) {
                HSVhigh[HSVMode] -= 5;
            }
        }
    }
}
