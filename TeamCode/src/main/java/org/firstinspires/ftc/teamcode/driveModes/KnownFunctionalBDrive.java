package org.firstinspires.ftc.teamcode.driveModes;

import android.view.View;

// wat

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.Button;
import org.firstinspires.ftc.teamcode.common.Constants;
import org.firstinspires.ftc.teamcode.common.HardwareDrive;

@TeleOp(name = "KnownFunctionalBDrive", group = "Drive")
// @Disabled
public class KnownFunctionalBDrive extends LinearOpMode {
    /* Declare OpMode members. */
    HardwareDrive robot = new HardwareDrive();
    // Constants constants = new Constants();
    private CRServo serv0;
    private ElapsedTime runtime = new ElapsedTime();

    Button lifterButton = new Button();
    Button lifterBottomButton = new Button();

    private boolean toggleButton = true;

    /**
     * The relativeLayout field is used to aid in providing interesting visual
     * feedback
     * in this sample application; you probably *don't* need this when you use a
     * color sensor on your
     * robot. Note that you won't see anything change on the Driver Station, only on
     * the Robot Controller.
     */
    View relativeLayout;

    @Override
    public void runOpMode() {
        serv0 = hardwareMap.get(CRServo.class, "serv0");
        robot.init(hardwareMap);
        telemetry.addData("Say", "Hello Driver");
        runtime.reset();
        robot.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        while (opModeIsActive()) {
            loop1();
        }

    }

    private void loop1() {
        telemetry.addData("Ending encoder values: ", "lf %d lb %d rf %d rb %d",
                robot.lf.getCurrentPosition(), robot.lb.getCurrentPosition(), robot.rf.getCurrentPosition(), robot.rb.getCurrentPosition());
        telemetry.update();
    }
}