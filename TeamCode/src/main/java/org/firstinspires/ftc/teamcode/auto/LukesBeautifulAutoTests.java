package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.common.HardwareDrive;
import org.firstinspires.ftc.teamcode.common.Utility;

// TODO: Make this work??
@Utility.Encapsulate
public class LukesBeautifulAutoTests extends LinearOpMode {
    HardwareDrive robot = new HardwareDrive();
    private CRServo serv0;
    private final Utility robotMethods = new Utility(robot, serv0);

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        serv0 = hardwareMap.get(CRServo.class, "serv0");
        telemetry.addData("Status", "Ready to run");
        telemetry.update();

        waitForStart();
        RobotLog.d("5921", "Step4");

        // haha, beautiful one-liners 🙂🙂🙂
        robotMethods.DriveForward(240, 0.4, opModeIsActive());
        robotMethods.DriveReverse(240, 0.4, opModeIsActive());
        robotMethods.SpinLeft(240, 0.4, opModeIsActive());
        Utility.Useless.happiHappiYay("Everything. Me so happi");
    }
}
