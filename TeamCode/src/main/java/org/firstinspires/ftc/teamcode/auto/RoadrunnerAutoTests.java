package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.Constants;
import org.firstinspires.ftc.teamcode.common.HardwareDrive;

public class RoadrunnerAutoTests extends LinearOpMode {
    Constants constants = new Constants();
    HardwareDrive robot = new HardwareDrive();
    private final ElapsedTime runtime = new ElapsedTime();
    private CRServo serv0;

    @Override
    public void runOpMode() throws InterruptedException {

    }
}
