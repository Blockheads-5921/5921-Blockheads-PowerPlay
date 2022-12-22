package org.firstinspires.ftc.teamcode.common.auto;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.common.HardwareDrive;

import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

public class Utility {
    @Retention(RetentionPolicy.RUNTIME)
    @Target({ElementType.METHOD, ElementType.TYPE})
    public @interface Encapsulate {

    }

    public Utility resetEncoder(HardwareDrive robot) {
        robot.lf.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.lb.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.rf.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.rb.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.lf.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        robot.lb.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        robot.rf.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        robot.rb.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        return new Utility();
    }
}
