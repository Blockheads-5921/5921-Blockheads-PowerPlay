package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "BasicTankDrive02 (Blocks to Java)")
@Disabled
public class BasicTankDrive02 extends LinearOpMode {

  private DcMotor LF;
  private DcMotor RB;
  private DcMotor RF;
  private DcMotor LB;

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    LF = hardwareMap.get(DcMotor.class, "LF");
    RB = hardwareMap.get(DcMotor.class, "RB");
    RF = hardwareMap.get(DcMotor.class, "RF");
    LB = hardwareMap.get(DcMotor.class, "LB");

    // Reverse one of the drive motors.
    // You will have to determine which motor to reverse for your robot.
    // In this example, the right motor was reversed so that positive
    // applied power makes it move the robot in the forward direction.
    LF.setDirection(DcMotorSimple.Direction.REVERSE);
    waitForStart();
    if (opModeIsActive()) {
      // Put run blocks here.
      while (opModeIsActive()) {
        // Put loop blocks here.
        // The Y axis of a joystick ranges from -1 in its topmost position
        // to +1 in its bottommost position. We negate this value so that
        // the topmost position corresponds to maximum forward power.
        RB.setPower(-0.5*gamepad1.left_stick_x);
        RF.setPower(-0.5*gamepad1.right_stick_x);
        // The Y axis of a joystick ranges from -1 in its topmost position
        // to +1 in its bottommost position. We negate this value so that
        // the topmost position corresponds to maximum forward power.
        LB.setPower(-0.5*gamepad1.left_stick_y);
        LF.setPower(-0.5*gamepad1.right_stick_y);
        telemetry.addData("LeftX Pow", RB.getPower());
        telemetry.addData("RightXPow", RF.getPower());
        telemetry.addData("LeftY Pow", LB.getPower());
        telemetry.addData("RightY Pow", LF.getPower());
        telemetry.update();
      }
    }
  }
}
