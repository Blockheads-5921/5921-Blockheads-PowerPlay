package org.firstinspires.ftc.teamcode.driveModes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "BasicTankDrive03 (Blocks to Java)")
@Disabled
public class BasicTankDrive03 extends LinearOpMode {

  private DcMotor LF;
  private CRServo serv0;
  private CRServo serv1;
  private DcMotor Elevator;
  private DcMotor spare01;
  private DcMotor RB;
  private DcMotor RF;
  private DcMotor LB;

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    LF = hardwareMap.get(DcMotor.class, "LF");
    serv0 = hardwareMap.get(CRServo.class, "serv0");
    serv1 = hardwareMap.get(CRServo.class, "serv1");
    Elevator = hardwareMap.get(DcMotor.class, "Elevator");
    spare01 = hardwareMap.get(DcMotor.class, "spare01");
    RB = hardwareMap.get(DcMotor.class, "RB");
    RF = hardwareMap.get(DcMotor.class, "RF");
    LB = hardwareMap.get(DcMotor.class, "LB");

    // Reverse one of the drive motors.
    // You will have to determine which motor to reverse for your robot.
    // In this example, the right motor was reversed so that positive
    // applied power makes it move the robot in the forward direction.
    LF.setDirection(DcMotorSimple.Direction.REVERSE);
    serv0.setDirection(DcMotorSimple.Direction.REVERSE);
    // Put run blocks here.
    waitForStart();
    if (opModeIsActive()) {
      while (opModeIsActive()) {
        serv0.setPower(2*gamepad1.left_trigger-1);
        serv1.setPower(2*gamepad1.right_trigger-1);
        // Put loop blocks here.
        // The Y axis of a joystick ranges from -1 in its topmost position
        // to +1 in its bottommost position. We negate this value so that
        // the topmost position corresponds to maximum forward power.
        Elevator.setPower(-gamepad1.right_stick_x);
        spare01.setPower(-gamepad1.right_stick_y);
        // The Y axis of a joystick ranges from -1 in its topmost position
        // to +1 in its bottommost position. We negate this value so that
        // the topmost position corresponds to maximum forward power.
       // RB.setPower(-gamepad1.left_stick_x);
       // RF.setPower(-gamepad1.right_stick_x);
        // The Y axis of a joystick ranges from -1 in its topmost position
        // to +1 in its bottommost position. We negate this value so that
        // the topmost position corresponds to maximum forward power.
       // LB.setPower(-gamepad1.left_stick_y);
      //  LF.setPower(-gamepad1.right_stick_y);
        telemetry.addData("LeftX Pow", RB.getPower());
        telemetry.addData("RightXPow", RF.getPower());
        telemetry.addData("LeftY Pow", LB.getPower());
        telemetry.addData("RightY Pow", LF.getPower());
        telemetry.update();
      }
    }
  }
}
