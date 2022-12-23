package org.firstinspires.ftc.teamcode.common;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.RobotLog;

import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

public class Utility {
    private final HardwareDrive robot;
    private final CRServo servo;

    @Retention(RetentionPolicy.RUNTIME)
    @Target({ElementType.METHOD, ElementType.TYPE})
    public @interface Encapsulate {

    }

    // constructor
    public Utility(HardwareDrive robot, CRServo servoMotor) {
        this.robot = robot;
        this.servo = servoMotor;
    }

    public void resetEncoder() {
        this.robot.lf.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        this.robot.lb.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        this.robot.rf.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        this.robot.rb.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        this.robot.lf.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        this.robot.lb.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        this.robot.rf.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        this.robot.rb.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    /**
     * A function that uses bezier curves to generate a constant sliding motion across the field
     * @param forwardInches The amount of encoder pulses to move forward can be used as reverse if the value is negative (RELATIVE TO FORWARD)
     * @param spinInches The amount of encoder pulses to turn (RELATIVE TO RIGHT)
     * @param strafeInches The amount of encoder to strafe (RELATIVE)
     * @param bezierFactor The calculated rate at which the robot's TURN to DRIVE ratio should decrease according to the bezier
     * @param drivePower The amount of power the motors should be allocated
     */
    public void SimultaneousMovement(int forwardInches, int spinInches, int strafeInches, double bezierFactor, double drivePower) {
        this.resetEncoder();

        if (bezierFactor != (0)) {
            for (double i = bezierFactor; i < forwardInches; i += 0.1) {
                this.robot.lf.setPower(forwardInches);
                this.robot.rf.setPower(forwardInches);
                this.robot.lb.setPower(forwardInches);
                this.robot.rf.setPower(forwardInches);
            }
        }
    }

    private void SetBrakes(boolean brakesOn) {
        if (brakesOn) {
            this.robot.lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            this.robot.rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            this.robot.lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            this.robot.rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } else {
            this.robot.lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            this.robot.rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            this.robot.lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            this.robot.rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
    }
    public void StrafeRight(int straferightEncoderPulses, double drivePower) {
        this.robot.lf.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        this.robot.lb.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        this.robot.rf.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        this.robot.rb.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        this.robot.lf.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        this.robot.lb.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        this.robot.rf.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        this.robot.rb.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        this.robot.lf.setTargetPosition(-straferightEncoderPulses);
        this.robot.rf.setTargetPosition(-straferightEncoderPulses);
        this.robot.lb.setTargetPosition(straferightEncoderPulses);
        this.robot.rb.setTargetPosition(straferightEncoderPulses);

        this.robot.lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.robot.rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.robot.lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.robot.rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.robot.lf.setPower(drivePower);
        this.robot.rf.setPower(drivePower);
        this.robot.lb.setPower(drivePower);
        this.robot.rb.setPower(drivePower);

        // update the telemetry monitor
        while (LinearOpMode.opModeIsActive() && (this.robot.lf.isBusy())) {
            telemetry.addData("Running to", " %7d ", straferightEncoderPulses);
            telemetry.addData("Currently at", " at %7d", this.robot.lf.getCurrentPosition());
            telemetry.update();
            RobotLog.d("StrafeRight: Encoders: %7d,%7d,%7d,%7d", this.robot.lf.getCurrentPosition(),
                    this.robot.rf.getCurrentPosition(), this.robot.lb.getCurrentPosition(), this.robot.rb.getCurrentPosition());
        }
    }

    private void StrafeLeft(int strafeleftEncoderPulses, double drivePower) {
        this.robot.lf.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        this.robot.lb.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        this.robot.rf.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        this.robot.rb.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        this.robot.lf.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        this.robot.lb.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        this.robot.rf.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        this.robot.rb.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        this.robot.lf.setTargetPosition(strafeleftEncoderPulses);
        this.robot.rf.setTargetPosition(strafeleftEncoderPulses);
        this.robot.lb.setTargetPosition(-strafeleftEncoderPulses);
        this.robot.rb.setTargetPosition(-strafeleftEncoderPulses);

        this.robot.lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.robot.rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.robot.lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.robot.rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.robot.lf.setPower(drivePower);
        this.robot.rf.setPower(drivePower);
        this.robot.lb.setPower(drivePower);
        this.robot.rb.setPower(drivePower);

        while (LinearOpMode.opModeIsActive() &&
                // (runtime.seconds() < timeoutS) &&
                (this.robot.lf.isBusy())) {
            telemetry.addData("Running to", " %7d ", strafeleftEncoderPulses);
            telemetry.addData("Currently at", " at %7d", this.robot.lf.getCurrentPosition());
            telemetry.update();
            RobotLog.d("StrafeLeft: Encoders: %7d,%7d,%7d,%7d", this.robot.lf.getCurrentPosition(),
                    this.robot.rf.getCurrentPosition(), this.robot.lb.getCurrentPosition(), this.robot.rb.getCurrentPosition());
        }
    }
    private void SpinLeft(int spinleftEncoderPulses, double drivePower) {

        this.robot.lf.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        this.robot.lb.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        this.robot.rf.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        this.robot.rb.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        this.robot.lf.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        this.robot.lb.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        this.robot.rf.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        this.robot.rb.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        this.robot.lf.setTargetPosition(spinleftEncoderPulses);
        this.robot.rf.setTargetPosition(spinleftEncoderPulses);
        this.robot.lb.setTargetPosition(spinleftEncoderPulses);
        this.robot.rb.setTargetPosition(spinleftEncoderPulses);

        this.robot.lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.robot.rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.robot.lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.robot.rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.robot.lf.setPower(drivePower);
        this.robot.rf.setPower(drivePower);
        this.robot.lb.setPower(drivePower);
        this.robot.rb.setPower(drivePower);

        while (LinearOpMode.opModeIsActive() &&
                // (runtime.seconds() < timeoutS) &&
                (this.robot.lf.isBusy())) {
            telemetry.addData("Running to", " %7d ", spinleftEncoderPulses);
            telemetry.addData("Currently at", " at %7d", this.robot.lf.getCurrentPosition());
            telemetry.update();
            RobotLog.d("SpinLeft: Encoders: %7d,%7d,%7d,%7d", this.robot.lf.getCurrentPosition(),
                    this.robot.rf.getCurrentPosition(), this.robot.lb.getCurrentPosition(), this.robot.rb.getCurrentPosition());
        }
    }
    private void SpinRight(int spinrightEncoderPulses, double drivePower) {
        this.robot.lf.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        this.robot.lb.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        this.robot.rf.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        this.robot.rb.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        this.robot.lf.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        this.robot.lb.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        this.robot.rf.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        this.robot.rb.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        this.robot.lf.setTargetPosition(-spinrightEncoderPulses);
        this.robot.rf.setTargetPosition(-spinrightEncoderPulses);
        this.robot.lb.setTargetPosition(-spinrightEncoderPulses);
        this.robot.rb.setTargetPosition(-spinrightEncoderPulses);

        this.robot.lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.robot.rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.robot.lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.robot.rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.robot.lf.setPower(drivePower);
        this.robot.rf.setPower(drivePower);
        this.robot.lb.setPower(drivePower);
        this.robot.rb.setPower(drivePower);

        while (LinearOpMode.opModeIsActive() &&
                // (runtime.seconds() < timeoutS) &&
                (this.robot.lf.isBusy())) {
            telemetry.addData("Running to", " %7d ", spinrightEncoderPulses);
            telemetry.addData("Currently at", " at %7d", this.robot.lf.getCurrentPosition());
            telemetry.update();
            RobotLog.d("SpinRight: Encoders: %7d,%7d,%7d,%7d", this.robot.lf.getCurrentPosition(),
                    this.robot.rf.getCurrentPosition(), this.robot.lb.getCurrentPosition(), this.robot.rb.getCurrentPosition());
        }
    }
    private void DriveForward(int forwardEncoderPulses, double drivePower) {
        this.robot.lf.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        this.robot.lb.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        this.robot.rf.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        this.robot.rb.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        this.robot.lf.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        this.robot.lb.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        this.robot.rf.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        this.robot.rb.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        this.robot.lf.setTargetPosition(-forwardEncoderPulses);
        this.robot.rf.setTargetPosition(+forwardEncoderPulses);
        this.robot.lb.setTargetPosition(-forwardEncoderPulses);
        this.robot.rb.setTargetPosition(+forwardEncoderPulses);

        this.robot.lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.robot.rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.robot.lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.robot.rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.robot.lf.setPower(drivePower);
        this.robot.rf.setPower(drivePower);
        this.robot.lb.setPower(drivePower);
        this.robot.rb.setPower(drivePower);

        while (LinearOpMode.opModeIsActive() &&
                // (runtime.seconds() < timeoutS) &&
                (this.robot.lf.isBusy())) {
            telemetry.addData("Running to", " %7d ", forwardEncoderPulses);
            telemetry.addData("Currently at", " at %7d", this.robot.lf.getCurrentPosition());
            telemetry.update();
            RobotLog.d("Forward: Encoders: %7d,%7d,%7d,%7d", this.robot.lf.getCurrentPosition(),
                    this.robot.rf.getCurrentPosition(), this.robot.lb.getCurrentPosition(), this.robot.rb.getCurrentPosition());
        }
    }
    private void DriveReverse(int reverseEncoderPulses, double drivePower) {
        this.robot.lf.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        this.robot.lb.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        this.robot.rf.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        this.robot.rb.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        this.robot.lf.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        this.robot.lb.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        this.robot.rf.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        this.robot.rb.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        this.robot.lf.setTargetPosition(reverseEncoderPulses);
        this.robot.rf.setTargetPosition(-reverseEncoderPulses);
        this.robot.lb.setTargetPosition(reverseEncoderPulses);
        this.robot.rb.setTargetPosition(-reverseEncoderPulses);

        this.robot.lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.robot.rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.robot.lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.robot.rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.robot.lf.setPower(drivePower);
        this.robot.rf.setPower(drivePower);
        this.robot.lb.setPower(drivePower);
        this.robot.rb.setPower(drivePower);

        while (LinearOpMode.opModeIsActive() &&
                // (runtime.seconds() < timeoutS) &&
                (this.robot.lf.isBusy())) {
            telemetry.addData("Running to", " %7d ", reverseEncoderPulses);
            telemetry.addData("Currently at", " at %7d", this.robot.lf.getCurrentPosition());
            telemetry.update();
            RobotLog.d("Reverse: Encoders: %7d,%7d,%7d,%7d", this.robot.lf.getCurrentPosition(),
                    this.robot.rf.getCurrentPosition(), this.robot.lb.getCurrentPosition(), this.robot.rb.getCurrentPosition());
        }
    }
    private void DepositCone(int junctionLevel) throws InterruptedException {
        //assumes lift is at bottom
        int targetPos = 0;
        switch (junctionLevel) {
            case 1:
                targetPos = Constants.elevatorPositionLow;
                break;
            case 2:
                targetPos = Constants.elevatorPositionMid;
                break;
            case 3:
                targetPos = Constants.elevatorPositionTop;
                break;
        }
        //raise arm
        this.robot.lift.setTargetPosition(targetPos);
        this.robot.lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.robot.lift.setPower(1.00);
        sleep(2300);
        this.robot.lift.setPower(0); //Brake arm, maybe unnecessary?
        //Drive forward
        SetBrakes(false);
        DriveForward(100,0.15);
        //Release cone
        this.servo.setPower(0.18);
        //Back up
        DriveReverse(75,0.30);
        sleep(250);
        //lower arm
        this.robot.lift.setTargetPosition(Constants.elevatorPositionBottom);
        this.robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.robot.lift.setPower(0.75);
        sleep(750);
        SetBrakes(true);
    }
}
