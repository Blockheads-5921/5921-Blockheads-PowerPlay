package org.firstinspires.ftc.teamcode.driveModes;

import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

//import org.firstinspires.ftc.teamcode.common.Button;
//import org.firstinspires.ftc.teamcode.common.Constants;

import org.firstinspires.ftc.teamcode.common.Button;
import org.firstinspires.ftc.teamcode.common.HardwareDrive;
//import org.firstinspires.ftc.teamcode.common.pid.CarouselPIDController;

@TeleOp(name="Base Drive", group="Drive")
@Disabled
public class BaseDrive extends OpMode{
    /* Declare OpMode members. */
    HardwareDrive robot = new HardwareDrive();
    //Constants constants = new Constants();
    private ElapsedTime runtime = new ElapsedTime();




    Button capUpButton = new Button();
    Button capDropButton = new Button();
    Button capIntakeButton = new Button();
    Button setCapMode = new Button();
    Button carouselButton = new Button();
    Button carouselButtonInverted = new Button();
    Button lifterButton = new Button();
    Button lifterBottomButton = new Button();
    Button spinInFullButton = new Button();
    Button spinOutFullButton = new Button();

    private boolean toggleButton = true;

    /** The relativeLayout field is used to aid in providing interesting visual feedback
     * in this sample application; you probably *don't* need this when you use a color sensor on your
     * robot. Note that you won't see anything change on the Driver Station, only on the Robot Controller. */
    View relativeLayout;

    @Override
    public void init() {
        robot.init(hardwareMap);

        telemetry.addData("Say", "Hello Driver");
        runtime.reset();

    }

    @Override
    public void init_loop() {
//        robot.lifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.lifter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void start() {
    }

    @Override
    public void loop() {
        UpdatePlayer1();
        //UpdatePlayer2();
        UpdateButton();
        UpdateTelemetry();
    }
    void UpdatePlayer1(){
        double drivePower = DriveTrainSpeed();

        DriveTrainBase(drivePower);
        DriveTrainSpeed();
        //Capping();
        DriveMicroAdjust(0.4);
        //OscillateServo();
    }

    /*void UpdatePlayer2(){
        Carousel();
        Lifter();
        SpinIntake();
    }*/

    void UpdateTelemetry(){

        telemetry.addData("X", gamepad1.left_stick_x);
        telemetry.addData("Y", -gamepad1.left_stick_y);
        telemetry.addData("R", gamepad1.right_stick_x);
//        telemetry.addData("Arm Position", robot.lifter.getCurrentPosition());

//        telemetry.addData("Touch Sensor", robot.digitalTouch.getState());
        telemetry.update();
    }

    void UpdateButton(){
        capDropButton.update(gamepad1.b);
        capIntakeButton.update(gamepad1.a);
        capUpButton.update(gamepad1.y);
        setCapMode.update(gamepad1.x);


        carouselButton.update(gamepad2.a);
        carouselButtonInverted.update(gamepad2.b);
        lifterButton.update(gamepad2.y);
        lifterBottomButton.update(gamepad2.x);
        spinInFullButton.update(gamepad2.dpad_down);
        spinOutFullButton.update(gamepad2.dpad_up);
    }

    /*

        void OscillateServo(){
        if (runtime.seconds() > 1){
            if (countSmile % 2 == 0)
                robot.cap.setPosition(constants.capPickUp);
            else
                robot.cap.setPosition(constants.capStart);
            runtime.reset();
            countSmile += 1;
        }
    }

     */

    void DriveTrainBase(double drivePower){
        double directionX = Math.pow(gamepad1.left_stick_x, 1); //Strafe
        double directionY = -Math.pow(gamepad1.left_stick_y, 1); //Forward
        double directionR = Math.pow(gamepad1.right_stick_x, 1); //Turn


        robot.lf.setPower((directionY + directionR + directionX) * drivePower);
        robot.rf.setPower((directionY - directionR - directionX) * drivePower);
        robot.lb.setPower((directionY + directionR - directionX) * drivePower);
        robot.rb.setPower((directionY - directionR + directionX) * drivePower);


    }

    void DriveMicroAdjust(double power){
        if (gamepad1.dpad_up){
            robot.lf.setPower(power);
            robot.rf.setPower(power);
            robot.lb.setPower(power);
            robot.rb.setPower(power);
        }
        else if (gamepad1.dpad_down){
            robot.lf.setPower(-power);
            robot.rf.setPower(-power);
            robot.lb.setPower(-power);
            robot.rb.setPower(-power);
        }
        else if (gamepad1.dpad_right){
            robot.lf.setPower(power);
            robot.rf.setPower(-power);
            robot.lb.setPower(-power);
            robot.rb.setPower(power);
        }
        else if (gamepad1.dpad_left){
            robot.lf.setPower(-power);
            robot.rf.setPower(power);
            robot.lb.setPower(power);
            robot.rb.setPower(-power);
        }

        if (gamepad1.left_trigger == 1){
            robot.lf.setPower(-power);
            robot.rf.setPower(power);
            robot.lb.setPower(-power);
            robot.rb.setPower(power);
        }
        else if (gamepad1.right_trigger == 1){
            robot.lf.setPower(power);
            robot.rf.setPower(-power);
            robot.lb.setPower(power);
            robot.rb.setPower(-power);
        }
    }

    double DriveTrainSpeed(){
        double drivePower = 0.75;



        if (gamepad1.right_bumper)
            drivePower = 1;
        else if (gamepad1.left_bumper)
            drivePower = 0.25;


        return drivePower;
    }

    /*void Lifter() {
        int position = robot.lifter.getCurrentPosition();
        if (lifterButton.is(Button.State.TAP)) {
            if (position >= (constants.elevatorPositionTop - 10)) {
                robot.lifter.setTargetPosition(constants.elevatorPositionDown + 10);
                robot.lifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.lifter.setPower(1);

            } else {
                robot.lifter.setTargetPosition(constants.elevatorPositionTop);
                robot.lifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.lifter.setPower(1);
            }
        }


        if (!robot.digitalTouch.getState() && toggleButton) {
            //Stop
            robot.lifter.setPower(0);

            //Reset
            robot.lifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.lifter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            robot.lifter.setTargetPosition(20);
            robot.lifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.lifter.setPower(0.1);
            runtime.reset();
        }

        toggleButton = !(runtime.seconds() < 1);


        if (lifterBottomButton.is(Button.State.TAP)){
                if (position >= (constants.elevatorPositionBottom - 400)) {
                    robot.lifter.setTargetPosition(constants.elevatorPositionDown - 390);
                    robot.lifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.lifter.setPower(1);
                }
                else {
                    robot.lifter.setTargetPosition(constants.elevatorPositionBottom);
                    robot.lifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.lifter.setPower(1);
                }
        }

        if (gamepad2.left_bumper) {
            robot.lifter.setTargetPosition(position - 50);
            robot.lifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.lifter.setPower(0.25);
        }
        if (gamepad2.right_bumper) {
            robot.lifter.setTargetPosition(position + 50);
            robot.lifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.lifter.setPower(-0.25);
        }
    }

    void Carousel(){
        if (carouselButton.is(Button.State.HELD)) {
            robot.duckWheel.setVelocity(1600);
        }
        else if (carouselButtonInverted.is(Button.State.HELD)) {
            robot.duckWheel.setVelocity(-1600);
        }
        robot.duckWheel.setPower(0);
    }

    void Capping(){
        if (capIntakeButton.is(Button.State.TAP))
            robot.cap.setPosition(constants.capPickUp);
        if (capUpButton.is(Button.State.TAP))
            robot.cap.setPosition(constants.capStart);
        if (capDropButton.is(Button.State.TAP))
            robot.cap.setPosition(constants.capAlmostDrop);
        if (capDropButton.is(Button.State.DOUBLE_TAP))
            robot.cap.setPosition(constants.capDrop);

    }

    void SpinIntake(){

        if (spinInFullButton.is(Button.State.HELD)) // Spin In
            robot.spin.setPower(1);
        else if (spinOutFullButton.is(Button.State.HELD)) // Spin Out Med
            robot.spin.setPower(-1);
        else if (gamepad2.right_trigger == 1) // Spin In Slow
            robot.spin.setPower(0.3);
        else if (gamepad2.left_trigger == 1) // Spin Out Slow
            robot.spin.setPower(-0.2);
        else
            robot.spin.setPower(0);

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}
