package org.firstinspires.ftc.teamcode.common;

public class Constants {

    //Elevator Motor Positions
    public static int elevatorPositionDown = 0; //Rest
    public static int elevatorPositionTop = -2900; //Extended
    public int elevatorPositionMid = -1200;
    public static int elevatorPositionBottom = -10;
    public int elevatorAcrossDrop = -2000;

    //Capping Servo Positions
    public double capStart = 0.6; //Straight Up Vertical
    public double capPickUp = 0.12; //Touching Floor / Pick Up Capstone
    public double capDrop = 0.38;
    public double capAlmostDrop = 0.44;

    //Drive Train Constants
    public double maxVelocityDT = 2700;
    public double clicksPerInch = 45.285;

    //Degrees Per Inch Auto
    public double degree = 23.47/90;

    //Distance Between Odo and Center
    public double horizontalDistanceOdo = 6.25;
    public double midDistanceOdo = 3.0;

    //Velocity of Spin
    public double spinClockwise = 1600;
    public double spinCounterClockwise = - 1600;

    public enum ELEVATOR_STATES {
        TOP,
        MIDDLE,
        BOTTOM
    }
}
