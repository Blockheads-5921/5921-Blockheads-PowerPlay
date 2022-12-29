package org.firstinspires.ftc.teamcode.common;

public class Constants {

    //Elevator Motor Positions
    public static final int elevatorPositionDown = 0; //Rest
    public static final int elevatorPositionTop = -3060; //High junction
    public static final int elevatorPositionMid = -2050; //Medium junction
    public static final int elevatorPositionLow = -1250; //Low junction
    public static final int elevatorPositionBottom = -20; // changed from -10 to -20
    public static final int elevatorPositionAboveCone = -400;
    public final int elevatorAcrossDrop = -2000;

    // Capping Servo Positions
    public double capStart = 0.6; //Straight Up Vertical
    public double capPickUp = 0.12; //Touching Floor / Pick Up Capstone
    public double capDrop = 0.38;
    public double capAlmostDrop = 0.44;

    // Drive Train Constants
    public double maxVelocityDT = 2700;
    public double clicksPerInch = 45.285;

    // Degrees Per Inch Auto
    public double degree = 23.47/90;

    // Distance Between Odo and Center
    public double horizontalDistanceOdo = 6.25;
    public double midDistanceOdo = 3.0;

    // Velocity of Spin
    public double spinClockwise = 1600;
    public double spinCounterClockwise = - 1600;
}
