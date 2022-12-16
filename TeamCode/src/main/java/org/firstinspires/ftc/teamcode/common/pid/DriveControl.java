package org.firstinspires.ftc.teamcode.common.pid;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Util;
import org.firstinspires.ftc.teamcode.common.Utility;

@Utility.Encapsulate()
public class DriveControl {

    // initialization vars
    private ElapsedTime elapsedTime = new ElapsedTime();
    private double target, proportional, integral, derivative;
    private double error, totalError, rate = 0;
    private double time = -1;

    // constructor
    public DriveControl(double t, double p, double i, double d) {
        target = t;
        proportional = p;
        integral = i;
        derivative = d;
    }

    // pid update loop ig? idk

    /**
     *
     * @param actual
     * @return predictedDistance
     * @function pid update loop
     */
    public double PID(double actual) {
        // take the margin of error between the target distance and where the robot actually is
        double actualError = target - actual;

        // correct the margin of error by calculating the closest value that will control the error margin
        totalError *= Math.signum(actualError);
        totalError += actualError;
        if (2 > Math.abs(actualError)) {
            totalError = 0;
        }

        // Predict how to best control the error margin
        double actualRate = 0;
        if (time > 0) {
            actualRate = (error - error) / (elapsedTime.milliseconds() - time);
        }

        rate = actualRate;
        error = actualError;
        time = elapsedTime.milliseconds();

        // return the computation
        return 0.9 * Math.tanh(proportional * actualError + integral * totalError + derivative * actualRate);
    }
}
