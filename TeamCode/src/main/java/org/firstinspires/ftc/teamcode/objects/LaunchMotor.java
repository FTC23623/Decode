package org.firstinspires.ftc.teamcode.objects;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

public class LaunchMotor {
    private final DcMotorEx motor;
    private final PIDFController pid;
    private final double ticksPerRev;
    private double lastPosition;
    private double lastTime;
    private final double RpmCalcIntervalMs = 10;
    private final double nsToMs = 1.0e-6;

    public LaunchMotor(DcMotorEx mot, DcMotorSimple.Direction motDir, double motTicksPerRev, PIDFCoefficients coeff){
        motor = mot;
        motor.setDirection(motDir);
        ticksPerRev = motTicksPerRev;
        pid = new PIDFController(coeff.p, coeff.i, coeff.d, 0);
    }

    void Process() {
        // calculate time since the last measurement
        double timeNow = System.nanoTime();
        double timeDifMs = (timeNow - lastTime) * nsToMs;
        if (timeDifMs < RpmCalcIntervalMs) {
            // just in case
            return;
        }
        // get the current position of the motor and calculate the difference since last time
        double position = motor.getCurrentPosition();
        double dif = position - lastPosition;
        // convert difference in ticks to revolutions
        double rev = dif / ticksPerRev;
        // rpm
        double rpm = 1000 * (rev / timeDifMs);
        double power = pid.calculate(rpm);
        motor.setPower(power);
        // capture present values for the next call
        lastPosition = position;
        lastTime = timeNow;
    }
}
