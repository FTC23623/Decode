package org.firstinspires.ftc.teamcode.objects;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

public class LaunchMotor {
    private final DcMotorEx motor;
    private final double ticksPerRev;
    private double lastPosition;
    private long lastTime;
    private final double nsToMs = 1.0e-6;

    public LaunchMotor(DcMotorEx mot, DcMotorSimple.Direction motorDir, double motTicksPerRev){
        motor = mot;
        motor.setDirection(motorDir);
        ticksPerRev = motTicksPerRev;
    }

    public double GetRPM() {
        // calculate time since the last measurement
        long timeNow = System.nanoTime();
        double timeDifMs = (timeNow - lastTime) * nsToMs;
        // get the current position of the motor and calculate the difference since last time
        double position = motor.getCurrentPosition();
        double dif = position - lastPosition;
        // convert difference in ticks to revolutions
        double rev = dif / ticksPerRev;
        // rpm
        double rpm = 1000 * (rev / timeDifMs);
        // capture present values for the next call
        lastPosition = position;
        lastTime = timeNow;
        return rpm;
    }

    public void SetPower(double power) {
        motor.setPower(power);
    }
}
