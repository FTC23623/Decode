package org.firstinspires.ftc.teamcode.objects;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.datalogger.LaunchDatalogger;

public class LaunchMotor {
    public final String mName;
    private final HydraOpMode mOp;
    private final DcMotorEx motor;
    private final double ticksPerRev;
    private double lastPosition;
    private long lastTime;
    private final double nsToMs = 1.0e-6;
    private final SampleAverage mAvgRpm;
    private final LaunchDatalogger mLogger;

    public LaunchMotor(String name, HydraOpMode opMode, DcMotorEx mot, DcMotorSimple.Direction motorDir, double motTicksPerRev, int samplesToAvg){
        mName = name;
        mOp = opMode;
        motor = mot;
        motor.setDirection(motorDir);
        lastPosition = 0;
        lastTime = 0;
        motor.setPower(0);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        ticksPerRev = motTicksPerRev;
        mAvgRpm = new SampleAverage(samplesToAvg);
        mLogger = new LaunchDatalogger(mName + "-launch");
    }

    public double GetRPM() {
        // calculate time since the last measurement
        long timeNow = System.nanoTime();
        double timeDifMs = (timeNow - lastTime) * nsToMs;
        // get the current position of the motor and calculate the difference since last time
        double position = motor.getCurrentPosition();
        double dif = Math.abs(position - lastPosition);
        // convert difference in ticks to revolutions
        double rev = dif / ticksPerRev;
        // rpm
        double rpm = 60000 * (rev / timeDifMs);
        // capture present values for the next call
        lastPosition = position;
        lastTime = timeNow;
        // calculate average of last few rpm values
        mAvgRpm.AddSample(rpm);
        double avgRpm = mAvgRpm.GetAverage();
        // telemetry and datalogging
        double current = motor.getCurrent(CurrentUnit.MILLIAMPS);
        mOp.mTelemetry.addData(mName + " ticks", dif);
        mOp.mTelemetry.addData(mName + " time", timeDifMs);
        mOp.mTelemetry.addData(mName + " current", current);
        mOp.mTelemetry.addData(mName + " RPM", rpm);
        mOp.mTelemetry.addData(mName + " RPM Avg", avgRpm);
        mLogger.rpm.set(rpm);
        mLogger.rpmAvg.set(avgRpm);
        mLogger.current.set(current);
        // return rpm
        return avgRpm;
    }

    public void SetPower(double power) {
        motor.setPower(power);
        mLogger.power.set(power);
        mLogger.writeLine();
    }
}
