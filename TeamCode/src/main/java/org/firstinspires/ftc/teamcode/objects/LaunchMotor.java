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
    private final SampleAverage mAvgRpm;
    public final LaunchDatalogger mLogger;

    public LaunchMotor(String name, HydraOpMode opMode, DcMotorEx mot, DcMotorSimple.Direction motorDir, double motTicksPerRev, int samplesToAvg){
        mName = name;
        mOp = opMode;
        motor = mot;
        motor.setDirection(motorDir);
        motor.setPower(0);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        ticksPerRev = motTicksPerRev;
        mAvgRpm = new SampleAverage(samplesToAvg);
        mLogger = new LaunchDatalogger(mName + "-launch");
    }

    public double GetRPM() {
        // get velocity from motor controller
        double ticksPerSecond = motor.getVelocity();
        // convert to rpm
        double ticksPerMin = 60 * ticksPerSecond;
        // rpm
        double rpm = ticksPerMin / ticksPerRev;
        mAvgRpm.AddSample(rpm);
        double avgRpm = mAvgRpm.GetAverage();
        // telemetry and datalogging
        double current = motor.getCurrent(CurrentUnit.MILLIAMPS);
        mLogger.rpm.set(rpm);
        mLogger.current.set(current);
        // return rpm
        return avgRpm;
    }

    public void SetPower(double power) {
        motor.setPower(power);
        mLogger.power.set(power);
        mLogger.writeLine();
    }

    public double GetCurrent() {
        return motor.getCurrent(CurrentUnit.MILLIAMPS);
    }
}
