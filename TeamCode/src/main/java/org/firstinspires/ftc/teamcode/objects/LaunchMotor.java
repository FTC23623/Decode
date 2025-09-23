package org.firstinspires.ftc.teamcode.objects;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class LaunchMotor {
    public final String mName;
    private final HydraOpMode mOp;
    private final DcMotorEx motor;
    private final double ticksPerRev;
    private double lastPosition;
    private long lastTime;
    private final double nsToMs = 1.0e-6;
    private final SampleAverage mAvgRpm;

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
    }

    public double GetRPM() {
        // calculate time since the last measurement
        long timeNow = System.nanoTime();
        double timeDifMs = (timeNow - lastTime) * nsToMs;
        // get the current position of the motor and calculate the difference since last time
        double position = Math.abs(motor.getCurrentPosition());
        double dif = position - lastPosition;
        // convert difference in ticks to revolutions
        double rev = dif / ticksPerRev;
        // rpm
        double rpm = 60000 * (rev / timeDifMs);
        // capture present values for the next call
        lastPosition = position;
        lastTime = timeNow;
        mOp.mTelemetry.addData(mName + " ticks", dif);
        mOp.mTelemetry.addData(mName + " time", timeDifMs);
        mOp.mTelemetry.addData(mName + " current", motor.getCurrent(CurrentUnit.MILLIAMPS));
        // pass into the average measurement and return the average if it has been calculated
        mAvgRpm.AddSample(rpm);
        if (mAvgRpm.Ready()) {
            return mAvgRpm.GetAverage();
        }
        return 0;
    }

    public void SetPower(double power) {
        motor.setPower(power);
    }
}
