package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.types.Constants.linearLaunchMotTicksPerRev;
import static org.firstinspires.ftc.teamcode.types.Constants.motorRpmIntervalMs;
import static org.firstinspires.ftc.teamcode.types.Constants.nsToMs;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.teamcode.objects.HydraOpMode;
import org.firstinspires.ftc.teamcode.objects.HydraSubsystem;
import org.firstinspires.ftc.teamcode.objects.LaunchMotor;

import java.util.ArrayList;

@Config
public class Launcher implements HydraSubsystem {
    private final HydraOpMode mOp;
    private final ArrayList<LaunchMotor> motors;
    public static double pidP = 0.0016;
    public static double pidI = 0.0001;
    public static double pidD = 0.0;
    public static double pidF = 0.00028;
    private final PIDFController pid;
    private long lastTime;
    public static int samplesToAverage = 0;
    public static double targetRPMtune = 0;
    public static boolean noPid = false;
    public static double noPidPwr = 0;

    public Launcher(HydraOpMode Opmode, double targetRPM) {
        mOp = Opmode;
        motors = new ArrayList<>(2);
        motors.add(new LaunchMotor("left", mOp, Opmode.mHardwareMap.get(DcMotorEx.class, "left"), DcMotorSimple.Direction.FORWARD, linearLaunchMotTicksPerRev, samplesToAverage));
        motors.add(new LaunchMotor("right", mOp, Opmode.mHardwareMap.get(DcMotorEx.class, "right"), DcMotorSimple.Direction.REVERSE, linearLaunchMotTicksPerRev, samplesToAverage));
        pid = new PIDFController(pidP, pidI, pidD, pidF);
        pid.setSetPoint(targetRPM);
    }

    @Override
    public void Process() {
        // calculate time since the last measurement
        long timeNow = System.nanoTime();
        double timeDifMs = (timeNow - lastTime) * nsToMs;
        if (timeDifMs < motorRpmIntervalMs) {
            return;
        }
        Tune();
        // get rpm of each motor
        double rpm0 = motors.get(0).GetRPM();
        motors.get(1).GetRPM();
        // PID for each motor. Never reverse the motor
        double power;
        if (noPid) {
            // apply power directly for testing
            power = noPidPwr;

        } else {
            // apply power from PID
            power = Math.max(0, pid.calculate(rpm0));
        }
        // set power to each motor
        for (LaunchMotor motor : motors) {
            motor.SetPower(power);
        }
        lastTime = timeNow;
        mOp.mTelemetry.addData("Pwr0", power);
        mOp.mTelemetry.addData("tgtRPM", targetRPMtune);
    }

    @Override
    public void HandleUserInput() {

    }

    public void Tune() {
        pid.setPIDF(pidP, pidI, pidD, pidF);
        if (pid.getSetPoint() != targetRPMtune) {
            pid.setSetPoint(targetRPMtune);
        }
    }
}
