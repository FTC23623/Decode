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
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import java.util.ArrayList;

@Config
public class LinearLaunchSystem implements HydraSubsystem {
    private final HydraOpMode mOp;
    private final ArrayList<LaunchMotor> motors;
    public static double mainPid_p = 0.0;
    public static double mainPid_i = 0.0;
    public static double mainPid_d = 0.0;
    public static double syncPid_p = 0.0;
    public static double syncPid_i = 0.0;
    public static double syncPid_d = 0.0;
    public static double targetRPMtune = 0;
    private final PIDFController mainPid;
    private final PIDFController syncPid;
    private long lastTime;

    public LinearLaunchSystem(HydraOpMode Opmode, double targetRPM) {
        mOp = Opmode;
        motors = new ArrayList<>(2);
        motors.add(new LaunchMotor("left", mOp, Opmode.mHardwareMap.get(DcMotorEx.class, "left"), DcMotorSimple.Direction.FORWARD, linearLaunchMotTicksPerRev));
        motors.add(new LaunchMotor("right", mOp, Opmode.mHardwareMap.get(DcMotorEx.class, "right"), DcMotorSimple.Direction.REVERSE, linearLaunchMotTicksPerRev));
        mainPid = new PIDFController(mainPid_p, mainPid_i, mainPid_d, 0);
        syncPid = new PIDFController(syncPid_p, syncPid_i, syncPid_d, 0);
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
        // average the two motor RPMs
        double rpm0 = motors.get(0).GetRPM();
        double rpm1 = motors.get(1).GetRPM();
        //double rpm = (rpm0 + rpm1) / 2;
        // main PID calculated on the average RPM of the system
        double power0 = mainPid.calculate(rpm0);
        // sync PID calculated on the error
        double power1 = syncPid.calculate(rpm1);
        // set the main PID output to the first motor
        motors.get(0).SetPower(power0);
        // apply the error to the second motor to sync the system
        motors.get(1).SetPower(power1);
        lastTime = timeNow;
        mOp.mTelemetry.addData(motors.get(0).mName + " RPM", rpm0);
        mOp.mTelemetry.addData(motors.get(1).mName + " RPM", rpm1);
        //mOp.mTelemetry.addData("RPM", rpm);
        mOp.mTelemetry.addData("Pwr0", power0);
        mOp.mTelemetry.addData("Pwr1", power1);
        mOp.mTelemetry.addData("tgtRPM", targetRPMtune);
    }

    public void Tune() {
        mainPid.setPIDF(mainPid_p, mainPid_i, mainPid_d, 0);
        syncPid.setPIDF(syncPid_p, syncPid_i, syncPid_d, 0);
        if (mainPid.getSetPoint() != targetRPMtune) {
            mainPid.setSetPoint(targetRPMtune);
            syncPid.setSetPoint(targetRPMtune);
        }
    }
}
