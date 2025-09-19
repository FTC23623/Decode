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
    private final ArrayList<LaunchMotor> motors;
    public static PIDFCoefficients mainPidCoeff;
    public static PIDFCoefficients syncPidCoeff;
    public static double targetRPMtune;
    private final PIDFController mainPid;
    private final PIDFController syncPid;
    private long lastTime;

    public LinearLaunchSystem(HydraOpMode Opmode, double targetRPM) {
        motors = new ArrayList<>(2);
        motors.add(new LaunchMotor(Opmode.mHardwareMap.get(DcMotorEx.class, "left"), DcMotorSimple.Direction.REVERSE, linearLaunchMotTicksPerRev));
        motors.add(new LaunchMotor(Opmode.mHardwareMap.get(DcMotorEx.class, "right"), DcMotorSimple.Direction.FORWARD, linearLaunchMotTicksPerRev));
        mainPidCoeff = new PIDFCoefficients(0, 0, 0, 0);
        mainPid = new PIDFController(mainPidCoeff.p, mainPidCoeff.i, mainPidCoeff.d, mainPidCoeff.f, targetRPM, 0);
        syncPidCoeff = new PIDFCoefficients(0, 0, 0, 0);
        syncPid = new PIDFController(syncPidCoeff.p, syncPidCoeff.i, syncPidCoeff.d, syncPidCoeff.f);
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
        double rpm = (rpm0 + rpm1) / 2;
        // main PID calculated on the average RPM of the system
        double mainPower = mainPid.calculate(rpm);
        // sync PID calculated on the error
        double syncPower = syncPid.calculate(rpm0 - rpm1);
        // set the main PID output to the first motor
        motors.get(0).SetPower(mainPower);
        // apply the error to the second motor to sync the system
        motors.get(1).SetPower(mainPower + syncPower);
        lastTime = timeNow;
    }

    public void Tune() {
        mainPid.setPIDF(mainPidCoeff.p, mainPidCoeff.i, mainPidCoeff.d, mainPidCoeff.f);
        syncPid.setPIDF(syncPidCoeff.p, syncPidCoeff.i, syncPidCoeff.d, syncPidCoeff.f);
        if (mainPid.getSetPoint() != targetRPMtune) {
            mainPid.setSetPoint(targetRPMtune);
        }
    }
}
