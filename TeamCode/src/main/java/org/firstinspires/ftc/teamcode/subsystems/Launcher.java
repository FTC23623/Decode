package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.types.Constants.LaunchServoRun;
import static org.firstinspires.ftc.teamcode.types.Constants.contServoOff;
import static org.firstinspires.ftc.teamcode.types.Constants.linearLaunchMotTicksPerRev;
import static org.firstinspires.ftc.teamcode.types.Constants.motorRpmIntervalMs;
import static org.firstinspires.ftc.teamcode.types.Constants.nsToMs;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.objects.HydraOpMode;
import org.firstinspires.ftc.teamcode.objects.Subsystem;
import org.firstinspires.ftc.teamcode.objects.LaunchMotor;
import org.firstinspires.ftc.teamcode.types.Constants;

import java.util.ArrayList;

@Config
public class Launcher implements Subsystem {
    private final HydraOpMode mOp;
    private final ArrayList<LaunchMotor> motors;
    public static double pidP = 0.0016;
    public static double pidI = 0.0001;
    public static double pidD = 0.0;
    public static double pidF = 0.000242;
    private final PIDFController pid;
    private long lastTime;
    public static int samplesToAverage = 0;
    public static double targetRPMtune = 0;
    public static boolean noPid = false;
    public static double noPidPwr = 0;
    private final Servo LaunchServoWheel;
    private boolean RunLaunchServo = false;

    public Launcher(HydraOpMode Opmode, double targetRPM) {
        mOp = Opmode;
        motors = new ArrayList<>(2);
        motors.add(new LaunchMotor("left", mOp, Opmode.mHardwareMap.get(DcMotorEx.class, "left"), DcMotorSimple.Direction.FORWARD, linearLaunchMotTicksPerRev, samplesToAverage));
        motors.add(new LaunchMotor("right", mOp, Opmode.mHardwareMap.get(DcMotorEx.class, "right"), DcMotorSimple.Direction.REVERSE, linearLaunchMotTicksPerRev, samplesToAverage));
        pid = new PIDFController(pidP, pidI, pidD, pidF);
        pid.setSetPoint(targetRPM);
        LaunchServoWheel = mOp.mHardwareMap.get(Servo.class, "LaunchServoWheel");
    }

    @Override
    public void Init() {

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

       if (RunLaunchServo){
           LaunchServoWheel.setPosition(LaunchServoRun);
       }
       else{
           LaunchServoWheel.setPosition(contServoOff);
       }

        mOp.mTelemetry.addData("Pwr0", power);
        mOp.mTelemetry.addData("tgtRPM", targetRPMtune);
    }

    @Override
    public void HandleUserInput() {
        boolean D_pad_Up = mOp.mOperatorGamepad.dpad_up;
        boolean D_pad_Left = mOp.mOperatorGamepad.dpad_left;
        boolean D_pad_Right = mOp.mOperatorGamepad.dpad_right;
        boolean D_pad_Down = mOp.mOperatorGamepad.dpad_down;
        RunLaunchServo = mOp.mOperatorGamepad.right_bumper;
        double R_stick = mOp.mOperatorGamepad.right_stick_x;
        boolean Circle = mOp.mOperatorGamepad.circle;

        if (D_pad_Up){
            targetRPMtune = Constants.LauncherTopRPM;
        }
        else if (D_pad_Left){
            targetRPMtune = Constants.LauncherMedRPM;
        }
        else if (D_pad_Right){
            targetRPMtune = Constants.LauncherLowRPM;
        }
        else if (D_pad_Down){
            targetRPMtune = Constants.LauncherIdleRPM;
        }
        //if (R_bumper){


    }

    public void Tune() {
        pid.setPIDF(pidP, pidI, pidD, pidF);
        if (pid.getSetPoint() != targetRPMtune) {
            pid.setSetPoint(targetRPMtune);
        }
    }
}
