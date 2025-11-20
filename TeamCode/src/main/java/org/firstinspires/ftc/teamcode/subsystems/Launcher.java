package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.types.Constants.LaunchServoRun;
import static org.firstinspires.ftc.teamcode.types.Constants.contServoOff;
import static org.firstinspires.ftc.teamcode.types.Constants.linearLaunchMotTicksPerRev;
import static org.firstinspires.ftc.teamcode.types.Constants.motorRpmIntervalMs;
import static org.firstinspires.ftc.teamcode.types.Constants.nsToMs;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.objects.Debouncer;
import org.firstinspires.ftc.teamcode.objects.HydraOpMode;
import org.firstinspires.ftc.teamcode.objects.Subsystem;
import org.firstinspires.ftc.teamcode.objects.LaunchMotor;
import org.firstinspires.ftc.teamcode.types.Constants;
import org.firstinspires.ftc.teamcode.types.LauncherActions;

import java.util.ArrayList;

@Config
public class Launcher implements Subsystem {
    private final HydraOpMode mOp;
    private final ArrayList<LaunchMotor> motors;
    private final ArrayList<Double> lastRpmMeasure;
    private final ArrayList<Double> lastPwrSetting;
    public static double pidP = 0.003;
    private final double pidpFar = 0.003;
    private final double pidpMed = 0.003;
    private final double pidpNear = 0.003;
    public static double pidI = 0.0001;
    private final double pidiFar = 0.0001;
    private final double pidiMed = 0.0001;
    private final double pidiNear = 0.0001;
    public static double pidD = 0.0;
    private final double piddFar = 0;
    private final double piddMed = 0;
    private final double piddNear = 0;
    public static double pidF = 0.000242;
    private final double pidfFar = 0.000215;
    private final double pidfMed = 0.000215;
    private final double pidfNear = 0.000248;
    private final PIDFController pid;
    private long lastTime;
    public static int samplesToAverage = 0;
    public static double targetRPMtune = 0;
    public static boolean noPid = false;
    public static double noPidPwr = 0;
    private final Servo LaunchServoWheel;
    private boolean RunLaunchServo = false;
    private boolean autoLaunchPressed;
    private final Debouncer autoLaunchDebounce;
    private final Turret turret;

    public Launcher(HydraOpMode Opmode, Turret turret, double targetRPM) {
        mOp = Opmode;
        motors = new ArrayList<>(2);
        motors.add(new LaunchMotor("left", mOp, Opmode.mHardwareMap.get(DcMotorEx.class, "left"), DcMotorSimple.Direction.FORWARD, linearLaunchMotTicksPerRev, samplesToAverage));
        //motors.add(new LaunchMotor("right", mOp, Opmode.mHardwareMap.get(DcMotorEx.class, "right"), DcMotorSimple.Direction.REVERSE, linearLaunchMotTicksPerRev, samplesToAverage));
        pid = new PIDFController(pidP, pidI, pidD, pidF);
        pid.setSetPoint(targetRPM);
        targetRPMtune = targetRPM;
        LaunchServoWheel = mOp.mHardwareMap.get(Servo.class, "LaunchServoWheel");
        lastRpmMeasure = new ArrayList<Double>(motors.size());
        lastPwrSetting = new ArrayList<Double>(motors.size());
        autoLaunchPressed = false;
        autoLaunchDebounce = new Debouncer(1);
        this.turret = turret;
        for (int i = 0; i < motors.size(); i++) {
            lastRpmMeasure.add(0.0);
            lastPwrSetting.add(0.0);
        }
    }

    @Override
    public boolean Init() {
        return true;
    }

    @Override
    public void Process() {
        // calculate time since the last measurement
        long timeNow = System.nanoTime();
        double timeDifMs = (timeNow - lastTime) * nsToMs;
        if (timeDifMs >= motorRpmIntervalMs) {
            Tune();
            // get rpm of each motor
            double rpm0 = motors.get(0).GetRPM();
            // motors.get(1).GetRPM();
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
                lastRpmMeasure.set(motors.indexOf(motor), rpm0);
                lastPwrSetting.set(motors.indexOf(motor), power);
            }
            lastTime = timeNow;
        }
        boolean AutoLaunch = false;
        if(turret != null) {
            if (autoLaunchDebounce.Out()) {
                turret.ForceUnlock();
                autoLaunchDebounce.Used();
            }
            AutoLaunch = autoLaunchPressed && turret.Locked();
        }
        if (RunLaunchServo || AutoLaunch){
            LaunchServoWheel.setPosition(LaunchServoRun);
        }
        else{
            LaunchServoWheel.setPosition(contServoOff);
        }

        mOp.mTelemetry.addData("LaunchPwr", lastPwrSetting.get(0));
        mOp.mTelemetry.addData("LaunchRPM", lastRpmMeasure.get(0));
        mOp.mTelemetry.addData("tgtRPM", targetRPMtune);
        mOp.mTelemetry.addData("launchLoad", RunLaunchServo);
        mOp.mTelemetry.addData("launchLoadSet", LaunchServoRun);
    }

    @Override
    public void HandleUserInput() {
        boolean D_pad_Up = mOp.mOperatorGamepad.dpad_up;
        boolean D_pad_Left = mOp.mOperatorGamepad.dpad_left;
        boolean D_pad_Right = mOp.mOperatorGamepad.dpad_right;
        boolean D_pad_Down = mOp.mOperatorGamepad.dpad_down;
        autoLaunchPressed = mOp.mOperatorGamepad.right_bumper;
        autoLaunchDebounce.In(autoLaunchPressed);
        RunLaunchServo = mOp.mOperatorGamepad.square;

        if (D_pad_Up){
            targetRPMtune = Constants.LauncherTopRPMTele;
        }
        else if (D_pad_Left || D_pad_Right) {
            targetRPMtune = Constants.LauncherMedRPMTele;
        }
        else if (D_pad_Down){
            targetRPMtune = Constants.LauncherLowRPMTele;
        }
    }

    public void Tune() {
        if (pid.getSetPoint() != targetRPMtune) {
            if (targetRPMtune < Constants.LauncherLowRPMThreshold) {
                pidF = pidfNear;
                pidI = pidiNear;
                pidP = pidpNear;
                pidD = piddNear;
            } else if (targetRPMtune < Constants.LauncherMedRPMThreshold) {
                pidF = pidfMed;
                pidI = pidiMed;
                pidP = pidpMed;
                pidD = piddMed;
            } else {
                pidF = pidfFar;
                pidI = pidiFar;
                pidP = pidpFar;
                pidD = piddFar;
            }
            pid.setSetPoint(targetRPMtune);
            pid.clearTotalError();
            for (LaunchMotor motor : motors) {
                motor.mLogger.rpmTgt.set(targetRPMtune);
            }
        }
        mOp.mTelemetry.addData("pidF", pidF);
        pid.setPIDF(pidP, pidI, pidD, pidF);
    }

    public boolean AtSpeed() {
        return Math.abs(lastRpmMeasure.get(0) - targetRPMtune) < Constants.LaunchWheelRpmDeadband;
    }

    public void SetSpeed(double speed) {
        targetRPMtune = speed;
    }

    /*
     * ROAD RUNNER API
     */
    /**
     * Get a new action object for Road Runner to run
     * @param action: the action to run in this instance
     * @return the action object for RR to use
     */
    public Action GetAction(LauncherActions action) {
        return new Launcher.RunAction(action);
    }
    /**
     * Runs the supplied action until completion
     */
    public class RunAction implements Action {
        // action this instance will run
        private boolean started = false;
        // run has been called once
        private final LauncherActions mAction;
        private final ElapsedTime launchTimer;


        // construct on the supplied action
        public RunAction(LauncherActions action) {
            mAction = action;
            launchTimer = new ElapsedTime();
        }

        /**
         * Runs the desired action until completion
         * @param packet: ??
         * @return true while the action is running
         */
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!started) {
                started = true;
                launchTimer.reset();
                switch (mAction) {
                    case LauncherRunOff:
                        targetRPMtune = Constants.LauncherIdleRPM;
                        break;
                    case LauncherRunSlow:
                        targetRPMtune = Constants.LauncherLowRPMAuto;
                        break;
                    case LauncherRunMid:
                        targetRPMtune = Constants.LauncherMedRPMAuto;
                        break;
                    case LauncherRunFast:
                        targetRPMtune = Constants.LauncherTopRPMAuto;
                        break;
                    case LauncherLaunch:
                        RunLaunchServo = true;
                        break;
                    default:
                        return false;
                }
            }
            switch (mAction) {
                case LauncherRunOff:
                case LauncherRunSlow:
                case LauncherRunMid:
                case LauncherRunFast:
                    return !AtSpeed() && launchTimer.milliseconds() < Constants.LauncherSpeedChangeWaitTimeMs;
                case LauncherLaunch:
                    boolean launching = launchTimer.milliseconds() < Constants.LauncherAutoLaunchTimeMs;
                    RunLaunchServo = launching;
                    return launching;
                default:
                    return false;
            }
        }
    }
}
