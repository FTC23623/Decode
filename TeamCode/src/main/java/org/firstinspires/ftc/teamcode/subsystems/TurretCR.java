package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.objects.Debouncer;
import org.firstinspires.ftc.teamcode.objects.HydraPIDFController;
import org.firstinspires.ftc.teamcode.types.Constants;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
//import com.seattlesolvers.solverslib.controller.PIDFController;
//import com.arcrobotics.ftclib.controller.PIDFController;
import com.seattlesolvers.solverslib.hardware.AbsoluteAnalogEncoder;
import com.seattlesolvers.solverslib.hardware.motors.CRServoEx;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.util.MathUtils;

import org.firstinspires.ftc.teamcode.objects.HydraOpMode;
import org.firstinspires.ftc.teamcode.objects.Subsystem;
import org.firstinspires.ftc.teamcode.objects.VisionResult;
@Config
public class TurretCR implements Subsystem {
    private final HydraOpMode mOp;
    private final double mPosChangeRate = 0.2;
    private double UserInput = 0;
    private final CRServoEx TurretCRServo;
    //private final AnalogInput TurretServoFb;
    public AbsoluteAnalogEncoder AnalogTurretEncoder;
    public Motor.Encoder TurretEncoder;
    public static double TurretSyncOffset = 0.0;
    public HydraPIDFController TurretController;
    public static boolean TurretSynced = false; // Indicates turret encoder is synced to servo absolute encoder.
    private long lastVisionTimestamp;
    private boolean autoSetAction;
    private double autoSetPos;
    private boolean visionLocked;
    private boolean disableAutoTrack;
    public static int VisionRefreshTimeMs = 10;
    private final Debouncer circleDebounce;
    private final Debouncer triangleDebounce;
    //private final Debouncer squareDebounce;
    public final VoltageSensor voltageSensor;
    public static double TuningTarget = 0;
//    public static double ExternalP = 0;

    public static PIDFCoefficients TurretPIDFCoefficients = new PIDFCoefficients(0.007, 0.0, 0.0005,0.07); //ToDo Set Turret PIDF Coefficients based on Units and Tuning.
    public static double TurretFF = 0.00; // 0.029 Power Acts as feedforward term when turret PIDF is non zero

    public TurretCR(HydraOpMode opMode) {
        mOp = opMode;

        voltageSensor = mOp.mHardwareMap.get(VoltageSensor.class, "Control Hub");

        //Continuous Rotation Dual Servo
        //Setup as Single Servo. The Servo output will be split to two power injector ports to run two servos.
        //Alternative is to setup a ServoExGroup to run both from C-Hub
        TurretCRServo = new CRServoEx(mOp.mHardwareMap,"TurretServo")
                .setCachingTolerance(0.001)
                .setRunMode(CRServoEx.RunMode.RawPower)
        ;
        TurretCRServo.setInverted(true);
        AnalogTurretEncoder = new AbsoluteAnalogEncoder(mOp.mHardwareMap,"TurretServoFb",Constants.TurretServoAnalogRangeVolts, AngleUnit.DEGREES)
                .zero(Constants.TurretEncoderOffset)
                .setReversed(true) // ToDo: Set based on observation.
        ;
        TurretEncoder = new Motor(mOp.mHardwareMap, "leftBack").encoder //ToDo: set name based on port used for encoder
                .setDirection(Motor.Direction.FORWARD) // ToDo: Set based on Encoder orientation and Positive rotation convention
                .overrideResetPos((int) TurretSyncOffset)
        ;
        TurretController = new HydraPIDFController(TurretPIDFCoefficients.p, TurretPIDFCoefficients.i, TurretPIDFCoefficients.d, TurretPIDFCoefficients.f);
        TurretController.setTolerance(Constants.TurretDeadbandDegrees);
        TurretController.setIntegrationBounds(-0.2, 0.2);
        //TurretController.setMaxOutput(Constants.TurretMaxPower);
        //TurretController.setMinOutput(Constants.TurretMinPower);

        lastVisionTimestamp = 0;
        autoSetAction = false;
        autoSetPos = 0;
        visionLocked = false;
        disableAutoTrack = false;
        circleDebounce = new Debouncer(Constants.debounceLong);
        triangleDebounce = new Debouncer(1);
        //squareDebounce = new Debouncer(2);
    }

    @Override
    public boolean Init() {
        return true;
    }

    // Synchronize throughbore encoder with Axon Servo feedback
    public void resetTurretEncoder() {
        if (!TurretSynced) {
            if (AnalogTurretEncoder.getVoltage() > 0.001) {
                TurretEncoder.overrideResetPos(0);
                TurretSyncOffset = TurretEncoder.getPosition() - (MathUtils.normalizeDegrees(AnalogTurretEncoder.getCurrentPosition(), false) * Constants.TurretDegreesPerTick); //ToDo: The Math should have a gear ratio from Servo to encoder in it.
                TurretEncoder.overrideResetPos((int) TurretSyncOffset);
                TurretSynced = true;
            }
        }
    }

    public void setTurret(double setPoint) {
        TurretController.setPIDF(TurretPIDFCoefficients.p, TurretPIDFCoefficients.i, TurretPIDFCoefficients.d, TurretPIDFCoefficients.f);
        TurretController.setSetPoint(Range.clip(setPoint,Constants.TurretMinAngle, Constants.TurretMaxAngle));
    }

    public double getTarget() {
        return TurretController.getSetPoint();
    }

    public double getPosition() {
        if (!TurretSynced) {
            resetTurretEncoder();
        }
        return MathUtils.normalizeDegrees(TurretEncoder.getPosition() * Constants.TurretDegreesPerTick, false);
    }

    @Override
    public void HandleUserInput() {
        //UserInput = mOp.mOperatorGamepad.right_stick_x;
        //UserInput = Range.scale(UserInput,0,1,Constants.TurretMinAngle, Constants.TurretMaxAngle); // Scale stick to Min Max Turret Angle
        circleDebounce.In(mOp.mOperatorGamepad.circle);
        if (circleDebounce.Out()) {
            circleDebounce.Used();
            disableAutoTrack = !disableAutoTrack;
            mOp.mOperatorGamepad.rumbleBlips(1);
        }
        triangleDebounce.In(mOp.mOperatorGamepad.triangle);
        if (triangleDebounce.Out()) {
            triangleDebounce.Used();
            GoHome();
        }
        //squareDebounce.In(mOp.mDriverGamepad.square);
        //if (squareDebounce.Out()) {
        //    GoHome();
        //}
    }

    @Override
    public void Process() {
        VisionResult vision = null;
        if (mOp.mVision != null) {
            vision = mOp.mVision.GetResult();
        }
        if (autoSetAction) {
            TurretController.setSetPoint(autoSetPos);
        }
        else if (vision != null && !disableAutoTrack) {
            mOp.mTelemetry.addData("AprilTag", vision.GetTagClass());
            //mOp.mTelemetry.addData("AprilTag", vision.GetXOffset());
            //mOp.mTelemetry.addData("AprilTag", vision.GetYOffset());
            TurretKinematics.CalcDistanceToTag(vision);
            if (vision.GetTimestamp() > lastVisionTimestamp + VisionRefreshTimeMs) {
                //mOp.mTelemetry.addData("timestamp", vision.GetTimestamp());
                double rotate = -vision.GetXOffset();
                //mOp.mTelemetry.addData("rotate", rotate);
                lastVisionTimestamp = vision.GetTimestamp();
                if (Math.abs(rotate) > 1) {
                    double Target = getPosition() + rotate; // calculate absolute angle target
                    setTurret(Target);
                    //mOp.mTelemetry.addData("Turret Pos V", Target);
                } else {
                    visionLocked = true;
                }
            }
        } else {
            // scale user input with a constant rate
            double position_change = UserInput * mPosChangeRate;
            if (position_change != 0) {
                // get the last set position and calculate the new position
                double Target = getPosition() + position_change;
                setTurret(Target);
                //mOp.mTelemetry.addData("Turret Pos U", NewPos);
            }
            //setTurret(TuningTarget);
            mOp.mTelemetry.addData("Turret tuning target", TuningTarget);
        }
        double power;
        double voltage = voltageSensor.getVoltage();
        power = TurretController.calculate(getPosition()); // PIDF positional control output
        power += TurretFF * (Constants.DefaultVoltage /voltage) * Math.signum(power); // kstatic feedforward output scaled relative to voltage
        mOp.mTelemetry.addData("Turret servo power", power);
        boolean hardStop = (getPosition() >= Constants.TurretMaxAngle || getPosition() <= Constants.TurretMinAngle) && (Math.signum(power) == Math.signum(getPosition()));
        mOp.mTelemetry.addData("Turret hard stop", hardStop);
        if (hardStop) {
            // don't push the turret even further in that direction if it is already past the hardware limits
            TurretCRServo.set(0);
        } else {
            TurretCRServo.set(power);
            //TurretCRServo.set((TuningTarget - getPosition()) * ExternalP);
        }

        if (disableAutoTrack || System.currentTimeMillis() - lastVisionTimestamp > Constants.TurretVisionLockTimeoutMs) {
            visionLocked = false;
        }
        //mOp.mTelemetry.addData("AutoPos", autoSetPos);
        //mOp.mTelemetry.addData("AutoAction", autoSetAction);
        mOp.mTelemetry.addData("VisionLocked", visionLocked);
        mOp.mTelemetry.addData("AutoTrack", !disableAutoTrack);
        mOp.mTelemetry.addData("TurretAnalogPos", MathUtils.normalizeDegrees(AnalogTurretEncoder.getCurrentPosition(), false)/Constants.TurretGearRatioTurretToServo);
        mOp.mTelemetry.addData("TurretAnalogEncVolt",AnalogTurretEncoder.getVoltage());
        mOp.mTelemetry.addData("TurretEncPos", getPosition());
        mOp.mTelemetry.addData("TurretEncTicks", TurretEncoder.getPosition());
    }

    public boolean Locked() {
        return visionLocked;
    }

    public void ForceUnlock(){
        visionLocked = false;
        // reset the timestamp so we can use the very next update from the camera
        lastVisionTimestamp -= (VisionRefreshTimeMs - 1);
    }

    public void GoHome() {
        autoSetAction = true;
        autoSetPos = 0; // Degrees
        Process();
        autoSetAction = false;
    }

    /*
     * ROAD RUNNER API
     */
    /**
     * Get a new action object for Road Runner to run
     * @param action: the action to run in this instance
     * @return the action object for RR to use
     */
    public Action GetLockAction() {
        return new TurretCR.RunLockAction();
    }

    public Action GetSetAction(double position) {
        return new TurretCR.RunSetAction(position);
    }

    public Action GetDisableAction(boolean disable) {
        return new TurretCR.RunDisableAction(disable);
    }
    /**
     * Runs the supplied action until completion
     */
    public class RunLockAction implements Action {
        // run has been called once
        protected boolean started = false;
        protected ElapsedTime timer;
        public RunLockAction() {
            timer = new ElapsedTime();
        }

        /**
         * Runs the desired action until completion
         * @param packet: ??
         * @return true while the action is running
         */
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!started) {
                timer.reset();
                started = true;
                ForceUnlock();
            }
            return !visionLocked && timer.milliseconds() < Constants.TurretVisionLockTimeoutMs;
        }
    }

    public class RunSetAction implements Action {
        private boolean started = false;
        private final double angleToSet;
        public RunSetAction(double angleToSet) {
            this.angleToSet = angleToSet;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!started) {
                started = true;
                autoSetAction = true;
                //autoSetPos = CalcAbsolutePositionFromAngle(angleToSet);
                autoSetPos = angleToSet; //Degrees.
            }
            Process();
            autoSetAction = false;
            /*if (visionLocked) {
                autoSetAction = false;
                return false;
            } else
            if (Math.abs(GetPositionFromFb() - autoSetPos) < CalcPositionOffsetAngle(Constants.TurretDeadbandDegrees)) {
                autoSetAction = false;
                return false;
            }*/
            return false;
        }
    }

    public class RunDisableAction implements Action {
        boolean disable;

        public RunDisableAction(boolean disable) {
            this.disable = disable;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            disableAutoTrack = disable;
            return false;
        }
    }

}
