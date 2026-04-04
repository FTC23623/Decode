package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.objects.HydraPIDFController;
import org.firstinspires.ftc.teamcode.types.Constants;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.hardware.motors.CRServoEx;
import com.seattlesolvers.solverslib.util.MathUtils;
import org.firstinspires.ftc.teamcode.objects.HydraOpMode;
import org.firstinspires.ftc.teamcode.types.TurretTrackMode;
import org.firstinspires.ftc.teamcode.types.VisionMode;

@Config
public class TurretCR extends Turret_Base {
    private final CRServoEx TurretCRServo;
    public HydraPIDFController TurretController;
    public static boolean TurretSynced = false; // Indicates turret encoder is synced to servo absolute encoder.
    public static PIDFCoefficients TurretPIDFCoefficients = new PIDFCoefficients(0.007, 0.0, 0.0005,0.07); //ToDo Set Turret PIDF Coefficients based on Units and Tuning.
    public static double TurretFF = 0.00; // 0.029 Power Acts as feedforward term when turret PIDF is non zero

    public TurretCR(HydraOpMode opMode, Imu imu, VisionMode target, TurretTrackMode trackingMode) {
        super(opMode, imu, target, trackingMode);
        //Continuous Rotation Dual Servo
        //Setup as Single Servo. The Servo output will be split to two power injector ports to run two servos.
        //Alternative is to setup a ServoExGroup to run both from C-Hub
        TurretCRServo = new CRServoEx(mOp.mHardwareMap,"TurretServo")
                .setCachingTolerance(0.001)
                .setRunMode(CRServoEx.RunMode.RawPower)
        ;
        TurretCRServo.setInverted(true);
        TurretController = new HydraPIDFController(TurretPIDFCoefficients.p, TurretPIDFCoefficients.i, TurretPIDFCoefficients.d, TurretPIDFCoefficients.f);
        TurretController.setTolerance(Constants.TurretDeadbandDegrees);
        TurretController.setIntegrationBounds(-0.2, 0.2);
        //TurretController.setMaxOutput(Constants.TurretMaxPower);
        //TurretController.setMinOutput(Constants.TurretMinPower);
    }

    @Override
    public boolean Init() {
        return true;
    }

    // Synchronize throughbore encoder with Axon Servo feedback
    public void resetTurretEncoder() { //Todo: Consider moving this to Turret_Base so it can be used for Servo based control as well.
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
    protected void SetTurretAngle(double angle) {
        setTurret(angle);
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
        }
    }
}
