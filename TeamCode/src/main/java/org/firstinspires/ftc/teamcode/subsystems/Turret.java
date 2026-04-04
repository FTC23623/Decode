package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.types.Constants;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;
import com.seattlesolvers.solverslib.util.MathUtils;

import org.firstinspires.ftc.teamcode.objects.HydraOpMode;
import org.firstinspires.ftc.teamcode.types.TurretTrackMode;
import org.firstinspires.ftc.teamcode.types.VisionMode;

@Config
public class Turret extends Turret_Base {
    private final ServoEx TurretServo;
    public static boolean TurretSynced = false; // Indicates turret encoder is synced to servo absolute encoder.
    private final ElapsedTime InitTimer;
    private boolean started = false;

    public Turret(HydraOpMode opMode, Imu imu, VisionMode target, TurretTrackMode trackingMode) {
        super(opMode, imu, target, trackingMode);
        //TurretServo = mOp.mHardwareMap.get(ServoEx.class,"TurretServo");
        TurretServo = new ServoEx(mOp.mHardwareMap,"TurretServo");
    //    TurretServo.setDirection(Servo.Direction.REVERSE);
        InitTimer = new ElapsedTime();
        TurretServo.setCachingTolerance(0.001);
    }

    @Override
    public boolean Init() {
    // Zero Absolute Encoder with Turret is in Home Position ToDo: HyDrive has a call to GoHome before OpMode Start
        if (!started) {
            started = true;
            InitTimer.reset();
            TurretServo.set(0.5); // Send Turret Home
        }
        // Wait a bit for Turret to get home.
        if (InitTimer.milliseconds() > 1000) {
            TurretEncoder.overrideResetPos(0);
            return true;
        } else{
            return false;
        }
    }

    @Override
    protected void SetTurretAngle(double angle) {
        double servoPosition = Range.scale(angle, -Constants.TurretRange / 2, Constants.TurretRange / 2, 0, 1);
        mOp.mTelemetry.addData("TurretServoSetPos", servoPosition);
       TurretServo.set(servoPosition);
    }

    @Override
    protected double getPosition() {
        return TurretEncoder.getPosition() * Constants.TurretDegreesPerTick; //Degrees
        //return MathUtils.normalizeDegrees(TurretEncoder.getPosition() * Constants.TurretDegreesPerTick, false);
    }
}
