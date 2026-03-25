package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.types.Constants;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.objects.HydraOpMode;
import org.firstinspires.ftc.teamcode.types.VisionMode;

@Config
public class Turret extends Turret_Base {
    private final Servo TurretServo;
    public static boolean TurretSynced = false; // Indicates turret encoder is synced to servo absolute encoder.
    private final ElapsedTime InitTimer;
    private boolean started = false;

    public Turret(HydraOpMode opMode, Imu imu, VisionMode target) {
        super(opMode, imu, target);
        TurretServo = mOp.mHardwareMap.get(Servo.class,"TurretServo");
        InitTimer = new ElapsedTime();
    }

    @Override
    public boolean Init() {
        //TurretEncoder.overrideResetPos(0); // Assumes Turret is at Zero on Init
        if (!started) {
            started = true;
            InitTimer.reset();
            SetTurretAngle(0); // Send Turret Home
        }
        // Wait a bit for Turret to get home.
        if (InitTimer.milliseconds() > 1000) {
            TurretEncoder.overrideResetPos(0); // Assumes Turret is at Zero on Init
            return true;
        } else{
            return false;
        }
    }

    @Override
    protected void SetTurretAngle(double angle) {
        // convert the angle to 0 to max since servo is 0 to 1
        double degrees = angle * Constants.TurretGearRatio + Constants.TurretRange / 2;
        // clamp the value to the range
        degrees = Math.max(0, Math.min(Constants.TurretRange, degrees));
        double position = degrees / Constants.TurretRange;
        TurretServo.setPosition(position);
    }
}
