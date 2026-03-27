package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.types.Constants;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

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
        double servoPosition = Range.scale(angle, -Constants.TurretRange / 2, Constants.TurretRange / 2, 0, 1);
        TurretServo.setPosition(servoPosition);
    }

    @Override
    protected double getPosition() {
        return TurretServo.getPosition();
    }
}
