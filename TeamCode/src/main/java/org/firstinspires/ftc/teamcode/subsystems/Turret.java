package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.types.Constants;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.objects.HydraOpMode;
import org.firstinspires.ftc.teamcode.types.VisionMode;

@Config
public class Turret extends Turret_Base {
    private final Servo TurretServo;
    public static boolean TurretSynced = false; // Indicates turret encoder is synced to servo absolute encoder.

    public Turret(HydraOpMode opMode, Imu imu, VisionMode target) {
        super(opMode, imu, target);
        TurretServo = mOp.mHardwareMap.get(Servo.class,"TurretServo");
    }

    @Override
    public boolean Init() {  //ToDo: ?????? Does this really get called during Opmode Init? ????????
        //TurretEncoder.overrideResetPos(0); // Assumes Turret is at Zero on Init
        return true;
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
