package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.objects.HydraOpMode;
import org.firstinspires.ftc.teamcode.types.VisionMode;

public class Imu_Pinpoint_ReadOnly extends Imu_Base {
    protected HydraOpMode mOpMode;
    protected final PinpointLocalizer mPinpoint;
    private boolean isInit;
    Vector2d targetPose;

    public Imu_Pinpoint_ReadOnly(VisionMode target, PinpointLocalizer pinpoint) {
        super();
        switch (target) {
            case VisionMode_RedGoal:
                targetPose = new Vector2d(-59, 55.189);
                break;
            case VisionMode_BlueGoal:
            default:
                targetPose = new Vector2d(-59, -55.189);
                break;
        }
        mPinpoint = pinpoint;
    }

    @Override
    public boolean Init() {
        if (!isInit) {
            isInit = InitPinpoint(mPinpoint, mOpMode.mTelemetry, true);
        }
        return isInit;
    }

    public static boolean InitPinpoint(PinpointLocalizer pinpoint, Telemetry telemetry, boolean limitTransferSize) {
        return true;
    }

    @Override
    public void ResetYaw() {
    }

    @Override
    public double GetYaw() {
        return 0;
    }

    @Override
    public Pose2d GetPose() {
        return mPinpoint.getPose();
    }

    @Override
    public double DistanceToTarget() {
        Vector2d pos = mPinpoint.getPose().position;
        return Math.sqrt(Math.pow(targetPose.x - pos.x, 2) + Math.pow(targetPose.y - pos.y, 2));
    }
}
