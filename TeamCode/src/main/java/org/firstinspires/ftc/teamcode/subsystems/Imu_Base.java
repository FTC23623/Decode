package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;

public abstract class Imu_Base implements Imu {
    protected double mOffset;

    public Imu_Base() {
        mOffset = 0;
    }

    @Override
    public abstract boolean Init();

    @Override
    public void Process() {
    }

    @Override
    public void HandleUserInput() {
    }

    @Override
    public void SetYawOffset(double offset) {
        mOffset = offset;
    }

    @Override
    public Pose2d GetPose() {
        return null;
    }
    @Override
    public PoseVelocity2d GetPoseVelocity() {
        return null;
    }

    @Override
    public boolean Calibrating() {
        return false;
    }

    @Override
    public boolean Connected() {
        return true;
    }

    @Override
    public double GetSnapHeading() {
        return 0;
    }

    @Override
    public double DistanceToTarget() {
        return 0;
    }
    @Override
    public void Close() {
    }
}
