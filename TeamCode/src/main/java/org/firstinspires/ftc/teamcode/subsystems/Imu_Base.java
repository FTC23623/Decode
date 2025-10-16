package org.firstinspires.ftc.teamcode.subsystems;

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
    public boolean Calibrating() {
        return false;
    }

    @Override
    public boolean Connected() {
        return true;
    }

    @Override
    public void Close() {
    }
}
