package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.objects.HydraOpMode;

public class Imu_Hub extends Imu_Base {
    protected IMU imu;
    private boolean isInit;

    public Imu_Hub(HydraOpMode opMode) {
        // Initialization Routines
        // Initialize the IMU with non-default settings. To use this block,
        // plug one of the "new IMU.Parameters" blocks into the parameters socket.
        // Create a Parameters object for use with an IMU in a REV Robotics Control Hub or
        // Expansion Hub, specifying the hub's orientation on the robot via the direction that
        // the REV Robotics logo is facing and the direction that the USB ports are facing.
        super();
        imu = opMode.mHardwareMap.get(IMU.class, "imu");
        isInit = false;
    }

    @Override
    public boolean Init() {
        if (!isInit) {
            isInit = true;
            imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                    RevHubOrientationOnRobot.UsbFacingDirection.RIGHT)));
            imu.resetYaw();
        }
        return true;
    }

    @Override
    public void ResetYaw() {
        imu.resetYaw();
        mOffset = 0;
    }

    @Override
    public double GetYaw() {
        double ret = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        return ret + mOffset;
    }
}
