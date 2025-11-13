package org.firstinspires.ftc.teamcode.subsystems;
import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.teamcode.objects.Subsystem;

public interface Imu extends Subsystem {
    public boolean Connected();
    public boolean Calibrating();
    public void Close();
    public void ResetYaw();
    public double GetYaw();
    public void SetYawOffset(Pose2d offset);
}
