package org.firstinspires.ftc.teamcode.opMode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.types.VisionMode;

@TeleOp(name = "TeleBlue")
public class TeleBlue extends HyDrive{
    public TeleBlue() {
        super(VisionMode.VisionMode_BlueGoal);
        mStart = new Pose2d(60, -15, Math.toRadians(0));
    }
}
