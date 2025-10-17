package org.firstinspires.ftc.teamcode.opMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.types.DecodeAprilTag;

@TeleOp(name = "HyDrive_Red")
public class HyDrive_Red extends HyDrive{
    public HyDrive_Red() {
        super(DecodeAprilTag.DecodeTag_RedGoal);
    }
}
