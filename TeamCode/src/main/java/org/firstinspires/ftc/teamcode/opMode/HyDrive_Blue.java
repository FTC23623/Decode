package org.firstinspires.ftc.teamcode.opMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.types.DecodeAprilTag;

@TeleOp(name = "HyDrive_Blue")
public class HyDrive_Blue extends HyDrive{
    public HyDrive_Blue() {
        super(DecodeAprilTag.DecodeTag_BlueGoal);
    }
}
