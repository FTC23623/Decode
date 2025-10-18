package org.firstinspires.ftc.teamcode.opMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.types.VisionMode;

@TeleOp(name = "TeleBlue")
public class TeleBlue extends HyDrive{
    public TeleBlue() {
        super(VisionMode.VisionMode_BlueGoal);
    }
}
