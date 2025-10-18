package org.firstinspires.ftc.teamcode.opMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.types.VisionMode;

@TeleOp(name = "TeleRed")
public class TeleRed extends HyDrive{
    public TeleRed() {
        super(VisionMode.VisionMode_RedGoal);
    }
}
