package org.firstinspires.ftc.teamcode.opMode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.types.VisionMode;

@Autonomous(name = "AutoFarRed", preselectTeleOp = "TeleRed")
public class AutoFarRed extends AutoFar {
    public AutoFarRed() {
        super(VisionMode.VisionMode_RedGoal, false);
    }
}
