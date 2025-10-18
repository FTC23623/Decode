package org.firstinspires.ftc.teamcode.opMode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.types.VisionMode;

@Autonomous(name = "AutoNearRed", preselectTeleOp = "TeleRed")
public class AutoNearRed extends AutoNear {
    public AutoNearRed() {
        super(VisionMode.VisionMode_RedGoal, false);
    }
}
