package org.firstinspires.ftc.teamcode.opMode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.types.VisionMode;

@Autonomous(name = "RedNear3", preselectTeleOp = "TeleRed", group = "Red")
public class AutoNearRed3 extends AutoNear {
    public AutoNearRed3() {
        super(VisionMode.VisionMode_RedGoal, false, 3);
    }
}
