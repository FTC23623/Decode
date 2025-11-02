package org.firstinspires.ftc.teamcode.opMode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.types.VisionMode;

@Autonomous(name = "RedFar3", preselectTeleOp = "TeleRed")
public class AutoFarRed3 extends AutoFar {
    public AutoFarRed3() {
        super(VisionMode.VisionMode_RedGoal, false, 3);
    }
}
