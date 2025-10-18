package org.firstinspires.ftc.teamcode.opMode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.types.VisionMode;

@Autonomous(name = "AutoNearBlue", preselectTeleOp = "TeleBlue")
public class AutoNearBlue extends AutoNear {
    public AutoNearBlue() {
        super(VisionMode.VisionMode_BlueGoal, true);
    }
}
