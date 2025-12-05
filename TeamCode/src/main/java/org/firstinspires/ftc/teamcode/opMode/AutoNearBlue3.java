package org.firstinspires.ftc.teamcode.opMode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.types.VisionMode;

@Autonomous(name = "BlueNear3", preselectTeleOp = "TeleBlue", group = "Blue")
public class AutoNearBlue3 extends AutoNear {
    public AutoNearBlue3() {
        super(VisionMode.VisionMode_BlueGoal, true, 3, 0);
    }
}
