package org.firstinspires.ftc.teamcode.opMode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.types.VisionMode;

@Autonomous(name = "AutoFarBlue", preselectTeleOp = "TeleBlue")
public class AutoFarBlue extends AutoFar {
    public AutoFarBlue() {
        super(VisionMode.VisionMode_BlueGoal, true);
    }
}
