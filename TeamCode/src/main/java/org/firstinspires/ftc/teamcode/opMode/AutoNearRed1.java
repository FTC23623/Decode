package org.firstinspires.ftc.teamcode.opMode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.types.VisionMode;

@Disabled
@Autonomous(name = "RedNear1", preselectTeleOp = "TeleRed", group = "Red")
public class AutoNearRed1 extends AutoNear {
    public AutoNearRed1() {
        super(VisionMode.VisionMode_RedGoal, false, 1, 0);
    }
}
