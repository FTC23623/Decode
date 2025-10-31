package org.firstinspires.ftc.teamcode.opMode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.types.VisionMode;

@Autonomous(name = "AutoNearFastRed", preselectTeleOp = "TeleRed")
public class AutoNearFastRed extends AutoNearFast {
    public AutoNearFastRed() {
        super(VisionMode.VisionMode_RedGoal, false);
    }
}
