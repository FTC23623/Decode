package org.firstinspires.ftc.teamcode.opMode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.types.VisionMode;

@Autonomous(name = "RedLz3", preselectTeleOp = "TeleRed", group = "Red")
public class AutoLzRed3 extends AutoLoadingZone {
    public AutoLzRed3() {
        super(VisionMode.VisionMode_RedGoal, false, 3);
    }
}
