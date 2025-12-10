package org.firstinspires.ftc.teamcode.opMode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.types.VisionMode;

@Autonomous(name = "RedLz", preselectTeleOp = "TeleRed", group = "Red")
public class AutoLzRed extends AutoLoadingZone {
    public AutoLzRed() {
        super(VisionMode.VisionMode_RedGoal, false, 4);
    }
}
