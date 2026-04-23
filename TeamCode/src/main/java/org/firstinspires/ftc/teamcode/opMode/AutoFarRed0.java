package org.firstinspires.ftc.teamcode.opMode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.types.VisionMode;

@Autonomous(name = "RedFar0", preselectTeleOp = "TeleRed", group = "Red")
public class AutoFarRed0 extends AutoFar {
    public AutoFarRed0() {
        super(VisionMode.VisionMode_RedGoal, false, 0);
    }
}
