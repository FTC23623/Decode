package org.firstinspires.ftc.teamcode.opMode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.types.VisionMode;

@Autonomous(name = "RedNear2", preselectTeleOp = "TeleRed", group = "Red")
public class AutoNearRed2 extends AutoNear {
    public AutoNearRed2() {
        super(VisionMode.VisionMode_RedGoal, false, 2);
    }
}
