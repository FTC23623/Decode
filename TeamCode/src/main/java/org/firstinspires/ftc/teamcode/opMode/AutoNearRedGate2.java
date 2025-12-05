package org.firstinspires.ftc.teamcode.opMode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.types.VisionMode;

@Autonomous(name = "RedNearGate2", preselectTeleOp = "TeleRed", group = "Red")
public class AutoNearRedGate2 extends AutoNear {
    public AutoNearRedGate2() {
        super(VisionMode.VisionMode_RedGoal, false, 2, 1);
    }
}
