package org.firstinspires.ftc.teamcode.opMode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.types.VisionMode;

@Autonomous(name = "RedNearGate1", preselectTeleOp = "TeleRed", group = "Red")
public class AutoNearRedGate1 extends AutoNear {
    public AutoNearRedGate1() {
        super(VisionMode.VisionMode_RedGoal, false, 1, true);
    }
}
