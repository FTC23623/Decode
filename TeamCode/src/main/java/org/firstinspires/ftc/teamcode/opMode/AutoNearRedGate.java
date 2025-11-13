package org.firstinspires.ftc.teamcode.opMode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.types.VisionMode;

@Autonomous(name = "RedNearGate", preselectTeleOp = "TeleRed", group = "Red")
public class AutoNearRedGate extends AutoNear {
    public AutoNearRedGate() {
        super(VisionMode.VisionMode_RedGoal, false, 1, true);
    }
}
