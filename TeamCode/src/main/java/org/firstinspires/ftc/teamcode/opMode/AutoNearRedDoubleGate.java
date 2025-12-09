package org.firstinspires.ftc.teamcode.opMode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.types.VisionMode;

@Autonomous(name = "RedNearDoubleGate", preselectTeleOp = "TeleRed", group = "Red")
public class AutoNearRedDoubleGate extends AutoNear {
    public AutoNearRedDoubleGate() {
        super(VisionMode.VisionMode_RedGoal, false, 2, 2);
    }
}
