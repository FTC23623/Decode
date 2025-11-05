package org.firstinspires.ftc.teamcode.opMode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.types.VisionMode;

@Autonomous(name = "RedNear1", preselectTeleOp = "TeleRed")
public class AutoNearRed1 extends AutoNear {
    public AutoNearRed1() {
        super(VisionMode.VisionMode_RedGoal, false, 1);
    }
}
