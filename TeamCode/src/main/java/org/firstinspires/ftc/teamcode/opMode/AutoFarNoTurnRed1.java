package org.firstinspires.ftc.teamcode.opMode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.types.VisionMode;

@Autonomous(name = "RedFar1", preselectTeleOp = "TeleRed", group = "Red")
public class AutoFarNoTurnRed1 extends AutoFarNoTurn {
    public AutoFarNoTurnRed1() {
        super(VisionMode.VisionMode_RedGoal, false, 1);
    }
}
