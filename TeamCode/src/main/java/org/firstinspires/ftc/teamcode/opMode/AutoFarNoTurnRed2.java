package org.firstinspires.ftc.teamcode.opMode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.types.VisionMode;

@Autonomous(name = "RedFarNoTurn2", preselectTeleOp = "TeleRed", group = "Red")
public class AutoFarNoTurnRed2 extends AutoFarNoTurn {
    public AutoFarNoTurnRed2() {
        super(VisionMode.VisionMode_RedGoal, false, 2);
    }
}
