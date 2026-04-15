package org.firstinspires.ftc.teamcode.opMode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.types.VisionMode;

@Autonomous(name = "BlueFarNoTurn2", preselectTeleOp = "TeleBlue", group = "Blue")
public class AutoFarNoTurnBlue2 extends AutoFarNoTurn {
    public AutoFarNoTurnBlue2() {
        super(VisionMode.VisionMode_BlueGoal, true, 2);
    }
}
