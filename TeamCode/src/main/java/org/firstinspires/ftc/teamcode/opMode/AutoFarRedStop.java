package org.firstinspires.ftc.teamcode.opMode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.types.VisionMode;

@Autonomous(name = "RedFarStop", preselectTeleOp = "TeleRed", group = "Red")
public class AutoFarRedStop extends AutoFarStop {
    public AutoFarRedStop() {
        super(VisionMode.VisionMode_RedGoal, false, 0);
    }
}
