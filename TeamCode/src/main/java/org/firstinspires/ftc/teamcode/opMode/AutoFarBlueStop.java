package org.firstinspires.ftc.teamcode.opMode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.types.VisionMode;

@Autonomous(name = "BlueFarStop", preselectTeleOp = "TeleBlue", group = "Blue")
public class AutoFarBlueStop extends AutoFarStop {
    public AutoFarBlueStop() {
        super(VisionMode.VisionMode_BlueGoal, true, 0);
    }
}
