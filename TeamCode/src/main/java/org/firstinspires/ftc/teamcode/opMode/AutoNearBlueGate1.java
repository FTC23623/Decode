package org.firstinspires.ftc.teamcode.opMode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.types.VisionMode;

@Autonomous(name = "BlueNearGate1", preselectTeleOp = "TeleBlue", group = "Blue")
public class AutoNearBlueGate1 extends AutoNear {
    public AutoNearBlueGate1() {
        super(VisionMode.VisionMode_BlueGoal, true, 1, 1);
    }
}
