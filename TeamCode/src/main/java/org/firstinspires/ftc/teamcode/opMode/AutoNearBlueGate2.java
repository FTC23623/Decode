package org.firstinspires.ftc.teamcode.opMode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.types.VisionMode;

@Autonomous(name = "BlueNearGate2", preselectTeleOp = "TeleBlue", group = "Blue")
public class AutoNearBlueGate2 extends AutoNear {
    public AutoNearBlueGate2() {
        super(VisionMode.VisionMode_BlueGoal, true, 2, 1);
    }
}
