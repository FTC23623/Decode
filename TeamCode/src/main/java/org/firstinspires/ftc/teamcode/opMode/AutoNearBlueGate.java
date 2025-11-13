package org.firstinspires.ftc.teamcode.opMode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.types.VisionMode;

@Autonomous(name = "BlueNearGate", preselectTeleOp = "TeleBlue", group = "Blue")
public class AutoNearBlueGate extends AutoNear {
    public AutoNearBlueGate() {
        super(VisionMode.VisionMode_BlueGoal, true, 1, true);
    }
}
