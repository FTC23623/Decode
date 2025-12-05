package org.firstinspires.ftc.teamcode.opMode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.types.VisionMode;

@Autonomous(name = "BlueNearDoubleGate", preselectTeleOp = "TeleBlue", group = "Blue")
public class AutoNearBlueDoubleGate extends AutoNear {
    public AutoNearBlueDoubleGate() {
        super(VisionMode.VisionMode_BlueGoal, true, 2, 2);
    }
}
