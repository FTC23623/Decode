package org.firstinspires.ftc.teamcode.objects;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class HydraOpMode {
    public Telemetry mTelemetry;
    public HardwareMap mHardwareMap;
    public com.qualcomm.robotcore.hardware.Gamepad mDriverGamepad;
    public com.qualcomm.robotcore.hardware.Gamepad mOperatorGamepad;
    public Vision mVision;

    public double mLoopTime;
    public HydraOpMode(Telemetry telemetry, HardwareMap hardwareMap,
                       com.qualcomm.robotcore.hardware.Gamepad driverGamepad,
                       com.qualcomm.robotcore.hardware.Gamepad operatorGamepad,
                       Vision vision) {
        mTelemetry = telemetry;
        mHardwareMap = hardwareMap;
        mDriverGamepad = driverGamepad;
        mOperatorGamepad = operatorGamepad;
        mVision = vision;
        mLoopTime = 0;
    }
}
