package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import org.firstinspires.ftc.teamcode.datalogger.SystemDatalogger;
import org.firstinspires.ftc.teamcode.objects.HydraOpMode;
import org.firstinspires.ftc.teamcode.objects.Subsystem;

public class SystemMonitor implements Subsystem {
    private final HydraOpMode op;
    private final VoltageSensor voltageSensor;
    private final AnalogInput floodgateAin;
    private final SystemDatalogger logger;
    public SystemMonitor(HydraOpMode opMode) {
        op = opMode;
        voltageSensor = op.mHardwareMap.get(VoltageSensor.class, "Control Hub");
        floodgateAin = op.mHardwareMap.get(AnalogInput.class, "Floodgate");
        logger = new SystemDatalogger("System");
    }

    @Override
    public boolean Init() {
        return true;
    }

    @Override
    public void Process() {
        double voltage = voltageSensor.getVoltage();
        double current = 50 * floodgateAin.getVoltage() / 3.3;
        op.mTelemetry.addData("Voltage", voltage);
        op.mTelemetry.addData("Current", current);
        logger.current.set(current);
        logger.voltage.set(voltage);
        logger.writeLine();
    }

    @Override
    public void HandleUserInput() {

    }
}
