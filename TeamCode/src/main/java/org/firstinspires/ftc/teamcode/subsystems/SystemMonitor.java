package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.datalogger.Datalogger;
import org.firstinspires.ftc.teamcode.datalogger.SystemDatalogger;
import org.firstinspires.ftc.teamcode.objects.HydraOpMode;
import org.firstinspires.ftc.teamcode.objects.Subsystem;

import java.util.ArrayList;

public class SystemMonitor implements Subsystem {
    private final HydraOpMode op;
    private final VoltageSensor voltageSensor;
    private final AnalogInput floodgateAin;
    private final SystemDatalogger logger;
    private final ArrayList<MotorToLog> motors;
    public SystemMonitor(HydraOpMode opMode) {
        op = opMode;
        voltageSensor = op.mHardwareMap.get(VoltageSensor.class, "Control Hub");
        floodgateAin = op.mHardwareMap.get(AnalogInput.class, "Floodgate");
        logger = new SystemDatalogger("System");
        motors = new ArrayList<>();
        motors.add(new MotorToLog(op.mHardwareMap.get(DcMotorEx.class, "leftFront"), logger.driveLF));
        motors.add(new MotorToLog(op.mHardwareMap.get(DcMotorEx.class, "rightFront"), logger.driveRF));
        motors.add(new MotorToLog(op.mHardwareMap.get(DcMotorEx.class, "leftBack"), logger.driveLB));
        motors.add(new MotorToLog(op.mHardwareMap.get(DcMotorEx.class, "rightFront"), logger.driveRB));
        motors.add(new MotorToLog(op.mHardwareMap.get(DcMotorEx.class, "intakeMotor"), logger.intake));
        motors.add(new MotorToLog(op.mHardwareMap.get(DcMotorEx.class, "transferMotor"), logger.transfer));
        motors.add(new MotorToLog(op.mHardwareMap.get(DcMotorEx.class, "left"), logger.launch));
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
        for (MotorToLog motor : motors) {
            op.mTelemetry.addData(motor.loggerField.GetName() + " Current", motor.motor.getCurrent(CurrentUnit.AMPS));
            motor.loggerField.set(motor.motor.getCurrent(CurrentUnit.AMPS));
        }
        logger.writeLine();
    }

    @Override
    public void HandleUserInput() {

    }

    public static class MotorToLog {
        public final DcMotorEx motor;
        public final Datalogger.GenericField loggerField;
        public MotorToLog(DcMotorEx motor, Datalogger.GenericField loggerField) {
            this.motor = motor;
            this.loggerField = loggerField;
        }
    }
}
