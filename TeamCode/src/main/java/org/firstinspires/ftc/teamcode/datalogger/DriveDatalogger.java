/*
This sample FTC OpMode uses methods of the Datalogger class to specify and
collect robot data to be logged in a CSV file, ready for download and charting.

For instructions, see the tutorial at the FTC Wiki:
https://github.com/FIRST-Tech-Challenge/FtcRobotController/wiki/Datalogging


The Datalogger class is suitable for FTC OnBot Java (OBJ) programmers.
Its methods can be made available for FTC Blocks, by creating myBlocks in OBJ.

Android Studio programmers can see instructions in the Datalogger class notes.

Credit to @Windwoes (https://github.com/Windwoes).

*/


package org.firstinspires.ftc.teamcode.datalogger;

public class DriveDatalogger extends HydraDatalogger
{
    // These are all of the fields that we want in the datalog.
    // Note that order here is NOT important. The order is important in the setFields() call below
    public Datalogger.GenericField loops = new Datalogger.GenericField("Loop");
    public Datalogger.GenericField fltarget = new Datalogger.GenericField("FL Tgt");
    public Datalogger.GenericField flposition = new Datalogger.GenericField("FL Pos");
    public Datalogger.GenericField frtarget = new Datalogger.GenericField("FR Tgt");
    public Datalogger.GenericField frposition = new Datalogger.GenericField("FR Pos");
    public Datalogger.GenericField bltarget = new Datalogger.GenericField("BL Tgt");
    public Datalogger.GenericField blposition = new Datalogger.GenericField("BL Pos");
    public Datalogger.GenericField brtarget = new Datalogger.GenericField("BR Tgt");
    public Datalogger.GenericField brposition = new Datalogger.GenericField("BR Pos");
    public Datalogger.GenericField drMotPwr = new Datalogger.GenericField("Dr Pwr");
    public Datalogger.GenericField battVoltage = new Datalogger.GenericField("Batt");
    public Datalogger.GenericField state = new Datalogger.GenericField("State");
    public Datalogger.GenericField yawError = new Datalogger.GenericField("Yaw Error");

    public DriveDatalogger(String name) {
        super(name);
        Builder(
                    loops,
                    battVoltage,
                    drMotPwr,
                    fltarget,
                    flposition,
                    frtarget,
                    frposition,
                    bltarget,
                    blposition,
                    brtarget,
                    brposition,
                    yawError
                );
    }
}
