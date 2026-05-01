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

public class TurretDatalogger extends HydraDatalogger
{
    // These are all of the fields that we want in the datalog.
    // Note that order here is NOT important. The order is important in the setFields() call below
    public Datalogger.GenericField visionSetPoint = new Datalogger.GenericField("VisionSP");
    public Datalogger.GenericField odoSetPoint = new Datalogger.GenericField("OdoSP");
    public Datalogger.GenericField fb = new Datalogger.GenericField("Fb");
    public Datalogger.GenericField lock = new Datalogger.GenericField("VisLock");
    public Datalogger.GenericField pid = new Datalogger.GenericField("pidOutput");

    public TurretDatalogger(String name) {
        super(name);
        Builder(visionSetPoint, odoSetPoint, fb, lock, pid);
    }
}
