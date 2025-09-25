package org.firstinspires.ftc.teamcode.datalogger;

public class ObjDetDatalogger extends HydraDatalogger {
    // These are all of the fields that we want in the datalog.
    // Note that order here is NOT important. The order is important in the setFields() call below
    public Datalogger.GenericField loops = new Datalogger.GenericField("Loop");
    public Datalogger.GenericField state = new Datalogger.GenericField("State");
    public Datalogger.GenericField camState = new Datalogger.GenericField("Cam State");
    public Datalogger.GenericField numObjDet = new Datalogger.GenericField("Object Count");
    public Datalogger.GenericField objX = new Datalogger.GenericField("Object X");
    public Datalogger.GenericField objY = new Datalogger.GenericField("Object Y");
    public Datalogger.GenericField objConf = new Datalogger.GenericField("Object Conf");
    public Datalogger.GenericField numAprilDet = new Datalogger.GenericField("April Count");
    public Datalogger.GenericField aprilName = new Datalogger.GenericField("April Name");
    public Datalogger.GenericField aprilRange = new Datalogger.GenericField("April Range");
    public Datalogger.GenericField aprilBearing = new Datalogger.GenericField("April Bearing");
    public Datalogger.GenericField aprilYaw = new Datalogger.GenericField("April Yaw");

    public ObjDetDatalogger(String name) {
        super(name);
        Builder(
                    loops,
                    state,
                    camState,
                    numObjDet,
                    objX,
                    objY,
                    objConf,
                    numAprilDet,
                    aprilName,
                    aprilRange,
                    aprilBearing,
                    aprilYaw
                );
    }
}
