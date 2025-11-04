package org.firstinspires.ftc.teamcode.datalogger;

public class HydraDatalogger {
    protected Datalogger datalogger;
    protected final String mName;

    HydraDatalogger(String name) {
        // append the date to the filename
        java.util.Date now = new java.util.Date(System.currentTimeMillis());
        String nowString = "";
        nowString += now;
        nowString.replace(":", "-");
        mName = name + "-" + nowString;
    }

    protected void Builder(Datalogger.LoggableField... fields) {
        // Build the underlying datalog object
        datalogger = new Datalogger.Builder()

                // Pass through the filename
                .setFilename(mName)

                // Request an automatic timestamp field
                .setAutoTimestamp(Datalogger.AutoTimestamp.DECIMAL_SECONDS)

                // Tell it about the fields we care to log.
                // Note that order *IS* important here! The order in which we list
                // the fields is the order in which they will appear in the log.
                .setFields(fields)
                .build();
    }

    // Tell the datalogger to gather the values of the fields
    // and write a new line in the log.
    public void writeLine()
    {
        datalogger.writeLine();
    }
}
