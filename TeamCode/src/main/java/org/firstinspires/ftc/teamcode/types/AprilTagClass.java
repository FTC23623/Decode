package org.firstinspires.ftc.teamcode.types;

public enum AprilTagClass {
    AprilTagClass_Unknown(0),
    AprilTagClass_BlueGoal(20),
    AprilTagClass_Obelisk_GPP(21),
    AprilTagClass_Obelisk_PGP(22),
    AprilTagClass_Obelisk_PPG(23),
    AprilTagClass_RedGoal(24);

    private int id;

    AprilTagClass(int id)
    {
        this.id = 0;
        for (AprilTagClass val : AprilTagClass.values()) {
            if (val.id == id) {
                this.id = id;
                break;
            }
        }
    }

    public int GetValue() {
        return id;
    }

    public static AprilTagClass FromInt(int id) {
        for (AprilTagClass val : AprilTagClass.values()) {
            if (val.id == id) {
                return val;
            }
        }
        return AprilTagClass_Unknown;
    }
}
