package org.firstinspires.ftc.teamcode.types;

public enum DecodeAprilTag {
    DecodeTag_Unknown,
    DecodeTag_BlueGoal,
    DecodeTag_Obelisk_GPP,
    DecodeTag_Obelisk_PGP,
    DecodeTag_Obelisk_PPG,
    DecodeTag_RedGoal;

    public static int IdFromTag(DecodeAprilTag tag) {
        switch (tag) {
            case DecodeTag_BlueGoal:
                return 20;
            case DecodeTag_Obelisk_GPP:
                return 21;
            case DecodeTag_Obelisk_PGP:
                return 22;
            case DecodeTag_Obelisk_PPG:
                return 23;
            case DecodeTag_RedGoal:
                return 24;
        }
        return 0;
    }

    public static DecodeAprilTag TagFromId(int id) {
        switch (id) {
            case 20:
                return DecodeTag_BlueGoal;
            case 21:
                return DecodeTag_Obelisk_GPP;
            case 22:
                return DecodeTag_Obelisk_PGP;
            case 23:
                return DecodeTag_Obelisk_PPG;
            case 24:
                return DecodeTag_RedGoal;
        }
        return DecodeTag_Unknown;
    }
}
