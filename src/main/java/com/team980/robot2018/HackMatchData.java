package com.team980.robot2018;

import openrio.powerup.MatchData;

/**
 * This is disgusting
 */
@Deprecated
public class HackMatchData extends MatchData {

    static final String gsm = "RRR";

    public static OwnedSide getOwnedSide(GameFeature feature) {
        if (gsm == null)
            return OwnedSide.UNKNOWN;

        // If the length is less than 3, it's not valid. Longer than 3 is permitted, but only
        // the first 3 characters are taken.
        if (gsm.length() < 3)
            return OwnedSide.UNKNOWN;

        int index = feature.ordinal();
        // Theoretically this should never happen, but it's good to be safe.
        if (index >= 3 || index < 0)
            return OwnedSide.UNKNOWN;

        char gd = gsm.charAt(index);
        switch (gd) {
            case 'L':
            case 'l':
                return OwnedSide.LEFT;
            case 'R':
            case 'r':
                return OwnedSide.RIGHT;
            default:
                return OwnedSide.UNKNOWN;
        }
    }
}
