package org.robolancers321.commands.auto;

/**
 * Marker interface bounding all possible sets of responses
 * 
 * To add an enum of options, make sure that the enum implements this marker interface
 */
public sealed interface Response {

    public enum YesNo implements Response {
        YES,
        NO
    }

    public enum FirstNote implements Response {
        CENTER1,
        CENTER2,
        SUBWOOFER
    }

    public enum StartLocation implements Response {
        AMP,
        MID,
        SOURCE
    }
}