package frc.utils;

import edu.wpi.first.hal.HALUtil;

/**
 * Utility class for logging messages to the console.
 */
public class Logger {

    private static final int teamNumber = HALUtil.getTeamNumber();

    /**
     * Prints a log message to the console.
     *
     * @param input The message to be logged.
     */
    public static void log(String input){
        System.out.println("[" + teamNumber + " LOG]: " + input);
    }

    /**
     * Prints an error message to the console.
     *
     * @param input The error message to be logged.
     */
    public static void error(String input){
        System.err.println("[" + teamNumber + " ERROR]: " + input);
    }

}