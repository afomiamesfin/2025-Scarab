package frc.robot;

public class Ports {
    /*
     *  CAN IDS
     */

    /* Drivetrain */
    public static final int FL_DRIVE = 12;
    public static final int FL_STEER = 10; // recovered
    public static final int FL_ENCODER = 11;

    public static final int FR_DRIVE = 3; // wrong # before
    public static final int FR_STEER = 21; // recovered
    public static final int FR_ENCODER = 2;

    public static final int BL_DRIVE = 9;
    public static final int BL_STEER = 13; // recovered (used to be 7)
    public static final int BL_ENCODER = 8;

    public static final int BR_DRIVE = 6;
    public static final int BR_STEER = 4; // not a spark max? (changed back 4)
    public static final int BR_ENCODER = 5;

    /* Intake */
    public static final int INTAKE = 13;

    /* Indexer */
    public static final int INDEX_LOWER = 14;
    public static final int INDEX_UPPER = 16;

    /* Shooter */
    public static final int SHOOTER_UPPER = 15;
    public static final int SHOOTER_LOWER = 17;  

    /* Climber */
    // public static final int CLIMBER_LEFT = 19;
    // public static final int CLIMBER_RIGHT = 21;

    /* Hood */
    public static final int HOOD_LEFT = 18;
    public static final int HOOD_RIGHT = 20;

    /*
     *  DIO
     */

    /* LED Pins */
    public static final int LED_CHANNEL_1_PIN = 3; // 2^0
    public static final int LED_CHANNEL_2_PIN = 4; // 2^1
    public static final int LED_CHANNEL_3_PIN = 5; // 2^2
    public static final int LED_CHANNEL_4_PIN = 6; // 2^3

    /* Indexer */
    public static final int LOADING_BEAM_BREAK = 8;
    public static final int STAGING_BEAM_BREAK = 0;

    /* Climber */
    public static final int CLIMBER_LIMIT_SWITCH = 7;
}
