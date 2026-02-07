package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

public class AutoLocations {
    // GENERAL
    public final static double FAST_SPLINE_MIN_SPEED = 70;
    public final static double SLOWER_INTAKE_MIN_SPEED = 19;
    public final static double OPEN_GATE_SPEED = 12;
    public final static double LAUNCH_TIME = 1.5;
    // ----------------------------------------------
    // BLUE CLOSE
    // ----------------------------------------------
    // INITIAL ANGLE
    public final static double BLUE_CLOSE_INITIAL_ANGLE = 48.5;
    // SPIKE ONE
    public final static Vector2d BLUE_CLOSE_FIRST_SHOOT_POS = new Vector2d(-24, -32);
    public final static Vector2d BLUE_CLOSE_SPIKE_ONE_START = new Vector2d(-37.5, -15);
    public final static double BLUE_CLOSE_SPIKE_ONE_END = -3;
    // ROUND ONE SHOOTING
    public final static Pose2d BLUE_CLOSE_SPIKE_ONE_SHOOT_POS = new Pose2d(new Vector2d(-24, -31),Math.toRadians(50));

    // SPIKE TWO
    public final static Vector2d BLUE_CLOSE_SPIKE_TWO_START = new Vector2d(-58, -24);
    public final static double BLUE_CLOSE_SPIKE_TWO_END = -2;
    // OPEN GATE
    public final static Vector2d BLUE_CLOSE_OPEN_GATE_START = new Vector2d(-45,-1);
    public final static Vector2d BLUE_CLOSE_OPEN_GATE_END = new Vector2d(-45,3);
    // ROUND TWO SHOOTING
    public final static Pose2d BLUE_CLOSE_SPIKE_TWO_SHOOT_POS = new Pose2d(new Vector2d(-22, -31),Math.toRadians(50));
    // LEAVE
    public final static Vector2d BLUE_CLOSE_LEAVE = new Vector2d(-27,-5);
    // ----------------------------------------------
    // RED CLOSE
    // ----------------------------------------------
    // INITIAL ANGLE
    public final static double RED_CLOSE_INITIAL_ANGLE = -48.5;
    // SPIKE ONE
    public final static Vector2d RED_CLOSE_FIRST_SHOOT_POS = new Vector2d(-24, 32);
    public final static Vector2d RED_CLOSE_SPIKE_ONE_START = new Vector2d(-37.5, 15);
    public final static double RED_CLOSE_SPIKE_ONE_END = 3;
    // ROUND ONE SHOOTING
    public final static Pose2d RED_CLOSE_SPIKE_ONE_SHOOT_POS = new Pose2d(new Vector2d(-24, 31),Math.toRadians(-50));
    // SPIKE TWO
    public final static Vector2d RED_CLOSE_SPIKE_TWO_START = new Vector2d(-58, 24);
    public final static double RED_CLOSE_SPIKE_TWO_END = 2;
    // OPEN GATE
    public final static Vector2d RED_CLOSE_OPEN_GATE_START = new Vector2d(-45,1);
    public final static Vector2d RED_CLOSE_OPEN_GATE_END = new Vector2d(-45,-3);
    // ROUND TWO SHOOTING
    public final static Pose2d RED_CLOSE_SPIKE_TWO_SHOOT_POS = new Pose2d(new Vector2d(-22, 31),Math.toRadians(-50));
    // LEAVE
    public final static Vector2d RED_CLOSE_LEAVE = new Vector2d(-27,5);






}
