package raidone.robot.auto.sequences;

import java.util.Arrays;

import edu.wpi.first.wpilibj.DriverStation;
import raidone.pathgen.Point;
import raidone.robot.auto.actions.*;
import raidone.robot.pathing.Path;


public class AlternateBigBallSequence {
    // Velocity variable in constants file --> DriveConstants.DEFAULT_CRUISE_VELOCITY

    private static final Point[] FIRST_CURVE_WAYPOINTS = {
        new Point(10, 0, 0),
        new Point(30, 0),
        new Point(50, 60, 90) //A3 -- (50, 100, 90)
    };
    private static final Path FIRST_CURVE_PATH = new Path(FIRST_CURVE_WAYPOINTS, false,
        2.5, 4);
}

    



