package raidone.robot.auto.sequences;

import java.util.Arrays;

import edu.wpi.first.wpilibj.DriverStation;
import raidone.pathgen.Point;
import raidone.robot.auto.actions.*;
import raidone.robot.pathing.Path;

public class BouncePathSequence extends AutoSequence {

    private static final Point[] FIRST_CURVE_WAYPOINTS = {
        new Point(10, 0, 0),
        new Point(30, 0),
        new Point(50, 60, 90) //A3 -- (50, 100, 90)
    };
    private static final Path FIRST_CURVE_PATH = new Path(FIRST_CURVE_WAYPOINTS, false,
        2.5, 4);

    private static final Point[] SECOND_CURVE_WAYPOINTS = {
        new Point(0, 0, 90), 
        new Point(0, 40),
        new Point(-30, 95), 
        new Point(-60, 120, 180),
        new Point(-103, 120, 180) 
    };
    private static final Path SECOND_CURVE_PATH = new Path(SECOND_CURVE_WAYPOINTS, true,
    2.5, 4);

    private static final Point[] THIRD_CURVE_WAYPOINTS = {
        new Point(0, 0, 180),
        new Point(-5, 120, 90) 
    };
    private static final Path THIRD_CURVE_PATH = new Path(THIRD_CURVE_WAYPOINTS, false,
    2.5, 4);

    private static final Point[] FOURTH_CURVE_WAYPOINTS = {
        new Point(0, 0, 90),
        new Point(-40, 110, 180), 
        new Point(-105, 110, 180) 
    };
    private static final Path FOURTH_CURVE_PATH = new Path(FOURTH_CURVE_WAYPOINTS, true,
    2.5, 4);

    private static final Point[] FIFTH_CURVE_WAYPOINTS = {
        new Point(7, 0, 180),
        new Point(-3, 60),
        new Point(-3, 120, 90) 
    };
    private static final Path FIFTH_CURVE_PATH = new Path(FIFTH_CURVE_WAYPOINTS, false,
    2.5, 4);

    private static final Point[] SIXTH_CURVE_WAYPOINTS = {
        new Point(0, 0, 90),
        new Point(0, 45),
        new Point(-40, 50, 180) 
    };
    private static final Path SIXTH_CURVE_PATH = new Path(SIXTH_CURVE_WAYPOINTS, true,
    2.5, 4);
    
    public BouncePathSequence() {
        System.out.println(DriverStation.getInstance().getAlliance().name());
    }

    @Override
    public void sequence() {
        addAction(new SeriesAction(
            Arrays.asList(
                new ParallelAction(
                    Arrays.asList(
                        new DrivePath(FIRST_CURVE_PATH)
                    )
                ),
                new ParallelAction(
                    Arrays.asList(
                        new DrivePath(SECOND_CURVE_PATH)
                    )
                ),
                new ParallelAction(
                    Arrays.asList(
                        new DrivePath(THIRD_CURVE_PATH)
                    )
                ),
                new ParallelAction(
                    Arrays.asList(
                        new DrivePath(FOURTH_CURVE_PATH)
                    )
                ),
                new ParallelAction(
                    Arrays.asList(
                        new DrivePath(FIFTH_CURVE_PATH)
                    )
                ),
                new ParallelAction(
                    Arrays.asList(
                        new DrivePath(SIXTH_CURVE_PATH)
                    )
                )
            )
        ));
        System.out.println("Added actions.");
    }

    @Override
    public void onEnded() {
        System.out.println("Bounce Path Sequence ended!");
    }

    @Override
    public String getName() {
        return "Bounce Path Sequence";
    }
}