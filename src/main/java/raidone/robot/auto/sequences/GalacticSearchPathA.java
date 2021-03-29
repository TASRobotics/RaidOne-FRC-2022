package raidone.robot.auto.sequences;
import raidone.pathgen.Point;
import raidone.robot.Constants.DriveConstants;
import raidone.robot.auto.actions.*;
import raidone.robot.pathing.Path;

import java.util.Arrays;

import edu.wpi.first.wpilibj.DriverStation;

public class GalacticSearchPathA extends AutoSequence {

    private static final Point[] FIRST_BLUE_WAYPOINTS = {
        new Point(0, 0, 0),
        new Point(-150, 60),
        new Point(-180, -60),
        new Point(-240, -30),
        new Point(-300, 0, -30)
    };
    private static final Path FIRST_BLUE_CURVE = new Path(FIRST_BLUE_WAYPOINTS, true,
        DriveConstants.DEFAULT_CRUISE_VELOCITY, DriveConstants.DEFAULT_TARGET_ACCELERATION);

    private static final Point[] FIRST_RED_WAYPOINTS = {
        new Point(0, 0, 0),
        new Point(60, 0),
        new Point(120, -30),
        new Point(150, -60, 60), 
        new Point(300, 0, -55)
    };
    private static final Path FIRST_RED_CURVE = new Path(FIRST_RED_WAYPOINTS, true,
        DriveConstants.DEFAULT_CRUISE_VELOCITY, DriveConstants.DEFAULT_TARGET_ACCELERATION);

    public GalacticSearchPathA() {
        System.out.println(DriverStation.getInstance().getAlliance().name());
    }

    @Override
    public void sequence() {
        addAction(new SeriesAction(
            Arrays.asList(
                new ParallelAction(
                    Arrays.asList(
                        new DrivePath(FIRST_BLUE_CURVE)
                    )
                )
            )
        ));
        // addAction(new SeriesAction(
        //     Arrays.asList(
        //         new ParallelAction(
        //             Arrays.asList(
        //                 new DrivePath(FIRST_RED_CURVE)
        //             )
        //         )
        //     )
        // ));
        System.out.println("Added actions.");
    }

    @Override
    public void onEnded() {
        
    }

    @Override
    public String getName() {
        return "Galactic Search Sequence";
    }
}