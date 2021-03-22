package raidone.robot.auto.sequences;

import java.util.Arrays;

import edu.wpi.first.wpilibj.DriverStation;
import raidone.pathgen.Point;
import raidone.robot.Constants.DriveConstants;
import raidone.robot.auto.actions.*;
import raidone.robot.pathing.Path;
import raidone.robot.submodules.Drive;
import raidone.robot.submodules.Intake;
import raidone.robot.submodules.Shooter;

public class BouncePathSequence extends AutoSequence {

    private static final Point[] FIRST_CURVE_WAYPOINTS = {
        new Point(50, -40, -45)
    };
    private static final Path FIRST_CURVE_PATH = new Path(FIRST_CURVE_WAYPOINTS, false,
        7.5, DriveConstants.DEFAULT_TARGET_ACCELERATION);

    private static final Point[] SECOND_CURVE_WAYPOINTS = {
        new Point(-70, -100, -45),
        new Point(50, -130, -45)
    };
    private static final Path SECOND_CURVE_PATH = new Path(SECOND_CURVE_WAYPOINTS, false,
        7.5, DriveConstants.DEFAULT_TARGET_ACCELERATION);

    private static final Point[] THIRD_CURVE_WAYPOINTS = {
        new Point(-55, -190, -45),
        new Point(50, -220, -45)
    };
    private static final Path THIRD_CURVE_PATH = new Path(THIRD_CURVE_WAYPOINTS, false,
        7.5, DriveConstants.DEFAULT_TARGET_ACCELERATION);

    private static final Point[] FOURTH_CURVE_WAYPOINTS = {
        new Point(0, -280, 180)
    };
    private static final Path FOURTH_CURVE_PATH = new Path(FOURTH_CURVE_WAYPOINTS, false,
        7.5, DriveConstants.DEFAULT_TARGET_ACCELERATION);

    
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
        return "Bounce Path Sequence ended.";
    }
}