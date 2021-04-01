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
    //private static final Point[] TEST_CURVE_WAYPOINTS = {
        //new Point(0, 0, 0),
        //new Point(60, 0, 0)
    //};
    //private static final Path TEST_CURVE_PATH = new Path(TEST_CURVE_WAYPOINTS, true,
        //DriveConstants.DEFAULT_CRUISE_VELOCITY, DriveConstants.DEFAULT_TARGET_ACCELERATION);

    private static final Point[] FIRST_CURVE_WAYPOINTS = {
        new Point(0, 0, 0),
        new Point(45, 60, 90) //A3 -- (50, 100, 90)
    };
    private static final Path FIRST_CURVE_PATH = new Path(FIRST_CURVE_WAYPOINTS, false,
        DriveConstants.DEFAULT_CRUISE_VELOCITY, DriveConstants.DEFAULT_TARGET_ACCELERATION);

    private static final Point[] SECOND_CURVE_WAYPOINTS = {
        new Point(0, 0, 90), //comments are old points for reference purposes
        new Point(-30, 95), //(-15, 100)
        new Point(-60, 120, 180), //(-60, 200, 180)
        new Point(-85, 95),//(-110, 100)
        new Point(-90, 0, -90) //A6 -- (-135, 0, -90)
    };
    private static final Path SECOND_CURVE_PATH = new Path(SECOND_CURVE_WAYPOINTS, true,
    DriveConstants.DEFAULT_CRUISE_VELOCITY, DriveConstants.DEFAULT_TARGET_ACCELERATION);

    private static final Point[] THIRD_CURVE_WAYPOINTS = {
        new Point(0, 0, -90),
        new Point(10, -95), //(15, -100)
        new Point(30, -110, 0), //(60, -200, 0)
        new Point(60, -110), //(110, -100)
        new Point(80, -95), //newly added
        new Point(90, 0, 90) //A9 -- (135, 0, 90)
    };
    private static final Path THIRD_CURVE_PATH = new Path(THIRD_CURVE_WAYPOINTS, false,
    DriveConstants.DEFAULT_CRUISE_VELOCITY, DriveConstants.DEFAULT_TARGET_ACCELERATION);

    private static final Point[] FOURTH_CURVE_WAYPOINTS = {
        new Point(0, 0, 90),
        //new Point(-35, 65),
        new Point(-60, 60, 180) //(-50, 65, 180)
    };
    private static final Path FOURTH_CURVE_PATH = new Path(FOURTH_CURVE_WAYPOINTS, true,
    DriveConstants.DEFAULT_CRUISE_VELOCITY, DriveConstants.DEFAULT_TARGET_ACCELERATION);
 
    
    public BouncePathSequence() {
        System.out.println(DriverStation.getInstance().getAlliance().name());
    }

    @Override
    public void sequence() {
        addAction(new SeriesAction(
            Arrays.asList(
                // new ParallelAction(
                //     Arrays.asList(
                //         new DrivePath(TEST_CURVE_PATH)
                //     )
                // )
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
        return "Bounce Path Sequence";
    }
}