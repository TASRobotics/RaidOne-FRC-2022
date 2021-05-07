package raidone.robot.auto.sequences;

import java.util.Arrays;

import edu.wpi.first.wpilibj.DriverStation;
import raidone.pathgen.Point;
import raidone.robot.Constants.DriveConstants;
import raidone.robot.auto.actions.*;
import raidone.robot.pathing.Path;
// import raidone.robot.submodules.Drive;
// import raidone.robot.submodules.Intake;
// import raidone.robot.submodules.Shooter;

public class SlalomPathSequence extends AutoSequence {

    private static final Point[] FIRST_CURVE_WAYPOINTS = {
        new Point(0, 0, 0),
        new Point(55, 0),
        new Point(50, 75),
        new Point(90, 75),
        new Point(150, 75, 0)
    };
    private static final Path FIRST_CURVE_PATH = new Path(FIRST_CURVE_WAYPOINTS, false,
        DriveConstants.DEFAULT_CRUISE_VELOCITY, DriveConstants.DEFAULT_TARGET_ACCELERATION);

    private static final Point[] SECOND_CURVE_WAYPOINTS = {
        new Point(0, 0, 0),
        new Point(42, 0),
        new Point(70, -53),
        new Point(118, -80, 0)
    };
    private static final Path SECOND_CURVE_PATH = new Path(SECOND_CURVE_WAYPOINTS, false,
        DriveConstants.DEFAULT_CRUISE_VELOCITY, DriveConstants.DEFAULT_TARGET_ACCELERATION);
    
    private static final Point[] THIRD_CURVE_WAYPOINTS = {
        new Point(0, 0, 0),
        new Point(28, 49),
        new Point(28, 73, 90)
    };
    private static final Path THIRD_CURVE_PATH = new Path(THIRD_CURVE_WAYPOINTS, false,
        DriveConstants.DEFAULT_CRUISE_VELOCITY, DriveConstants.DEFAULT_TARGET_ACCELERATION);
    
    private static final Point[] FOURTH_CURVE_WAYPOINTS = {
        new Point(0, 0, 90),
        new Point(0, 10),
        new Point(-62, -30),
        new Point(-62, -62, -90)
    };
    private static final Path FOURTH_CURVE_PATH = new Path(FOURTH_CURVE_WAYPOINTS, false,
        DriveConstants.DEFAULT_CRUISE_VELOCITY, DriveConstants.DEFAULT_TARGET_ACCELERATION);
    
    private static final Point[] FIFTH_CURVE_WAYPOINTS = {
        new Point(0, 0, -90),
        new Point(-30, -20, -180),
        new Point(-150, -5, -180),
    };
    private static final Path FIFTH_CURVE_PATH = new Path(FIFTH_CURVE_WAYPOINTS, false,
        DriveConstants.DEFAULT_CRUISE_VELOCITY, DriveConstants.DEFAULT_TARGET_ACCELERATION);
        
    private static final Point[] SIXTH_CURVE_WAYPOINTS = {
        new Point(0, 0, -180),
        new Point(-5, 70, -270),
        // new Point(-30, 82, 90),
        new Point(-40, 96, -225)
    };
    private static final Path SIXTH_CURVE_PATH = new Path(SIXTH_CURVE_WAYPOINTS, false,
        DriveConstants.DEFAULT_CRUISE_VELOCITY, DriveConstants.DEFAULT_TARGET_ACCELERATION);
    
    public SlalomPathSequence() {
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
        System.out.println("Slalom Path Sequence ended!");
    }

    @Override
    public String getName() {
        return "Slalom Path Sequence";
    }
}