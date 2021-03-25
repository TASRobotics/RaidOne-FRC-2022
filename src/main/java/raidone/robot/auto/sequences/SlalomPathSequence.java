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

public class SlalomPathSequence extends AutoSequence {

    private static final Point[] FIRST_CURVE_WAYPOINTS = {
        new Point(0, 0, 0),
        new Point(78, 53),
        new Point(192, 63),
        new Point(235, 30),
        new Point(268, -2, 0)
    };
    private static final Path FIRST_CURVE_PATH = new Path(FIRST_CURVE_WAYPOINTS, false,
        DriveConstants.DEFAULT_CRUISE_VELOCITY, DriveConstants.DEFAULT_TARGET_ACCELERATION);

    private static final Point[] SECOND_CURVE_WAYPOINTS = {
        new Point(0, 0, 0),
        new Point(28, 49),
        new Point(-2, 78),
        new Point(-40, 39),
        new Point(-133, -11, 180),
        new Point(-198, -3),
        new Point(-234, 35, 135),
        new Point(-250, 60, 180)
    };
    private static final Path SECOND_CURVE_PATH = new Path(SECOND_CURVE_WAYPOINTS, false,
        DriveConstants.DEFAULT_CRUISE_VELOCITY, DriveConstants.DEFAULT_TARGET_ACCELERATION);
    
    // private static final Point[] THIRD_CURVE_WAYPOINTS = {
    //     new Point(310, 30, -90),
    //     new Point(270, 60, 180),
    //     new Point(240, 30, 135),
    //     new Point(150, 0, 180)
    // };
    // private static final Path THIRD_CURVE_PATH = new Path(THIRD_CURVE_WAYPOINTS, false,
    //     DriveConstants.DEFAULT_CRUISE_VELOCITY, DriveConstants.DEFAULT_TARGET_ACCELERATION);
    
    // private static final Point[] FOURTH_CURVE_WAYPOINTS = {
    //     new Point(150, 0, 180),
    //     new Point(80, 10, -135),
    //     new Point(0, 60, 180)
    // };
    // private static final Path FOURTH_CURVE_PATH = new Path(FOURTH_CURVE_WAYPOINTS, false,
    //     DriveConstants.DEFAULT_CRUISE_VELOCITY, DriveConstants.DEFAULT_TARGET_ACCELERATION);
    
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
                )
                // new ParallelAction(
                //     Arrays.asList(
                //         new DrivePath(THIRD_CURVE_PATH)
                //     )
                // ),
                // new ParallelAction(
                //     Arrays.asList(
                //         new DrivePath(FOURTH_CURVE_PATH)
                //     )
                // )
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