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

    // private static final Point[] FRONT_CURVE_WAYPOINTS = {
    //     new Point(-60, 90, 0),
    //     new Point(-60, 210, 0),
    //     new Point(0, 270, 0)
    // };
    private static final Point[] FRONT_CURVE_WAYPOINTS = {
        new Point(30,30,0)
    };
    private static final Path FRONT_CURVE_PATH = new Path(FRONT_CURVE_WAYPOINTS, false,
        7.5, DriveConstants.DEFAULT_TARGET_ACCELERATION);

    // private static final Point[] MIDDLE_CURVE_WAYPOINTS = {
    //     new Point(-30, 320, -90),
    //     new Point(-60, 270, -180),
    // };
    // private static final Path MIDDLE_CURVE_PATH = new Path(MIDDLE_CURVE_WAYPOINTS, true,
    //     7.5, DriveConstants.DEFAULT_TARGET_ACCELERATION);


    // private static final Point[] BACK_CURVE_WAYPOINTS = {
    //     new Point(0, 210, -180),
    //     new Point(0, 90, -180),
    //     new Point(-60, 0, -180)
    // };
    private static final Point[] BACK_CURVE_WAYPOINTS = {
        new Point(0,0,0)
    };
    private static final Path BACK_CURVE_PATH = new Path(BACK_CURVE_WAYPOINTS, true,
        7.5, DriveConstants.DEFAULT_TARGET_ACCELERATION);


    public SlalomPathSequence() {
        System.out.println(DriverStation.getInstance().getAlliance().name());
    }               

    @Override
    public void sequence() {
        addAction(new SeriesAction(
            Arrays.asList(
                new ParallelAction(
                    Arrays.asList(
                        new DrivePath(FRONT_CURVE_PATH)
                    )
                ),
                // new ParallelAction(
                //     Arrays.asList(
                //         new DrivePath(MIDDLE_CURVE_PATH)
                //     )
                // ),
                new ParallelAction(
                    Arrays.asList(
                        new DrivePath(BACK_CURVE_PATH)
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
        return "Slalom Path Sequence ended.";
    }
}