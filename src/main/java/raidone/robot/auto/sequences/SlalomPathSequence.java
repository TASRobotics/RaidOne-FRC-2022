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
    //     new Point(90, -75, 0),
    //     new Point(215, -75, 0),
    //     new Point(310, 0, -45)
    // };
    private static final Point[] FRONT_CURVE_WAYPOINTS = {
        new Point(30,30,0)
    };
    private static final Path FRONT_CURVE_PATH = new Path(FRONT_CURVE_WAYPOINTS, false,
        7.5, DriveConstants.DEFAULT_TARGET_ACCELERATION);

    // private static final Point[] BACK_CURVE_WAYPOINTS = {
    //     new Point(310, -70, -180),
    //     new Point(215, 0, -180),
    //     new Point(90, 0, -180),
    //     new Point(0, -70, -180)
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