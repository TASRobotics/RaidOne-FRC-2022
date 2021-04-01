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

public class BarrelPathSequence extends AutoSequence {

    private static final Point[] FIRST_LOOP_WAYPOINTS = {
        new Point(0, 0, 0),
        new Point(95, 0, -15),
        new Point(130, -30),
        new Point(100, -60),
        new Point(60, -30),
        new Point(130, 10, 0)
    };
    private static final Path FIRST_LOOP_PATH = new Path(FIRST_LOOP_WAYPOINTS, false,
        7.5, DriveConstants.DEFAULT_TARGET_ACCELERATION);

    private static final Point[] SECOND_LOOP_WAYPOINTS = {
        new Point(0, 0, 0),
        new Point(60, 0, 0),
        new Point(100, 30),
        new Point(60, 60),
        new Point(15, 20),
        new Point(15, 5),
        new Point(60, -25, -60)
    };
    private static final Path SECOND_LOOP_PATH = new Path(SECOND_LOOP_WAYPOINTS, false,
        7.5, DriveConstants.DEFAULT_TARGET_ACCELERATION);

    private static final Point[] THIRD_LOOP_WAYPOINTS = {
        new Point(0, 0 ,-60),
        new Point(65, -45),
        new Point(95, -15),
        new Point(60, 14, -180)
    };
    private static final Path THIRD_LOOP_PATH = new Path(THIRD_LOOP_WAYPOINTS, false,
        7.5, DriveConstants.DEFAULT_TARGET_ACCELERATION);

    private static final Point[] FOURTH_LOOP_WAYPOINTS = {
        new Point(0, 0, -180),
        new Point(-250, 20, -180)
    };
    private static final Path FOURTH_LOOP_PATH = new Path(FOURTH_LOOP_WAYPOINTS, false,
        7.5, DriveConstants.DEFAULT_TARGET_ACCELERATION);

    public BarrelPathSequence() {
        System.out.println(DriverStation.getInstance().getAlliance().name());
    }

    @Override
    public void sequence() {
        addAction(new SeriesAction(
            Arrays.asList(
                new ParallelAction(
                    Arrays.asList(
                        new DrivePath(FIRST_LOOP_PATH)
                    )
                ),
                new ParallelAction(
                    Arrays.asList(
                        new DrivePath(SECOND_LOOP_PATH)
                    )
                ),
                new ParallelAction(
                    Arrays.asList(
                        new DrivePath(THIRD_LOOP_PATH)
                    )
                ),
                new ParallelAction(
                    Arrays.asList(
                        new DrivePath(FOURTH_LOOP_PATH)
                    )
                )
            )
        ));
        System.out.println("Added actions.");
    }

    @Override
    public void onEnded() {
        System.out.println("Barrel Path Sequence ended!");
    }

    @Override
    public String getName() {
        return "Barrel Path Sequence";
    }
}