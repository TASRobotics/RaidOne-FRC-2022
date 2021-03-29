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
        new Point(30, 135, 70),
        new Point(40, 120, 180),
        new Point(30, 105, -60),
        new Point(0, 140, -10)
    };
    private static final Path FIRST_LOOP_PATH = new Path(FIRST_LOOP_WAYPOINTS, false,
        7.5, DriveConstants.DEFAULT_TARGET_ACCELERATION);

    private static final Point[] SECOND_LOOP_WAYPOINTS = {
        new Point(-40, 259, -100),
        new Point(-40, 195, 135)
    };
    private static final Path SECOND_LOOP_PATH = new Path(SECOND_LOOP_WAYPOINTS, false,
        7.5, DriveConstants.DEFAULT_TARGET_ACCELERATION);

    private static final Point[] THIRD_LOOP_WAYPOINTS = {
        new Point(40, 250, 20),
        new Point(15, 290, -90),
        new Point(0, 250, -70)
    };
    private static final Path THIRD_LOOP_PATH = new Path(THIRD_LOOP_WAYPOINTS, false,
        7.5, DriveConstants.DEFAULT_TARGET_ACCELERATION);

    private static final Point[] FOURTH_LOOP_WAYPOINTS = {
        new Point(-20, 120, -180),
        new Point(-20, 30, -180)
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
        System.out.println("Bounce Path Sequence ended!");
    }

    @Override
    public String getName() {
        return "Bounce Path Sequence";
    }
}