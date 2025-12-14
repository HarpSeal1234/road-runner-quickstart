package org.firstinspires.ftc.teamcode;

// RR-specific imports
import static org.firstinspires.ftc.teamcode.OrcaRoboticsConstants.CLOSE_OUTTAKE_VELOCITY;
import static org.firstinspires.ftc.teamcode.OrcaRoboticsConstants.trajectoryWait;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Config
@Autonomous(name = "Red Close Auto", group = "Autonomous")

public class redclose12 extends LinearOpMode{
    private static final boolean USE_WEBCAM = true;

    public MecanumDrive drive ;
    public void reportPosition(){
        telemetry.addData("Current Position", this.drive.localizer.getPose());
        telemetry.update();
    }
    public void runOpMode() {
        // instantiate your MecanumDrive at a particular pose.
        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(0));
        drive = new MecanumDrive(hardwareMap, initialPose);
        AprilTagDetector aprilTagDetector = new AprilTagDetector(hardwareMap);
        Intake intake1 = new Intake(hardwareMap);
        Launcher outtake1 = new Launcher(hardwareMap);
        Avocado blocker = new Avocado(hardwareMap);
        Pivot pivot = new Pivot(hardwareMap);


        int visionOutputPosition = 1;

        waitForStart();
        if (isStopRequested()) return;

        TrajectoryActionBuilder path1 = drive.actionBuilder(initialPose)
                .setTangent(0.0)
                .splineToConstantHeading(new Vector2d(-24, 36), 0)
                .waitSeconds(trajectoryWait);
        pivot.closePivot();
        Action trajectoryActionChosen1 = path1.build();

        Actions.runBlocking(new ParallelAction(outtake1.startLauncher(CLOSE_OUTTAKE_VELOCITY+200), trajectoryActionChosen1));
        int aprilTagId = aprilTagDetector.fetchAprilTag();
        telemetry.addData("April Tag Id", aprilTagId);
        telemetry.update();

        TrajectoryActionBuilder path2 = path1.endTrajectory()
                .fresh()
                .turn(Math.toRadians(-45))
                .waitSeconds(trajectoryWait);
        Action trajectoryActionChosen2 = path2.build();

        if (isStopRequested()) return;

        if (aprilTagId == 21) { // GPP  PURPLE ON THE RIGHT
            Actions.runBlocking(
                    new SequentialAction(
                            outtake1.startLauncher(CLOSE_OUTTAKE_VELOCITY + 90),
                            trajectoryActionChosen2,
                            new SleepAction(0.2),
                            blocker.l_Engaged(),
                            new SleepAction(0.2),
                            new ParallelAction(blocker.r_Engaged(), blocker.l_Disengaged(), intake1.intakeOn()), // purple ball #1 end
                            new SleepAction(0.2),
                            new ParallelAction(blocker.r_Disengaged()), // green ball #1 end // purple ball #1 start
                            new SleepAction(0.5),
                            blocker.r_Engaged(),
                            new SleepAction(0.2),
                            blocker.r_Disengaged(),
                            new SleepAction(0.3)
                    )
            );

        } else if (aprilTagId == 22) { // PGP
            Actions.runBlocking(
                    new SequentialAction(
                            outtake1.startLauncher(CLOSE_OUTTAKE_VELOCITY + 90),
                            trajectoryActionChosen2,
                            new SleepAction(0.2),
                            blocker.r_Engaged(),
                            new SleepAction(0.3),
                            new ParallelAction(blocker.r_Disengaged(), blocker.l_Engaged()),
                            intake1.intakeOn(),
                            new SleepAction(0.2),
                            blocker.l_Disengaged(),
                            new SleepAction(0.3),
                            blocker.r_Engaged(),
//                            new ParallelAction(blocker.r_Engaged(), blocker.l_Disengaged()),
                            new SleepAction(0.2),
                            blocker.r_Disengaged(),
                            new SleepAction(0.3)
                    )
            );
        } else { //PPG
            Actions.runBlocking(
                    new SequentialAction(
                            outtake1.startLauncher(CLOSE_OUTTAKE_VELOCITY + 90),
                            trajectoryActionChosen2,
                            new SleepAction(0.2),
                            intake1.intakeOn(),
                            blocker.r_Engaged(),
                            new SleepAction(0.2),
                            blocker.r_Disengaged(),
                            new SleepAction(0.5),
                            blocker.r_Engaged(),
                            new SleepAction(0.2),
                            new ParallelAction(blocker.l_Engaged(), blocker.r_Disengaged()),
                            new SleepAction(0.3),
                            blocker.l_Disengaged(),
                            new SleepAction(0.3)
                    ) // launch 0.2 disengage 0.5 engage 0.2 diengate
            );
        }
        /*
        Actions.runBlocking(
                new SequentialAction(
                        outtake1.startLauncher(CLOSE_OUTTAKE_VELOCITY+90),
                        trajectoryActionChosen2,
                        new SleepAction(0.2),
                        new ParallelAction(blocker.l_Engaged(), blocker.r_Engaged()), // green ball #1 end // purple ball #1 start
                        new SleepAction(0.2),
                        new ParallelAction(blocker.l_Disengaged(), blocker.r_Disengaged(),intake1.intakeOn()), // purple ball #1 end
                        new SleepAction(0.5),
                        new ParallelAction(blocker.l_Engaged(), blocker.r_Engaged()), // green ball #1 end // purple ball #1 start
                        new SleepAction(0.3),
                    new ParallelAction(blocker.l_Disengaged(), blocker.r_Disengaged()) // purple ball #1 end
                )
        );*/

        TrajectoryActionBuilder path3 = path2.endTrajectory()
                .fresh()
                .splineTo(new Vector2d(-29.0, 15), Math.toRadians(-120))
                .waitSeconds(trajectoryWait)
                .lineToY(3, new TranslationalVelConstraint(17.0));
        Action trajectoryActionChosen3 = path3.build();

        TrajectoryActionBuilder toShooter = path3.endTrajectory()
                .fresh()
                .splineToLinearHeading(new Pose2d(new Vector2d(-24, 34),Math.toRadians(-54)), 0); // like a z facing towards 90
//                .waitSeconds(0.6);
        Action trajectoryActionToShooterR1 = toShooter.build();

        Actions.runBlocking(
                new SequentialAction(
                        trajectoryActionChosen3,
                        intake1.intakeOff(),
                        trajectoryActionToShooterR1,
                        intake1.intakeOn(),
                        new SleepAction(0.2),
                        new ParallelAction(blocker.l_Engaged(), blocker.r_Engaged()), // green ball #1 end // purple ball #1 start
                        new SleepAction(0.2),
                        new ParallelAction(blocker.l_Disengaged(), blocker.r_Disengaged()), // purple ball #1 end
                        new SleepAction(0.6),
                        new ParallelAction(blocker.l_Engaged(), blocker.r_Engaged()), // green ball #1 end // purple ball #1 start
                        new SleepAction(0.4),
                        new ParallelAction(blocker.l_Disengaged(), blocker.r_Disengaged()) // purple ball #1 end
                )
        );

        reportPosition();

        TrajectoryActionBuilder path4 = toShooter.endTrajectory()
                .fresh()
                .splineToLinearHeading(new Pose2d(new Vector2d(-49, 24),Math.toRadians(-115)), 0)
                .waitSeconds(trajectoryWait)
                .lineToY(5, new TranslationalVelConstraint(17.0));
//                .lineToX(-68, new TranslationalVelConstraint(17.0));

        Action trajectoryActionChosen4 = path4.build();

        TrajectoryActionBuilder toShooter2 = path4.endTrajectory()
                .fresh()
                .splineToLinearHeading(new Pose2d(new Vector2d(-24, 36),Math.toRadians(-57)), 0)
                .waitSeconds(trajectoryWait);

        Action trajectoryActionToShooterR2 = toShooter2.build();


        Actions.runBlocking(
                new SequentialAction(
                        outtake1.startLauncher(CLOSE_OUTTAKE_VELOCITY+60),
                        trajectoryActionChosen4,
                        intake1.intakeOff(),
                        trajectoryActionToShooterR2,
                        new SleepAction(0.2),
                        new ParallelAction(blocker.l_Engaged(), blocker.r_Engaged()), // green ball #1 end // purple ball #1 start
                        new SleepAction(0.2),
                        new ParallelAction(blocker.l_Disengaged(), blocker.r_Disengaged(),intake1.intakeOn()), // purple ball #1 end
                        new SleepAction(0.5),
                        new ParallelAction(blocker.l_Engaged(), blocker.r_Engaged()), // green ball #1 end // purple ball #1 start
                        new SleepAction(0.8),
                        new ParallelAction(blocker.l_Disengaged(), blocker.r_Disengaged()), // purple ball #1 end
                        new ParallelAction(blocker.l_Disengaged(), blocker.r_Disengaged())// purple ball #1 end
                )
        );

        TrajectoryActionBuilder leave = toShooter2.endTrajectory()
                .fresh()
                .strafeTo(new Vector2d(-24,5));
//                .splineToConstantHeading(new Vector2d(-24,10),0);
        Action leaveAction = leave.build();

        Actions.runBlocking(
                new SequentialAction(
                        leaveAction
                )
        );
        reportPosition();
        sleep(500);
    }
}

