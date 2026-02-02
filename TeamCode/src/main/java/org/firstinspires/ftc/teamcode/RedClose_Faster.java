package org.firstinspires.ftc.teamcode;

// RR-specific imports

import static org.firstinspires.ftc.teamcode.AutoLocations.BLUE_CLOSE_FASTER_SPLINE_VELOCITY;
import static org.firstinspires.ftc.teamcode.AutoLocations.BLUE_CLOSE_LEAVE;
import static org.firstinspires.ftc.teamcode.AutoLocations.BLUE_CLOSE_SPIKE_ONE_FINAL_Y;
import static org.firstinspires.ftc.teamcode.AutoLocations.BLUE_CLOSE_SPIKE_ONE_MIN_VELOCITY;
import static org.firstinspires.ftc.teamcode.AutoLocations.BLUE_CLOSE_SPIKE_ONE_START;
import static org.firstinspires.ftc.teamcode.AutoLocations.BLUE_CLOSE_SPIKE_TWO_FINAL_Y;
import static org.firstinspires.ftc.teamcode.AutoLocations.BLUE_CLOSE_SPIKE_TWO_MIN_VELOCITY;
import static org.firstinspires.ftc.teamcode.AutoLocations.BLUE_CLOSE_SPIKE_TWO_START;
import static org.firstinspires.ftc.teamcode.OrcaRoboticsConstants.CLOSE_OUTTAKE_VELOCITY;
import static org.firstinspires.ftc.teamcode.OrcaRoboticsConstants.trajectoryWait;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Config
@Disabled

@Autonomous(name = "Red Close Fast", group = "Autonomous")

public class RedClose_Faster extends LinearOpMode{
    private static final boolean USE_WEBCAM = true;

    public MecanumDrive drive ;
    private double fTrajectoryWait = 0.04;

    public double shootYpos = -35;
    public void reportPosition(){
        telemetry.addData("Current Position", this.drive.localizer.getPose());
        telemetry.update();
    }
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(0));
        drive = new MecanumDrive(hardwareMap, initialPose);
        AprilTagDetector aprilTagDetector = new AprilTagDetector(hardwareMap);
        Intake intake1 = new Intake(hardwareMap);
        Launcher outtake1 = new Launcher(hardwareMap);
        Avocado blocker = new Avocado(hardwareMap);
//        Pivot pivot = new Pivot(hardwareMap);
        BallManager ballManager = new BallManager(hardwareMap);

        int visionOutputPosition = 1;

        // DRIVE TO POSITION
        TrajectoryActionBuilder path1 = drive.actionBuilder(initialPose)
                .setTangent(0.0)
                .splineToConstantHeading(new Vector2d(-24, 35), 0)
                .waitSeconds(fTrajectoryWait);
        Action trajectoryActionChosen1 = path1.build();
//        pivot.closePivot();

        waitForStart();
        if (isStopRequested()) return;

        Actions.runBlocking(new ParallelAction(outtake1.startLauncher(CLOSE_OUTTAKE_VELOCITY+200), trajectoryActionChosen1,aprilTagDetector.detectAprilTag()));

        int aprilTagId = aprilTagDetector.getDesiredTagId();
        int ballNumber;
        BallManager.DecodeBallColor rfColor = ballManager.detectRightFrontColor();
        BallManager.DecodeBallColor rbColor = ballManager.detectRightBackColor();
        BallManager.DecodeBallColor lfColor = ballManager.detectLeftFrontColor();
        BallManager.DecodeBallColor lbColor = ballManager.detectLeftBackColor();
        telemetry.addData("April Tag Id", aprilTagId);
        telemetry.addData("rf", rfColor);
        telemetry.addData("rb", rbColor);
        telemetry.addData("lf", lfColor);
        telemetry.addData("lb", lbColor);
//        telemetry.addData("number of balls", ballNumber);

        telemetry.update();

        // TURN TO SHOOT
        TrajectoryActionBuilder path2 = path1.endTrajectory()
                .fresh()
                .turn(Math.toRadians(-47));
        Action trajectoryActionChosen2 = path2.build();

        // TRAVEL TO FIRST SPIKE MARK
        TrajectoryActionBuilder path3 = path2.endTrajectory()
                .fresh()
                .splineTo(new Vector2d(-29,15), Math.toRadians(-120))
                .waitSeconds(fTrajectoryWait)
                .lineToY(3, new TranslationalVelConstraint(BLUE_CLOSE_SPIKE_ONE_MIN_VELOCITY))
                .waitSeconds(fTrajectoryWait);
        Action trajectoryActionChosen3 = path3.build();

        // TRAVEL BACK TO SHOOTER
        TrajectoryActionBuilder toShooter = path3.endTrajectory()
                .fresh()
                .splineToLinearHeading(new Pose2d(new Vector2d(-24, 33),Math.toRadians(-54)), 0, new TranslationalVelConstraint(BLUE_CLOSE_FASTER_SPLINE_VELOCITY)) // like a z facing towards 90
                .waitSeconds(fTrajectoryWait);
        Action trajectoryActionToShooterR1 = toShooter.build();

        // TRAVEL TO SECOND SPIKE MARK
        TrajectoryActionBuilder path4 = toShooter.endTrajectory()
                .fresh()
                .splineToLinearHeading(new Pose2d(new Vector2d(-49,24),Math.toRadians(-115)), 0,new TranslationalVelConstraint(BLUE_CLOSE_FASTER_SPLINE_VELOCITY))
                .waitSeconds(fTrajectoryWait)
                .lineToY(5, new TranslationalVelConstraint(BLUE_CLOSE_SPIKE_TWO_MIN_VELOCITY))
                .waitSeconds(fTrajectoryWait);
        Action trajectoryActionChosen4 = path4.build();

        // TRAVEL BACK TO SHOOTER
        TrajectoryActionBuilder toShooter2 = path4.endTrajectory()
                .fresh()
                .splineToLinearHeading(new Pose2d(new Vector2d(-24, 35),Math.toRadians(-57)), 0, new TranslationalVelConstraint(BLUE_CLOSE_FASTER_SPLINE_VELOCITY))
                .waitSeconds(fTrajectoryWait);
        Action trajectoryActionToShooterR2 = toShooter2.build();

        // LEAVE
        TrajectoryActionBuilder leavePath = toShooter2.endTrajectory()
                .fresh()
                .splineToConstantHeading(new Vector2d(-24,10),0.0,new TranslationalVelConstraint(BLUE_CLOSE_FASTER_SPLINE_VELOCITY+10))
                .waitSeconds(fTrajectoryWait);
        Action leave = leavePath.build();

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
                            new ParallelAction(blocker.r_Engaged(), blocker.l_Engaged()),
                            new SleepAction(0.2),
                            new ParallelAction(blocker.l_Disengaged(), blocker.r_Disengaged()),
                            new SleepAction(1)
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
                            new ParallelAction(blocker.r_Engaged(), blocker.l_Engaged()),
                            new SleepAction(0.2),
                            new ParallelAction(blocker.l_Disengaged(), blocker.r_Disengaged()),
                            new SleepAction(1)
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
                            new SleepAction(1)
                    ) // launch 0.2 disengage 0.5 engage 0.2 diengate
            );
        }
        ballNumber = ballManager.getNumOfBalls();
        if (ballNumber > 0) {
            Actions.runBlocking(
                    new SequentialAction(
                            new ParallelAction(blocker.r_Engaged()), blocker.l_Engaged(),
                            new SleepAction(0.2),
                            new ParallelAction(blocker.l_Disengaged(), blocker.r_Disengaged())
                    )
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
                        new SleepAction(0.5),
                        new ParallelAction(blocker.l_Engaged(), blocker.r_Engaged()), // green ball #1 end // purple ball #1 start
                        new SleepAction(0.4),
                        new ParallelAction(blocker.l_Disengaged(), blocker.r_Disengaged()) // purple ball #1 end
                )
        );

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
                        new SleepAction(0.5),
                        new ParallelAction(blocker.l_Disengaged(), blocker.r_Disengaged()), // purple ball #1 end
                        leave
                )
        );

    }
}
