package org.firstinspires.ftc.teamcode.AutoOld;

// RR-specific imports
import static org.firstinspires.ftc.teamcode.OrcaRoboticsConstants.CLOSE_OUTTAKE_VELOCITY;

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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.AprilTagDetector;
import org.firstinspires.ftc.teamcode.Avocado;
import org.firstinspires.ftc.teamcode.Intake;
import org.firstinspires.ftc.teamcode.Launcher;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Pivot;

@Config
@Disabled
@Autonomous(name = "red close", group = "Autonomous")

public class RedClosePathOnly_Actions extends LinearOpMode{
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

        // Look at april tag
        TrajectoryActionBuilder path1 = drive.actionBuilder(initialPose)
                .setTangent(0.0)
                .splineToConstantHeading(new Vector2d(-24, 36), 0)
                .waitSeconds(0.6);
        pivot.closePivot();
        Action trajectoryActionChosen1 = path1.build();

        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                outtake1.startLauncher(CLOSE_OUTTAKE_VELOCITY+200),
                                trajectoryActionChosen1,
                                aprilTagDetector.detectAprilTag()
                        ),
                        new SleepAction(0.3)
                )
        );
        int aprilTagId = aprilTagDetector.getDesiredTagId();
        telemetry.addData("April Tag Id", aprilTagId);
        telemetry.update();

        // TURN TO SHOOT
        TrajectoryActionBuilder path2 = path1.endTrajectory()
                .fresh()
                .turn(Math.toRadians(-45));
        Action trajectoryActionChosen2 = path2.build();

        // FETCH FIRST ROW
        TrajectoryActionBuilder path3 = path2.endTrajectory()
                .fresh()
                .splineTo(new Vector2d(-29.0, 15), Math.toRadians(-120))
//                .waitSeconds(0.6)
                .lineToY(3, new TranslationalVelConstraint(17.0));
//                .waitSeconds(0.6);
        Action trajectoryActionChosen3 = path3.build();



        // GO TO SHOOT
        TrajectoryActionBuilder toShooter = path3.endTrajectory()
                .fresh()
                .splineToLinearHeading(new Pose2d(new Vector2d(-24, 34),Math.toRadians(-54)), 0); // like a z facing towards 90
//                .waitSeconds(0.6);
        Action trajectoryActionToShooterR1 = toShooter.build();

        // FETCH SECOND ROW
        TrajectoryActionBuilder path4 = toShooter.endTrajectory()
                .fresh()
                .splineToLinearHeading(new Pose2d(new Vector2d(-49, 24),Math.toRadians(-115)), 0)
//                .waitSeconds(0.5)
                .lineToY(5, new TranslationalVelConstraint(17.0));
//                .waitSeconds(0.5);
        Action trajectoryActionChosen4 = path4.build();

        // GO TO SHOOT
        TrajectoryActionBuilder toShooter2 = path4.endTrajectory()
                .fresh()
                .splineToLinearHeading(new Pose2d(new Vector2d(-24, 34),Math.toRadians(-57)), 0);
//                .waitSeconds(0.5);
        Action trajectoryActionToShooterR2 = toShooter2.build();

        // LEAVE
        TrajectoryActionBuilder leave = toShooter2.endTrajectory()
                .fresh()
                .splineToConstantHeading(new Vector2d(-24,29),0.0);
//                .waitSeconds(0.5);
        Action leaveAction = leave.build();

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
                            outtake1.startLauncher(CLOSE_OUTTAKE_VELOCITY + 100),
                            new SleepAction(0.5),
                            blocker.r_Engaged(),
                            new SleepAction(0.2),
                            blocker.r_Disengaged(),
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
                            blocker.r_Engaged(),
//                            new ParallelAction(blocker.r_Engaged(), blocker.l_Disengaged()),
                            new SleepAction(0.2),
                            blocker.r_Disengaged(),
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
                        new SleepAction(0.8),
                        new ParallelAction(blocker.l_Disengaged(), blocker.r_Disengaged()), // purple ball #1 end
                        leaveAction
                )
        );

    }
}
