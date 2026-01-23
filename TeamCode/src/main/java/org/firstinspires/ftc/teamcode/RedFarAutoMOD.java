package org.firstinspires.ftc.teamcode;

// RR-specific imports

import static org.firstinspires.ftc.teamcode.OrcaRoboticsConstants.FAR_OUTTAKE_VELOCITY;
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
@Autonomous(name = "Red Far Auto mod", group = "Autonomous")
public class RedFarAutoMOD extends LinearOpMode {
    private static final boolean USE_WEBCAM = true;

    public MecanumDrive drive;

    public void reportPosition() {
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
//        Pivot pivot = new Pivot(hardwareMap);
        BallManager ballManager = new BallManager(hardwareMap);


        TrajectoryActionBuilder path1 = drive.actionBuilder(initialPose)
                .setTangent(0.0)
                .splineToLinearHeading(new Pose2d(new Vector2d(10.0, 2), Math.toRadians(-19.0)), Math.toRadians(0.0))
                .waitSeconds(trajectoryWait);
        Action trajectoryActionChosen1 = path1.build();
//        pivot.farPivot();
/*
        TrajectoryActionBuilder path2 = path1.endTrajectory()
                .fresh()
                .setTangent(0.0)
                .splineToLinearHeading(new Pose2d(new Vector2d(23, -19.5), Math.toRadians(-70)), Math.toRadians(0.0))
                .waitSeconds(trajectoryWait)
                .lineToY(-30, new TranslationalVelConstraint(17.0))
                .waitSeconds(trajectoryWait);
        Action trajectoryActionChosen2 = path2.build();

        TrajectoryActionBuilder toShooterP1 = path2.endTrajectory()
                .fresh()
                .splineToLinearHeading(new Pose2d(new Vector2d(9.5, -1.5), Math.toRadians(-25)), Math.toRadians(0.0))
                .waitSeconds(trajectoryWait);
        Action toShooterA1 = toShooterP1.build();
*/
        TrajectoryActionBuilder path3 = path1.endTrajectory()
                .fresh()
                .setTangent(0.0)
                .splineToLinearHeading(new Pose2d(new Vector2d(9, -24), Math.toRadians(-100)), Math.toRadians(0.0))
                .waitSeconds(trajectoryWait)
                .lineToY(-45, new TranslationalVelConstraint(17.0))
                .waitSeconds(trajectoryWait);
        Action trajectoryActionChosen3 = path3.build();

        TrajectoryActionBuilder toShooterP2 = path3.endTrajectory()
                .fresh()
                .splineToLinearHeading(new Pose2d(new Vector2d(10, -1.5), Math.toRadians(335)), Math.toRadians(0.0))
                .waitSeconds(trajectoryWait);
        Action toShooterA2 = toShooterP2.build();

        TrajectoryActionBuilder leavePath = toShooterP2.endTrajectory()
                .fresh()
                .splineToConstantHeading(new Vector2d(10,-13),0)
                .waitSeconds(trajectoryWait);
        Action leave = leavePath.build();

        waitForStart();
        if (isStopRequested()) return;

        int aprilTagId = aprilTagDetector.fetchAprilTag();
        telemetry.addData("April Tag Id", aprilTagId);
        telemetry.update();

        if (aprilTagId == 21) {
            Actions.runBlocking(
                    new SequentialAction(
                            new SleepAction(5),
                            outtake1.startLauncher(FAR_OUTTAKE_VELOCITY + 170),
                            trajectoryActionChosen1,
                            blocker.l_Engaged_Speed_Check(),
                            new SleepAction(0.1),
                            new ParallelAction(blocker.r_Engaged(), intake1.intakeOn()), // purple ball #1 end
                            new SleepAction(0.2),
                            new ParallelAction(blocker.l_Disengaged(), blocker.r_Disengaged()), // green ball #1 end // purple ball #1 start
                            new SleepAction(0.6),
                            blocker.r_Engaged(),
                            new SleepAction(0.3),
                            blocker.r_Disengaged(),
                            new SleepAction(0.7)
                    )
            );

        } else if (aprilTagId == 22) { // PGP
            Actions.runBlocking(
                    new SequentialAction(
                            new SleepAction(5),
                            outtake1.startLauncher(FAR_OUTTAKE_VELOCITY + 170),
                            trajectoryActionChosen1,
                            blocker.r_Engaged_Speed_Check(),
                            new SleepAction(0.2),
                            new ParallelAction(blocker.r_Disengaged(), blocker.l_Engaged()),
                            intake1.intakeOn(),
                            new SleepAction(0.5),
                            new ParallelAction(blocker.r_Engaged(), blocker.l_Disengaged()),
                            new SleepAction(0.3),
                            blocker.r_Disengaged(),
                            new SleepAction(0.7)
                    )
            );
        } else { //PPG
            Actions.runBlocking(
                    new SequentialAction(
                            new SleepAction(5),
                            outtake1.startLauncher(FAR_OUTTAKE_VELOCITY + 170),
                            trajectoryActionChosen1,
                            blocker.r_Engaged_Speed_Check(),
                            new SleepAction(0.2),
                            blocker.r_Disengaged(),
                            intake1.intakeOn(),
                            new SleepAction(0.5),
                            blocker.r_Engaged(),
                            new SleepAction(0.4),
                            outtake1.startLauncher(FAR_OUTTAKE_VELOCITY + 180),
                            new ParallelAction(blocker.l_Engaged(), blocker.r_Disengaged()),
                            new SleepAction(0.3),
                            blocker.l_Disengaged(),
                            new SleepAction(0.7)
                    )
            );
        }
        int numOfBalls = ballManager.getNumOfBalls();
        telemetry.addData("Number of Balls", numOfBalls);
        telemetry.update();
        if (numOfBalls > 0) {
            Actions.runBlocking(
                    new SequentialAction(
                            blocker.r_Engaged(),
                            new SleepAction(0.2),
                            blocker.l_Engaged(),
                            new SleepAction(0.3),
                            new ParallelAction(blocker.r_Disengaged(), blocker.l_Disengaged()),
                            new SleepAction(0.5)
                    )
            );
        }

        Actions.runBlocking(
                new SequentialAction(
//                        trajectoryActionChosen2,
//                        outtake1.startLauncher(FAR_OUTTAKE_VELOCITY + 40),
//                        intake1.intakeOff(),
//                        new SleepAction(0.6),
//                        toShooterA1,
//                        intake1.intakeOn(),
//                        new SleepAction(0.6),
//                        blocker.r_Engaged(),
//                        new SleepAction(0.2),
//                        blocker.l_Engaged(),
//                        new SleepAction(0.2),
//                        blocker.r_Disengaged(),
//                        new SleepAction(0.1),
//                        blocker.l_Disengaged(),
//                        new SleepAction(0.4),
//                        blocker.r_Engaged(),
//                        new SleepAction(0.1),
//                        blocker.l_Engaged(),
//                        new SleepAction(0.2),
//                        new ParallelAction(blocker.r_Disengaged(), blocker.l_Disengaged()),
                        new SleepAction(0.6),
                        outtake1.startLauncher(FAR_OUTTAKE_VELOCITY+10),
                        trajectoryActionChosen3,
                        intake1.intakeOff(),
                        new SleepAction(0.6),
                        toShooterA2,
                        intake1.intakeOn(),
                        new SleepAction(0.3),
                        blocker.r_Engaged(),
                        new SleepAction(0.1),
                        blocker.l_Engaged(),
                        new SleepAction(0.1),
                        blocker.r_Disengaged(),
                        new SleepAction(0.1),
                        blocker.l_Disengaged(),
                        outtake1.startLauncher(FAR_OUTTAKE_VELOCITY+20),
                        new SleepAction(0.4),
                        blocker.r_Engaged(),
                        new SleepAction(0.1),
                        blocker.l_Engaged(),
                        new SleepAction(0.2),
                        new ParallelAction(blocker.r_Disengaged(), blocker.l_Disengaged()),
                        new SleepAction(0.6),
                        leave
                        )
        );
    }
}