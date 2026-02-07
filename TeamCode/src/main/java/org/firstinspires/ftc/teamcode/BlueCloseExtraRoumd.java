package org.firstinspires.ftc.teamcode;

// RR-specific imports

import static org.firstinspires.ftc.teamcode.AutoLocations.BLUE_CLOSE_FIRST_SHOOT_POS;
import static org.firstinspires.ftc.teamcode.AutoLocations.BLUE_CLOSE_LEAVE;
import static org.firstinspires.ftc.teamcode.AutoLocations.BLUE_CLOSE_SPIKE_ONE_END;
import static org.firstinspires.ftc.teamcode.AutoLocations.BLUE_CLOSE_SPIKE_ONE_SHOOT_POS;
import static org.firstinspires.ftc.teamcode.AutoLocations.BLUE_CLOSE_SPIKE_ONE_START;
import static org.firstinspires.ftc.teamcode.AutoLocations.BLUE_CLOSE_SPIKE_TWO_END;
import static org.firstinspires.ftc.teamcode.AutoLocations.BLUE_CLOSE_SPIKE_TWO_SHOOT_POS;
import static org.firstinspires.ftc.teamcode.AutoLocations.BLUE_CLOSE_SPIKE_TWO_START;
import static org.firstinspires.ftc.teamcode.AutoLocations.FAST_SPLINE_MIN_SPEED;
import static org.firstinspires.ftc.teamcode.AutoLocations.BLUE_CLOSE_INITIAL_ANGLE;
import static org.firstinspires.ftc.teamcode.AutoLocations.LAUNCH_TIME;
import static org.firstinspires.ftc.teamcode.AutoLocations.SLOWER_INTAKE_MIN_SPEED;
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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Config
@Autonomous(name = "Blue Close New", group = "Autonomous")

public class BlueCloseExtraRoumd extends LinearOpMode{
    private static final boolean USE_WEBCAM = true;

    public MecanumDrive drive ;
    private double fTrajectoryWait = 0.04;

    public double shootYpos = -35;
    public void reportPosition(){
        telemetry.addData("Current Position", this.drive.localizer.getPose());
        telemetry.update();
    }
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(BLUE_CLOSE_INITIAL_ANGLE));
        drive = new MecanumDrive(hardwareMap, initialPose);
        Intake intake1 = new Intake(hardwareMap);
        Launcher outtake1 = new Launcher(hardwareMap);

        int visionOutputPosition = 1;

        // DRIVE TO POSITION
        TrajectoryActionBuilder path1 = drive.actionBuilder(initialPose)
                .setTangent(0.0)
//                .lineToX(-24)
                .splineToConstantHeading(BLUE_CLOSE_FIRST_SHOOT_POS, 0,new TranslationalVelConstraint(FAST_SPLINE_MIN_SPEED+20))
                .waitSeconds(fTrajectoryWait);
        Action trajectoryActionChosen1 = path1.build();

        waitForStart();
        if (isStopRequested()) return;

        Actions.runBlocking(new ParallelAction(outtake1.startLauncher(CLOSE_OUTTAKE_VELOCITY+50), trajectoryActionChosen1));


        telemetry.update();



        if (isStopRequested()) return;

        Actions.runBlocking(
                new SequentialAction(
                        outtake1.startLauncher(CLOSE_OUTTAKE_VELOCITY + 115),
                        new SleepAction(0.1),
                        intake1.intakeLaunch(),
                        new SleepAction(LAUNCH_TIME),
                        intake1.intakeOff()
                )
        );

        // TRAVEL TO SECOND SPIKE MARK
        TrajectoryActionBuilder spike2 = path1.endTrajectory()
                .fresh()
                .splineToLinearHeading(new Pose2d(BLUE_CLOSE_SPIKE_TWO_START,Math.toRadians(90)), 0,new TranslationalVelConstraint(FAST_SPLINE_MIN_SPEED+10))
                .waitSeconds(fTrajectoryWait)
                .lineToY(BLUE_CLOSE_SPIKE_TWO_END, new TranslationalVelConstraint(SLOWER_INTAKE_MIN_SPEED))
                .waitSeconds(fTrajectoryWait);
        Action Spike2 = spike2.build();

        // TRAVEL BACK TO SHOOTER
        TrajectoryActionBuilder toShooter2 = spike2.endTrajectory()
                .fresh()
                .splineToLinearHeading(BLUE_CLOSE_SPIKE_TWO_SHOOT_POS, 0, new TranslationalVelConstraint(FAST_SPLINE_MIN_SPEED))
                .waitSeconds(fTrajectoryWait);
        Action ToShooter2 = toShooter2.build();

        // OPEN GATE
        TrajectoryActionBuilder openGate = spike2.endTrajectory()
                .fresh()
                /*
                .splineToConstantHeading(BLUE_CLOSE_OPEN_GATE_START,0)
                .waitSeconds(fTrajectoryWait)
                .strafeToConstantHeading(BLUE_CLOSE_OPEN_GATE_END,new TranslationalVelConstraint(OPEN_GATE_SPEED)
                 */
                .splineToConstantHeading(new Vector2d(-47, 3.5),0)
                .waitSeconds(fTrajectoryWait);
        Action openGateAction = openGate.build();

        // INTAKE
        TrajectoryActionBuilder openGateIntake = openGate.endTrajectory()
                .fresh()
                .splineToLinearHeading(new Pose2d(new Vector2d(-63,4),Math.toRadians(45)),0)
//                .splineToConstantHeading(new Vector2d(-63,4),0)
                .waitSeconds(fTrajectoryWait);
//                .lineToY(7, new TranslationalVelConstraint(12));
        Action openGateintake = openGateIntake.build();

        // TRAVEL BACK TO SHOOTER
        TrajectoryActionBuilder toShooter3 = openGateIntake.endTrajectory()
                .fresh()
                .splineToLinearHeading(new Pose2d(new Vector2d(-37.5, -21),Math.toRadians(50)), 0, new TranslationalVelConstraint(FAST_SPLINE_MIN_SPEED))
                .waitSeconds(fTrajectoryWait)
                .splineToLinearHeading(new Pose2d(new Vector2d(-22, -31),Math.toRadians(50)), 0, new TranslationalVelConstraint(FAST_SPLINE_MIN_SPEED))
                .waitSeconds(fTrajectoryWait);
        Action ToShooter3 = toShooter3.build();

        // TRAVEL TO FIRST SPIKE MARK
        TrajectoryActionBuilder spike1 = toShooter3.endTrajectory()
                .fresh()
//                .splineTo(BLUE_CLOSE_SPIKE_ONE_START, Math.toRadians(120))
                .splineToLinearHeading(new Pose2d(BLUE_CLOSE_SPIKE_ONE_START,Math.toRadians(90)),0)
                .waitSeconds(fTrajectoryWait)
                .lineToY(BLUE_CLOSE_SPIKE_ONE_END, new TranslationalVelConstraint(SLOWER_INTAKE_MIN_SPEED))
                .waitSeconds(fTrajectoryWait);
        Action Spike1 = spike1.build();

        // TRAVEL BACK TO SHOOTER
        TrajectoryActionBuilder toShooter1 = spike1.endTrajectory()
                .fresh()
                .splineToLinearHeading(BLUE_CLOSE_SPIKE_ONE_SHOOT_POS, 0, new TranslationalVelConstraint(FAST_SPLINE_MIN_SPEED)) // like a z facing towards 90
                .waitSeconds(fTrajectoryWait);
        Action ToShooter1 = toShooter1.build();

        Actions.runBlocking(
                new SequentialAction(
//                        intake1.intakeOff(),
                        intake1.intakeOn(),
                        outtake1.startLauncher(CLOSE_OUTTAKE_VELOCITY+10),
                        Spike2,
                        intake1.intakeOff(),
                        new SleepAction(0.05),
                        ToShooter2,
                        new SleepAction(0.05),
                        intake1.intakeLaunch(),
                        new SleepAction(LAUNCH_TIME),
                        intake1.intakeOff(),
                        openGateAction
                ));

        Actions.runBlocking(
                new SequentialAction(
                        intake1.intakeOn(),
                        openGateintake,
                        intake1.intakeOff(),
                        ToShooter3,
                        new SleepAction(0.05),
                        intake1.intakeLaunch(),
                        new SleepAction(LAUNCH_TIME),
                        intake1.intakeOff()
                ));

        Actions.runBlocking(
                new SequentialAction(
                        intake1.intakeOn(),
                        Spike1,
                        intake1.intakeOff(),
                        ToShooter1,
                        new SleepAction(0.05),
                        intake1.intakeLaunch(),
                        new SleepAction(LAUNCH_TIME),
                        intake1.intakeOff()
                ));

        // LEAVE
        TrajectoryActionBuilder leave = toShooter1.endTrajectory()
                .fresh()
                .splineToLinearHeading(new Pose2d(BLUE_CLOSE_LEAVE,Math.toRadians(0)),0,new TranslationalVelConstraint(FAST_SPLINE_MIN_SPEED+10));
//                .splineToConstantHeading(BLUE_CLOSE_LEAVE,0.0,new TranslationalVelConstraint(BLUE_CLOSE_FASTER_SPLINE_VELOCITY+10))
//                .waitSeconds(fTrajectoryWait);
        Action Leave = leave.build();


        Actions.runBlocking(
                new SequentialAction(
                        outtake1.startLauncher(CLOSE_OUTTAKE_VELOCITY+30),
                        intake1.intakeOn(),
                        Spike2,
                        intake1.intakeOff(),
                        openGateAction,
                        new SleepAction(0.05),
                        ToShooter2,
                        new SleepAction(0.05),
                        intake1.intakeLaunch(),
                        new SleepAction(LAUNCH_TIME),
                        intake1.intakeOff(),
                        Leave
                )
        );

    }
}