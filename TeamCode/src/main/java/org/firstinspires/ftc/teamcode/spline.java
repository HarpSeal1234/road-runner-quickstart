package org.firstinspires.ftc.teamcode;// RR-specific imports

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Config
@Autonomous(name = "splineto4848LINEAR", group = "Autonomous")

public class spline extends LinearOpMode{
    private static final boolean USE_WEBCAM = true;
    public final static int FAR_OUTTAKE_VELOCITY = 1700;
    public final static int CLOSE_OUTTAKE_VELOCITY = 1400;
    public final static int MEDIUM_OUTTAKE_VELOCITY = 1200;

    public void runOpMode() {
        // instantiate your MecanumDrive at a particular pose.
        Pose2d initialPose = new Pose2d(0, 0, Math.PI / 2);
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        AprilTagDetector aprilTagDetector = new AprilTagDetector(hardwareMap);
        Intake intake1 = new Intake(hardwareMap);
        Launcher outtake1 = new Launcher(hardwareMap);
        Avocado blockerR = new Avocado(hardwareMap);
        Avocado blockerL = new Avocado(hardwareMap);
        Pivot pivot = new Pivot(hardwareMap);


        int visionOutputPosition = 1;

        TrajectoryActionBuilder path1 = drive.actionBuilder(initialPose)
                .setTangent(0)
//                .splineToConstantHeading(new Vector2d(48,48), Math.PI / 2);
                .splineToLinearHeading(new Pose2d(48,48,0), Math.PI / 2);
        waitForStart();

        if (isStopRequested()) return;
        Action trajectoryActionChosen;
        trajectoryActionChosen = path1.build();
        Actions.runBlocking(new ParallelAction(trajectoryActionChosen)); // RIGHT
        telemetry.addData("position", drive.localizer.getPose());
        telemetry.update();
        sleep(30000);

    }

    }




