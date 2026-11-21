package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import static com.pedropathing.api.Paths.*;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.commands.Commands.instant;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.OpModeStorage;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterIndexerSubsystemV3;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

@Autonomous(name = "Ivy Pedro Sequential Shooter Auto V7", group = "Autonomous")
public class PedroIvyLimelightAutoV7 extends OpMode {

    private Follower follower;
    private VisionSubsystem vision;
    private IntakeSubsystem intake;
    private ShooterIndexerSubsystemV3 shooter;

    private Path approachPath;
    private Path parkEndPath;

    // --- Alliance selection ---
    private enum Alliance { RED, BLUE }
    private Alliance currentAlliance = Alliance.RED;
    private boolean lastGamepad1X = false;
    private boolean lastGamepad1B = false;

    // --- Red route (field inches, corner origin, red frame) ---
    private final Pose redStartPose = new Pose(53.3, 6.5, Math.toRadians(270));
    private final Pose redTargetAreaPose = new Pose(53.3, 12, Math.toRadians(270));

    // Curve around the Red HIVE (x 46 to 48.6, y 50 to 91.4) to the park end
    private final Pose redParkCtrl1 = new Pose(30.75, 33.75, 0);
    private final Pose redParkCtrl2 = new Pose(27.25, 82.94, 0);
    private final Pose redParkEndPose = new Pose(14.10, 90.93, Math.toRadians(329));

    // Active poses, selected in start()
    private Pose startPose;
    private Pose targetAreaPose;
    private Pose parkCtrl1;
    private Pose parkCtrl2;
    private Pose parkEndPose;

    // --- Shot and align settings ---
    private static final int BALL_COUNT = 4;          // preloaded pollen
    private static final long ALIGN_TIMEOUT_MS = 3000;

    @Override
    public void init() {
        Scheduler.reset();
        OpModeStorage.autonomousEndPose = null; // never hand a stale pose to TeleOp

        follower = Constants.create(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
        vision = new VisionSubsystem(hardwareMap);
        shooter = new ShooterIndexerSubsystemV3(hardwareMap, vision);
        vision.switchPipeline(0);

        telemetry.addData("Status", "Initialized. Select Alliance on Gamepad1.");
    }

    @Override
    public void init_loop() {
        if (gamepad1.b && !lastGamepad1B) currentAlliance = Alliance.RED;
        if (gamepad1.x && !lastGamepad1X) currentAlliance = Alliance.BLUE;
        lastGamepad1B = gamepad1.b;
        lastGamepad1X = gamepad1.x;

        telemetry.addLine("=== ALLIANCE CONFIGURATION ===");
        telemetry.addData("Selected Alliance", currentAlliance == Alliance.RED ? "RED (B)" : "BLUE (X)");
        telemetry.addData("Press B for red, X for blue", "");
        telemetry.update();
    }

    @Override
    public void start() {
        vision.startCamera();
        boolean red = currentAlliance == Alliance.RED;
        OpModeStorage.isRedAlliance = red;

        if (red) {
            startPose = redStartPose;
            targetAreaPose = redTargetAreaPose;
            parkCtrl1 = redParkCtrl1;
            parkCtrl2 = redParkCtrl2;
            parkEndPose = redParkEndPose;
        } else {
            startPose = toBlue(redStartPose);
            targetAreaPose = toBlue(redTargetAreaPose);
            parkCtrl1 = toBlue(redParkCtrl1);
            parkCtrl2 = toBlue(redParkCtrl2);
            parkEndPose = toBlue(redParkEndPose);
        }
        shooter.setAlliance(red);

        follower.setPose(startPose);

        // Step 1: start to shooting area
        Command driveToZone = Command.build()
                .setStart(() -> {
                    approachPath = line(startPose, targetAreaPose).linear(startPose, targetAreaPose);
                    follower.follow(approachPath);
                })
                .setDone(() -> !follower.isBusy());

        // Step 2: align to the hive tags (shared with TeleOp aim logic)
        Command alignToHive = shooter.alignToTags(follower, ALIGN_TIMEOUT_MS);

        // Step 3: spin up, shoot all balls, spin down (shared shot functions)
        Command shoot = sequential(
                shooter.turnFlywheelOn(),
                shooter.shootAll(BALL_COUNT),
                shooter.turnFlywheelOff()
        );

        // Step 4: curve around the hive to the park end point
        Command driveToParkEnd = Command.build()
                .setStart(() -> {
                    Pose s = follower.pose();
                    parkEndPath = curve(s, parkCtrl1, parkCtrl2, parkEndPose).linear(s, parkEndPose);
                    follower.follow(parkEndPath);
                })
                .setDone(() -> !follower.isBusy());

        // Step 5: store the final pose for TeleOp
        Command saveFinalPose = instant(() -> OpModeStorage.autonomousEndPose = follower.pose());

        Scheduler.schedule(
                sequential(
                        driveToZone,
                        alignToHive,
                        shoot,
                        driveToParkEnd,
                        saveFinalPose
                )
        );
    }

    @Override
    public void loop() {
        follower.update();
        shooter.update(follower.pose()); // same speed logic as TeleOp
        Scheduler.execute();

        // Keep the TeleOp handoff current even if AUTO is stopped early
        OpModeStorage.autonomousEndPose = follower.pose();

        telemetry.addData("Alliance Status", currentAlliance == Alliance.RED ? "RED" : "BLUE");
        telemetry.addData("Robot Pose", follower.pose().toString());
        telemetry.addData("Flywheel target / actual (tps)",
                String.format("%.0f / %.0f", shooter.getTargetSpeed(), shooter.getActualSpeed()));
        telemetry.addData("Flywheel ready", shooter.isAtSpeed());
        telemetry.update();
    }

    // BIOBUZZ field is rotated 180 degrees between alliances, not mirrored
    private static Pose toBlue(Pose p) {
        return new Pose(144.0 - p.x(), 144.0 - p.y(), normalizeAngle(p.heading() + Math.PI));
    }

    private static double normalizeAngle(double a) {
        return Math.atan2(Math.sin(a), Math.cos(a));
    }
}
