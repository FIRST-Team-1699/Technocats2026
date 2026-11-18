package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.OpModeStorage;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterIndexerSubsystemV3;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

@TeleOp(name = "MainTeleop W Vel V4", group = "TeleOp")
public class MainTeleOpV4 extends OpMode {

    private Follower follower;
    private IntakeSubsystem intake;
    private ShooterIndexerSubsystemV3 shooter;
    private VisionSubsystem vision;

    private enum Alliance { RED, BLUE }
    private Alliance alliance = Alliance.RED;
    private boolean bLastInit = false;
    private boolean xLastInit = false;

    private double headingOffset = 0.0;   // radians
    private boolean startLast = false;

    // Edge-trigger state
    private boolean rightBumper2Last = false;
    private boolean cross2Last = false;

    @Override
    public void init() {
        Scheduler.reset();

        follower = Constants.create(hardwareMap);

        // Use the pose saved by the autonomous, or the origin if none was saved
        Pose savedPose = OpModeStorage.autonomousEndPose != null
                ? OpModeStorage.autonomousEndPose
                : new Pose(0.0, 0.0, 0.0);
        follower.setPose(savedPose);

        intake = new IntakeSubsystem(hardwareMap);
        vision = new VisionSubsystem(hardwareMap);
        shooter = new ShooterIndexerSubsystemV3(hardwareMap, vision);

        alliance = OpModeStorage.isRedAlliance ? Alliance.RED : Alliance.BLUE;
        shooter.setAlliance(alliance == Alliance.RED);
        headingOffset = follower.pose().heading();
    }

    @Override
    public void init_loop() {
        if (gamepad1.b && !bLastInit) alliance = Alliance.RED;
        if (gamepad1.x && !xLastInit) alliance = Alliance.BLUE;
        bLastInit = gamepad1.b;
        xLastInit = gamepad1.x;
        shooter.setAlliance(alliance == Alliance.RED);

        telemetry.addData("Alliance", alliance == Alliance.RED ? "RED (B)" : "BLUE (X)");
        telemetry.addData("Press B for red, X for blue", "");
        telemetry.update();
    }

    @Override
    public void start() {
        vision.startCamera();
    }

    @Override
    public void loop() {

        // Driver presses START while the robot faces away from them
        if (gamepad1.start && !startLast) {
            headingOffset = follower.pose().heading();
        }
        startLast = gamepad1.start;

        // ==========================================
        // GAMEPAD 1: DRIVE WITH HOLD-TO-AIM
        // ==========================================
        // Holding left bumper aims at the hive tags. The subsystem returns the turn power.
        double turn = shooter.aimTurn(gamepad1.left_bumper, gamepad1.right_stick_x);

        DrivePowers powers = ManualDrive.fieldCentric(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                turn,
                follower.pose().heading() - headingOffset
        );
        follower.manual(powers);
        follower.update();

        // ==========================================
        // GAMEPAD 2: INTAKE
        // ==========================================
        if (gamepad2.right_trigger > 0.1) {
            intake.setPower(gamepad2.right_trigger);
        } else if (gamepad2.left_trigger > 0.1) {
            intake.setPower(-gamepad2.left_trigger);
        } else {
            intake.setPower(0.0);
        }

        // ==========================================
        // GAMEPAD 2: SHOOTER
        // ==========================================
        // Circle toggles the flywheel
        if (gamepad2.circle && !cross2Last) {
            shooter.setFlywheelOn(!shooter.isFlywheelOn());
        }
        cross2Last = gamepad2.circle;

        // Sets the flywheel speed from the current distance every loop
        shooter.update(follower.pose());

        // Right bumper shoots one ball, only when aimed and at speed
        if (gamepad2.right_bumper && !rightBumper2Last && shooter.isAimed() && shooter.isAtSpeed()) {
            Scheduler.schedule(shooter.shootBall());
        }
        rightBumper2Last = gamepad2.right_bumper;

        // ==========================================
        // PERSISTENT PROCESSING ENGINE
        // ==========================================
        Scheduler. execute();

        // Telemetry
        String aimState = !gamepad1.left_bumper ? "off"
                : shooter.isAimed() ? "LOCKED"
                : shooter.hasAimTarget() ? "AIMING"
                : "NO TAG";

        telemetry.addData("X Position", follower.pose().x());
        telemetry.addData("Y Position", follower.pose().y());
        telemetry.addData("Robot Heading", Math.toDegrees(follower.pose().heading()));
        telemetry.addData("Flywheel", shooter.isFlywheelOn() ? "ON" : "OFF");
        telemetry.addData("Flywheel target / actual (tps)",
                String.format("%.0f / %.0f", shooter.getTargetSpeed(), shooter.getActualSpeed()));
        telemetry.addData("Flywheel ready", shooter.isAtSpeed());
        telemetry.addData("Aim (hold LB)", aimState);
        telemetry.addData("Tag tx raw / filtered",
                String.format("%.2f / %.2f", shooter.getRawTx(), shooter.getFilteredTx()));
        telemetry.addData("Shot distance (in)", shooter.shotDistance(follower.pose()));
        telemetry.update();
    }
}
