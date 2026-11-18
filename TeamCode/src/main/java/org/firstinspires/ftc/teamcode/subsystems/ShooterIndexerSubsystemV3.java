package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.hold;

/**
 * Shared shooter logic for TeleOp and Auto.
 * Owns the flywheel speed, the indexer, hive AprilTag aiming, and shot distance.
 * Both OpModes call update(pose) every loop and use the same shot and align functions.
 */
public class ShooterIndexerSubsystemV3 {

    // Hive pivot midpoints in field inches, red frame. Zeros mean unmeasured.
    private static final double RED_HIVE_X = 58.0, RED_HIVE_Y = 71.5;
    private static final double BLUE_HIVE_X = 84.0, BLUE_HIVE_Y = 71.5;

    // Flywheel speed in encoder ticks per second. Placeholders until calibrated on the field.
    private static final double[] DIST_IN = {36, 60, 84, 108};
    private static final double[] SPEED_TPS = {1200, 1400, 1600, 1800};
    private static final double FALLBACK_SPEED_TPS = 1400;
    private static final double SPEED_TOLERANCE_TPS = 50;
    private static final int SPIN_UP_TIMEOUT_MS = 2000;
	
	// Flywheel speed pidf tuning
	private static final double P_COEF = 10.0;
    private static final double I_COEF = 0.0;
    private static final double D_COEF = 0.0;
    private static final double F_COEF = 12.0;

    // Indexer timing per ball
    private static final double FEED_POWER = 1.0;
    private static final int FEED_MS = 500;      // time for one ball to clear the indexer
    private static final int RECOVER_MS = 500;   // time for the flywheel to recover between balls

    // Aim tuning
    private static final double AIM_KP = 0.012;          // turn power per degree of tx
    private static final double AIM_LOCK_DEG = 1.5;      // must be inside this to lock
    private static final double AIM_RELEASE_DEG = 4.0;   // stays locked until outside this
    private static final double AIM_FILTER = 0.3;        // 0..1, lower = smoother
    private static final long AIM_LOST_HOLD_MS = 150;    // keep last angle this long after tag drops
    private static final double AIM_MAX_TURN = 0.4;

    private final DcMotorEx flywheel;
    private final CRServo indexer;
    private final VisionSubsystem vision;

    private int[] tags = VisionSubsystem.RED_HIVE_TAGS;
    private double hiveX = RED_HIVE_X;
    private double hiveY = RED_HIVE_Y;

    private boolean flywheelOn = false;
    private double targetSpeed = 0.0;

    private double txFiltered = 0.0;
    private boolean txValid = false;
    private long lastTagSeenMs = 0;
    private boolean aimLocked = false;

    private long spinStartMs = 0;
    private long alignStartMs = 0;
    private boolean turnIssued = false;
    private Command turnCommand = null;

    public ShooterIndexerSubsystemV3(HardwareMap hardwareMap, VisionSubsystem vision) {
        this.vision = vision;
        flywheel = hardwareMap.get(DcMotorEx.class, "Flywheel");
        indexer = hardwareMap.get(CRServo.class, "Indexer");
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // required for setVelocity
		 // Velocity PIDF. Starting values only; tune on the robot.
        // F is feedforward and should do most of the work. Check the coefficient scale for your SDK.
        flywheel.setVelocityPIDFCoefficients(P_COEF, I_COEF, D_COEF, F_COEF);
    }

    // ==========================================
    // ALLIANCE
    // ==========================================

    /** Sets the hive tags and pivot midpoint for the alliance. Call from init_loop, or from start in Auto. */
    public void setAlliance(boolean red) {
        tags = red ? VisionSubsystem.RED_HIVE_TAGS : VisionSubsystem.BLUE_HIVE_TAGS;
        hiveX = red ? RED_HIVE_X : BLUE_HIVE_X;
        hiveY = red ? RED_HIVE_Y : BLUE_HIVE_Y;
    }

    // ==========================================
    // DISTANCE
    // ==========================================

    /** Distance to the hive in inches: from the AprilTags if visible, otherwise from odometry. NaN if neither. */
    public double shotDistance(Pose pose) {
        double d = vision.distanceToTagsInches(tags);
        if (Double.isNaN(d) && hiveMeasured()) {
            d = Math.hypot(hiveX - pose.x(), hiveY - pose.y());
        }
        return d;
    }

    private boolean hiveMeasured() {
        return hiveX != 0.0 || hiveY != 0.0;
    }

    /** Flywheel speed in ticks per second for a distance in inches. */
    public static double speedForDistance(double d) {
        return interpolate(DIST_IN, SPEED_TPS, d);
    }

    private static double interpolate(double[] x, double[] y, double d) {
        if (d <= x[0]) return y[0];
        if (d >= x[x.length - 1]) return y[y.length - 1];
        for (int i = 0; i < x.length - 1; i++) {
            if (d <= x[i + 1]) {
                double t = (d - x[i]) / (x[i + 1] - x[i]);
                return y[i] + t * (y[i + 1] - y[i]);
            }
        }
        return y[y.length - 1];
    }

    // ==========================================
    // FLYWHEEL
    // ==========================================

    public void setFlywheelOn(boolean on) {
        flywheelOn = on;
    }

    public boolean isFlywheelOn() {
        return flywheelOn;
    }

    /** Call once per loop, before Scheduler.execute(). Sets the flywheel speed from the current distance. */
    public void update(Pose pose) {
        if (flywheelOn) {
            double d = shotDistance(pose);
            targetSpeed = Double.isNaN(d) ? FALLBACK_SPEED_TPS : speedForDistance(d);
        } else {
            targetSpeed = 0.0;
        }
        flywheel.setVelocity(targetSpeed);
    }

    public double getTargetSpeed() {
        return targetSpeed;
    }

    public double getActualSpeed() {
        return flywheel.getVelocity();
    }

    public boolean isAtSpeed() {
        return flywheelOn && Math.abs(getActualSpeed() - targetSpeed) <= SPEED_TOLERANCE_TPS;
    }

    // ==========================================
    // ALIGN
    // ==========================================

    /** Filtered horizontal angle to the alliance hive tags in degrees, or NaN if no tag was seen recently. */
    private double measureAimError() {
        long now = System.currentTimeMillis();
        double tx = vision.averageTx(tags);
        if (!Double.isNaN(tx)) {
            txFiltered = txValid ? txFiltered + AIM_FILTER * (tx - txFiltered) : tx;
            txValid = true;
            lastTagSeenMs = now;
        } else if (now - lastTagSeenMs > AIM_LOST_HOLD_MS) {
            txValid = false;
        }
        return txValid ? txFiltered : Double.NaN;
    }

    public void resetAim() {
        txValid = false;
        aimLocked = false;
    }

    /**
     * TeleOp aim. Returns the turn power to use with the drive.
     * When aimRequested is false, returns stickTurn unchanged.
     * When aiming, returns a proportional turn, or 0 once locked.
     * Falls back to stickTurn when no tag is available.
     */
    public double aimTurn(boolean aimRequested, double stickTurn) {
        if (!aimRequested) {
            resetAim();
            return stickTurn;
        }
        double err = measureAimError();
        if (Double.isNaN(err)) {
            aimLocked = false;
            return stickTurn;
        }
        double a = Math.abs(err);
        aimLocked = aimLocked ? (a < AIM_RELEASE_DEG) : (a < AIM_LOCK_DEG);
        if (aimLocked) return 0.0;
        // Positive tx = target right of center, so turn right (negative turn)
        return Math.max(-AIM_MAX_TURN, Math.min(AIM_MAX_TURN, -AIM_KP * err));
    }

    public boolean isAimed() {
        return aimLocked;
    }

    public boolean hasAimTarget() {
        return txValid;
    }

    public double getFilteredTx() {
        return txFiltered;
    }

    public double getRawTx() {
        return vision.averageTx(tags);
    }

    /**
     * Auto align. Turns the robot once toward the hive tags using Pedro hold.
     * Keeps looking for a tag until timeoutMs. Skips the turn if already inside the lock band.
     */
    public Command alignToTags(Follower follower, long timeoutMs) {
        return Command.build()
                .setStart(() -> {
                    alignStartMs = System.currentTimeMillis();
                    turnIssued = false;
                    turnCommand = null;
                    resetAim();
                })
                .setExecute(() -> {
                    if (turnIssued) return;
                    double err = measureAimError();
                    if (Double.isNaN(err)) return; // no tag yet, keep looking
                    turnIssued = true;
                    if (Math.abs(err) <= AIM_LOCK_DEG) return; // already aimed
                    Pose here = follower.pose();
                    // Positive tx = target right of center, so subtract to turn right
                    double h = normalizeAngle(here.heading() - Math.toRadians(err));
                    turnCommand = hold(follower, new Pose(here.x(), here.y(), h));
                    Scheduler.schedule(turnCommand);
                })
                .setDone(() -> (turnIssued && (turnCommand == null || !follower.isBusy()))
                        || System.currentTimeMillis() - alignStartMs > timeoutMs);
    }

    // ==========================================
    // SHOOT
    // ==========================================

    /** Waits until the flywheel is at speed, or the timeout passes. */
    public Command waitForShotSpeed() {
        return Command.build()
                .setStart(() -> spinStartMs = System.currentTimeMillis())
                .setDone(() -> isAtSpeed()
                        || System.currentTimeMillis() - spinStartMs > SPIN_UP_TIMEOUT_MS);
    }

    /** Waits for speed, feeds one ball, then recovers. */
    public Command shootBall() {
        return sequential(
                waitForShotSpeed(),
                feedStart(),
                waitMs(FEED_MS),
                feedStop(),
                waitMs(RECOVER_MS)
        );
    }

    /** Shoots the given number of balls in order, waiting for speed before each one. */
    public Command shootAll(int count) {
        if (count <= 0) return instant(() -> { });
        Command all = shootBall();
        for (int i = 1; i < count; i++) {
            all = sequential(all, shootBall());
        }
        return all;
    }

    public Command turnFlywheelOn() {
        return instant(() -> setFlywheelOn(true));
    }

    public Command turnFlywheelOff() {
        return instant(() -> setFlywheelOn(false));
    }

    private Command feedStart() {
        return Command.build()
                .setStart(() -> indexer.setPower(-FEED_POWER))
                .setDone(() -> true)
                .requiring(indexer);
    }

    private Command feedStop() {
        return Command.build()
                .setStart(() -> indexer.setPower(0.0))
                .setDone(() -> true)
                .requiring(indexer);
    }

    // ==========================================
    // HELPERS
    // ==========================================

    private static double normalizeAngle(double a) {
        return Math.atan2(Math.sin(a), Math.cos(a));
    }
}
