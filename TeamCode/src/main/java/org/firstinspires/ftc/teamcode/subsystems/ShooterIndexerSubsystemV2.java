package org.firstinspires.ftc.teamcode.subsystems;

import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.sequential;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ShooterIndexerSubsystemV2 {
    private final DcMotorEx flywheel;
    private final CRServo indexer;

    // Constants for your BioBuzz setup
    private static final double SHOOTING_POWER = 0.65;
    private static final double INDEXER_FEED_POWER = 1.0;
    private static final double INDEXER_STOP_POWER = 0.0;
    private static final double TICKS_PER_REV = 28.0;
    private static final double MIN_DIST = 24.0;
    private static final double MAX_DIST = 55.0;
    private static final double MIN_VEL = 1800.0;
    private static final double MAX_VEL = 2650.0;


    public ShooterIndexerSubsystemV2(HardwareMap hardwareMap) {
        flywheel = hardwareMap.get(DcMotorEx.class, "Flywheel");
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        indexer = hardwareMap.get(CRServo.class, "Indexer"); // Your CR Servo indexer
    }

    // Direct power control for the flywheel, used by teleop each loop
    public void setFlywheelVelocity(double vel) {
        flywheel.setVelocity(vel);
    }

    // Command to rev up the shooter flywheel
    public Command spinUp(double vel) {
        return Command.build()
                .setStart(() -> flywheel.setVelocity(vel))
                .setDone(() -> true) // Finishes instantly, letting the motor rev in the background
                .requiring(flywheel);
    }

    // Command to stop the flywheel
    public Command stopFlywheel() {
        return Command.build()
                .setStart(() -> flywheel.setVelocity(0.0))
                .setDone(() -> true)
                .requiring(flywheel);
    }


    // Command to feed pollen via the CR Servo Indexer
    public Command feedPollen() {
        return Command.build()
                .setStart(() -> indexer.setPower(-INDEXER_FEED_POWER))
                .setDone(() -> true)
                .requiring(indexer);
    }

    // Command to stop the CR Servo Indexer
    public Command stopIndexer() {
        return Command.build()
                .setStart(() -> indexer.setPower(INDEXER_STOP_POWER))
                .setDone(() -> true)
                .requiring(indexer);
    }

    public double calculateShooterVelocity(double distanceInches) {
        // Tune these two test points on the field: (Distance -> Velocity in ticks/sec)

        double clampedDist = Math.max(MIN_DIST, Math.min(MAX_DIST, distanceInches));
        return MIN_VEL + ((clampedDist - MIN_DIST) / (MAX_DIST - MIN_DIST)) * (MAX_VEL - MIN_VEL);
    }


    public Command shootMacro(double vel) {
        return sequential(
                spinUp(vel),
                waitMs(1000),       // Allow time for the flywheel to reach target velocity
                feedPollen(),
                waitMs(500),       // Time duration required to push item completely through
                stopIndexer()
        );
    }
}
