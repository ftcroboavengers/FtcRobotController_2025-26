package org.firstinspires.ftc.teamcode.auto;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "Forward + AutoShoot AnyTag", group = "RoboAvengers")
public class ForwardAuto_AutoShootAnyAlliance extends LinearOpMode {

    // ---------------- Drive Motors ----------------
    private DcMotor leftFront, rightFront, leftBack, rightBack;

    // ---------------- Intake + Launcher ----------------
    private DcMotor intake;
    private DcMotorEx leftLauncher, rightLauncher;

    // ---------------- Odometry ----------------
    private GoBildaPinpointDriver pinpoint;

    // ---------------- Motion Tuning ----------------
    private static final double MAX_DRIVE_POWER = 0.5;
    private static final double MAX_TURN_POWER  = 0.45;
    private static final double kDrive          = 0.025;

    // ---------------- AutoShoot RPM Buckets ----------------
    private static final double TPR = 28.0;  // ticks per revolution
    private static double rpmToTPS(double rpm) { return rpm * TPR / 60.0; }

    private static final double SHORT_RPM = 2900;
    private static final double MID_RPM   = 3000;
    private static final double LONG_RPM  = 3500;

    private static final double FIXED_F = 18.0;

    // Target TPS for ready-check
    private double launcherTargetTPS = rpmToTPS(MID_RPM);

    // ---------------- Tag IDs (Center) ----------------
    private static final int RED_TAG_ID  = 24;
    private static final int BLUE_TAG_ID = 20;

    // ---------------- AutoShoot State ----------------
    private enum AutoShootState { FIND, AIM, SPINUP, FEED, STOP }
    private AutoShootState autoShootState = AutoShootState.FIND;

    // Smoothing
    private double smoothedDistanceM = -1;
    private double smoothedYawDeg    = 0;
    private double lastRawDistM      = -1;
    private static final double MAX_DISTANCE_JUMP_M = 0.20; // 20 cm spike rejection

    private int aimStableCount = 0;
    private final ElapsedTime autoShootTimer = new ElapsedTime();
    private final ElapsedTime aimTimer       = new ElapsedTime();
    private final ElapsedTime stateTimer     = new ElapsedTime();

    // Vision
    private VisionPortal visionPortal;
    private AprilTagProcessor tagProcessor;

    // ---------------- Main Auto State Machine ----------------
    private enum AutoState { AUTOSHOOT, FORWARD_30, DONE }
    private AutoState state = AutoState.AUTOSHOOT;

    private final ElapsedTime autoStateTimer = new ElapsedTime();

    // Odometry helpers
    private double lastX = 0;
    private long wrongWayCountX = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        // ---------------- MOTOR MAP ----------------
        leftFront  = getMotor("front_left_drive");
        rightFront = getMotor("front_right_drive");
        leftBack   = getMotor("back_left_drive");
        rightBack  = getMotor("back_right_drive");

        intake        = getMotor("intake");
        leftLauncher  = getMotorEx("left_launcher");
        rightLauncher = getMotorEx("right_launcher");

        // Directions
        if (leftFront  != null) leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        if (leftBack   != null) leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        if (rightFront != null) rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        if (rightBack  != null) rightBack.setDirection(DcMotorSimple.Direction.FORWARD);

        if (intake != null) intake.setDirection(DcMotorSimple.Direction.REVERSE);

        setBrake(leftFront, rightFront, leftBack, rightBack);

        // Launchers
        initLauncher(leftLauncher, true);
        initLauncher(rightLauncher, false);

        // ---------------- Pinpoint Init ----------------
        initPinpoint();

        // ---------------- Vision Init ----------------
        initAprilTags();

        telemetry.addLine("READY: Forward + AutoShoot AnyTag");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        autoStateTimer.reset();
        resetAutoShootFSM();

        // ---------------- MAIN AUTO LOOP ----------------
        while (opModeIsActive() && state != AutoState.DONE) {

            if (pinpoint != null) {
                pinpoint.update();
            }

            switch (state) {

                // ----------------------------------------------------
                // 1) AutoShoot at whichever center tag we see first
                // ----------------------------------------------------
                case AUTOSHOOT:
                    if (runAutoShootStep()) {
                        // AutoShoot finished (either shot or bailed)
                        stopLaunch();
                        stopDrive();
                        state = AutoState.FORWARD_30;
                        autoStateTimer.reset();
                    }
                    break;

                // ----------------------------------------------------
                // 2) Drive forward 30 inches (X increasing)
                // ----------------------------------------------------
                case FORWARD_30:
                    if (moveToX(30.0) || autoStateTimer.seconds() > 4.0) {
                        stopDrive();
                        state = AutoState.DONE;
                    }
                    break;

                default:
                    state = AutoState.DONE;
                    break;
            }

            // Telemetry
            telemetry.addData("AutoState", state);
            telemetry.addData("AutoShootState", autoShootState);

            if (pinpoint != null) {
                telemetry.addData("X (in)", "%.2f", pinpoint.getPosX(DistanceUnit.INCH));
                telemetry.addData("Y (in)", "%.2f", pinpoint.getPosY(DistanceUnit.INCH));
            }

            telemetry.addData("SmoothedYaw", "%.2f", smoothedYawDeg);
            telemetry.addData("SmoothedDist(m)", "%.3f", smoothedDistanceM);
            telemetry.addData("AimStable", aimStableCount);
            telemetry.update();
        }

        stopLaunch();
        stopDrive();
    }

    // =========================================================================
    // AUTO SHOOT SYSTEM  (Red OR Blue tag)
    // =========================================================================

    private void resetAutoShootFSM() {
        autoShootState    = AutoShootState.FIND;
        smoothedDistanceM = -1;
        smoothedYawDeg    = 0;
        lastRawDistM      = -1;
        aimStableCount    = 0;

        autoShootTimer.reset();
        aimTimer.reset();
        stateTimer.reset();
    }

    /**
     * Single-step the embedded AutoShoot FSM.
     * Returns true when AutoShoot sequence is finished.
     */
    private boolean runAutoShootStep() {

        double now = autoShootTimer.seconds();
        AprilTagDetection best = getBestCenterTag(); // Red (24) OR Blue (20)

        switch (autoShootState) {

            // --------------------------------------------------------
            // FIND: Wait for a center tag to appear
            // --------------------------------------------------------
            case FIND:
                if (best != null && best.ftcPose != null) {
                    resetSmoothing(best);
                    autoShootState = AutoShootState.AIM;
                    aimTimer.reset();
                    stateTimer.reset();
                } else if (now > 2.0) {
                    // Fail-safe: no tag → skip shooting and continue auto
                    return true;
                }
                break;

            // --------------------------------------------------------
            // AIM: Rotate using yaw to center tag
            // --------------------------------------------------------
            case AIM:
                if (best == null || best.ftcPose == null) {
                    // Lost tag: stop turning but keep trying for a bit
                    drive(0, 0, 0);
                    if (stateTimer.seconds() > 2.0) {
                        // Give up after 2 seconds without tag
                        return true;
                    }
                    break;
                }

                // Update smoothing
                if (!updateSmoothingFromTag(best)) {
                    // Spike ignored; don't move robot this cycle
                    drive(0, 0, 0);
                    break;
                }

                // Turn only (no strafe, no forward)
                double turn = aimTurnPower(smoothedYawDeg);
                drive(0, 0, turn);

                // Stability logic
                if (Math.abs(smoothedYawDeg) < 3.0) {
                    aimStableCount++;
                } else {
                    aimStableCount = 0;
                }

                // Soft timeout: if we've been aiming for > 2.0s and are "close", force go
                if (aimTimer.seconds() > 2.0 && Math.abs(smoothedYawDeg) < 8.0) {
                    aimStableCount = 2;
                }

                // When stable, pick RPM & spin up
                if (aimStableCount >= 2) {
                    pickRPM(smoothedDistanceM);
                    startLaunch();
                    autoShootState = AutoShootState.SPINUP;
                    stateTimer.reset();
                }

                break;

            // --------------------------------------------------------
            // SPINUP: wait until launchers reach target speed
            // --------------------------------------------------------
            case SPINUP:
                boolean ready = readyToShoot();
                if (ready && stateTimer.seconds() > 0.35) {
                    if (intake != null) intake.setPower(1.0);
                    autoShootState = AutoShootState.FEED;
                    stateTimer.reset();
                }

                // Hard timeout: shoot anyway after 2.5s
                if (stateTimer.seconds() > 2.5) {
                    if (intake != null) intake.setPower(1.0);
                    autoShootState = AutoShootState.FEED;
                    stateTimer.reset();
                }
                break;

            // --------------------------------------------------------
            // FEED: run intake to push note through
            // --------------------------------------------------------
            case FEED:
                if (stateTimer.seconds() > 1.0) {
                    if (intake != null) intake.setPower(0);
                    autoShootState = AutoShootState.STOP;
                    stateTimer.reset();
                }
                break;

            // --------------------------------------------------------
            // STOP: stop launchers and report done
            // --------------------------------------------------------
            case STOP:
                stopLaunch();
                return true;
        }

        return false;
    }

    // ---------------- Tag Selection + Smoothing ----------------

    /**
     * Look at all detections; choose whichever one is
     * Red center (24) or Blue center (20) with smallest |bearing|.
     */
    private AprilTagDetection getBestCenterTag() {
        if (tagProcessor == null) return null;

        List<AprilTagDetection> dets = new ArrayList<>(tagProcessor.getDetections());
        AprilTagDetection best = null;
        double bestAbsBearing = Double.MAX_VALUE;

        for (AprilTagDetection t : dets) {
            if (t.ftcPose == null) continue;
            if (t.id != RED_TAG_ID && t.id != BLUE_TAG_ID) continue;

            double absBearing = Math.abs(t.ftcPose.bearing);
            if (absBearing < bestAbsBearing) {
                bestAbsBearing = absBearing;
                best = t;
            }
        }
        return best;
    }

    private void resetSmoothing(AprilTagDetection tag) {
        double distM = tag.ftcPose.range * 0.0254;  // inches -> meters
        double yaw   = tag.ftcPose.yaw;             // degrees

        smoothedDistanceM = distM;
        smoothedYawDeg    = yaw;
        lastRawDistM      = distM;
    }

    /**
     * Update smoothed distance & yaw.
     * Returns false if the reading is rejected as a spike (no movement this cycle).
     */
    private boolean updateSmoothingFromTag(AprilTagDetection tag) {
        double rawDistM = tag.ftcPose.range * 0.0254;
        double rawYaw   = tag.ftcPose.yaw;

        // Reject huge jumps in distance
        if (lastRawDistM > 0 && Math.abs(rawDistM - lastRawDistM) > MAX_DISTANCE_JUMP_M) {
            return false;
        }
        lastRawDistM = rawDistM;

        if (smoothedDistanceM < 0) {
            smoothedDistanceM = rawDistM;
        } else {
            smoothedDistanceM = 0.7 * smoothedDistanceM + 0.3 * rawDistM;
        }

        smoothedYawDeg = 0.7 * smoothedYawDeg + 0.3 * rawYaw;

        return true;
    }

    private double aimTurnPower(double yawDeg) {
        double kP  = 0.03;          // proportional gain
        double turn = -kP * yawDeg; // positive yaw -> turn opposite direction

        double min = 0.12;  // minimum power to overcome friction
        double max = 0.40;  // cap

        // If we're far off, enforce a minimum turn power
        if (Math.abs(turn) < min && Math.abs(yawDeg) > 2.0) {
            turn = Math.signum(turn) * min;
        }

        // As we get closer to center, soften the turn
        if (Math.abs(yawDeg) < 8.0) {
            turn *= 0.6;
        }

        // Clamp
        if (Math.abs(turn) > max) {
            turn = Math.signum(turn) * max;
        }

        return turn;
    }

    // ---------------- RPM Bucketing ----------------

    private void pickRPM(double distM) {
        // short: < 3 ft (0.9144 m)
        // mid:   3–9 ft (0.9144–2.7432 m)
        // long:  > 9 ft (2.7432+ m)
        if (distM < 0.9144) {
            setRPM(SHORT_RPM);
        } else if (distM < 2.7432) {
            setRPM(MID_RPM);
        } else {
            setRPM(LONG_RPM);
        }
    }

    private void setRPM(double rpm) {
        launcherTargetTPS = rpmToTPS(rpm);
        double tps = launcherTargetTPS;

        if (leftLauncher != null) {
            leftLauncher.setVelocity(tps);
        }
        if (rightLauncher != null) {
            rightLauncher.setVelocity(tps);
        }
    }

    private boolean readyToShoot() {
        if (leftLauncher == null || rightLauncher == null) return true;

        double lv = leftLauncher.getVelocity();
        double rv = rightLauncher.getVelocity();

        return Math.abs(lv - launcherTargetTPS) < 200 &&
                Math.abs(rv - launcherTargetTPS) < 200;
    }

    private void startLaunch() {
        // Re-assert velocity in case battery sagged
        setRPM(launcherTargetTPS * 60.0 / TPR);
    }

    private void stopLaunch() {
        if (leftLauncher != null)  leftLauncher.setPower(0);
        if (rightLauncher != null) rightLauncher.setPower(0);
    }

    // =========================================================================
    // MOVEMENT HELPERS (X movement only + mecanum drive)
    // =========================================================================

    private double getX() {
        if (pinpoint == null) return 0;
        return pinpoint.getPosX(DistanceUnit.INCH);
    }

    private boolean moveToX(double targetXInches) {
        if (pinpoint == null) {
            // If no odometry, just drive forward a bit as a fallback
            drive(0.3, 0, 0);
            if (autoStateTimer.seconds() > 2.0) {
                drive(0, 0, 0);
                return true;
            }
            return false;
        }

        double currentX = getX();
        double error = targetXInches - currentX;
        double power = kDrive * error;

        power = Math.max(-MAX_DRIVE_POWER, Math.min(MAX_DRIVE_POWER, power));

        // Forward/back only
        drive(power, 0, 0);

        double dx = currentX - lastX;
        if (Math.abs(power) > 0.05 && Math.abs(dx) > 0.01) {
            boolean wrong = Math.signum(power) != Math.signum(dx);
            wrongWayCountX = wrong ? wrongWayCountX + 1 : 0;
            if (wrongWayCountX > 15) {
                // Odometry looks inverted; bail out
                return true;
            }
        } else {
            wrongWayCountX = 0;
        }

        lastX = currentX;
        return Math.abs(error) < 2.0;
    }

    // Basic mecanum drive wrapper: fwd/strafe/turn in [-1, 1]
    private void drive(double fwd, double strafe, double turn) {
        double fl = fwd + strafe + turn;
        double bl = fwd - strafe + turn;
        double fr = fwd - strafe - turn;
        double br = fwd + strafe - turn;

        // Normalize if needed
        double max = Math.max(Math.max(Math.abs(fl), Math.abs(fr)),
                Math.max(Math.abs(bl), Math.abs(br)));
        if (max > 1.0) {
            fl /= max;
            fr /= max;
            bl /= max;
            br /= max;
        }

        fl *= MAX_DRIVE_POWER;
        fr *= MAX_DRIVE_POWER;
        bl *= MAX_DRIVE_POWER;
        br *= MAX_DRIVE_POWER;

        setPower(fl, fr, bl, br);
    }

    private void setPower(double fl, double fr, double bl, double br) {
        if (leftFront  != null) leftFront.setPower(fl);
        if (rightFront != null) rightFront.setPower(fr);
        if (leftBack   != null) leftBack.setPower(bl);
        if (rightBack  != null) rightBack.setPower(br);
    }

    private void stopDrive() {
        setPower(0, 0, 0, 0);
    }

    // =========================================================================
    // INIT HELPERS
    // =========================================================================

    private void initLauncher(DcMotorEx m, boolean reverse) {
        if (m == null) return;
        m.setDirection(reverse ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        m.setZeroPowerBehavior(BRAKE);
        m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m.setVelocityPIDFCoefficients(25, 0, 5, FIXED_F);
    }

    private void initPinpoint() {
        try {
            pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
            pinpoint.setEncoderResolution(
                    GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
            pinpoint.setEncoderDirections(
                    GoBildaPinpointDriver.EncoderDirection.FORWARD,
                    GoBildaPinpointDriver.EncoderDirection.FORWARD);
            pinpoint.setOffsets(0, 0, DistanceUnit.MM);
            pinpoint.resetPosAndIMU();

            // Let it settle a bit
            for (int i = 0; i < 25; i++) {
                pinpoint.update();
                sleep(20);
            }
            pinpoint.resetPosAndIMU();

        } catch (Exception e) {
            telemetry.addLine("Pinpoint NOT found; running without odometry.");
            telemetry.update();
            pinpoint = null;
        }
    }

    private void initAprilTags() {
        tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawTagID(true)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(tagProcessor)
                .build();
    }

    // =========================================================================
    // MISC HELPERS
    // =========================================================================

    private void setBrake(DcMotor... motors) {
        for (DcMotor m : motors) {
            if (m != null) m.setZeroPowerBehavior(BRAKE);
        }
    }

    private DcMotor getMotor(String name) {
        try { return hardwareMap.get(DcMotor.class, name); }
        catch (Exception e) { return null; }
    }

    private DcMotorEx getMotorEx(String name) {
        try { return hardwareMap.get(DcMotorEx.class, name); }
        catch (Exception e) { return null; }
    }
}
