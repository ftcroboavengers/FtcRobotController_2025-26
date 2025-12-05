package org.firstinspires.ftc.teamcode.auto;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import java.util.ArrayList;
import java.util.List;
@Disabled
@Autonomous(name = "Back + AutoShoot Red", group = "RoboAvengers")
public class BackFireRedDec6 extends LinearOpMode {

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

    private static final double kDrive  = 0.025;
    private static final double kTurn   = 0.012;
    private static final double TURN_MIN_POWER = 0.20;

    // ---------------- AutoShoot RPM Buckets ----------------
    private static final double TPR = 28.0;  // ticks per revolution
    private static double rpmToTPS(double rpm){ return rpm * TPR / 60.0; }

    private static final double SHORT_RPM = 2900;
    private static final double MID_RPM   = 3000;
    private static final double LONG_RPM  = 3500;

    private static final double FIXED_F = 18.0;

    // Track current target TPS for readyToShoot()
    private double currentTargetTPS = rpmToTPS(MID_RPM);

    // ---------------- Target Tag (RED) ----------------
    private static final int RED_TAG_ID = 24;

    // ---------------- AutoShoot State ----------------
    private enum AutoShootState { FIND, AIM, SPINUP, FEED, STOP }
    private AutoShootState autoShootState;

    // Smoothing
    private double smoothedDistance = -1;
    private double smoothedBearing  = 0;
    private double lastRawDistance  = -1;
    private static final double MAX_DISTANCE_JUMP = 0.20; // ignore spikes (m)

    // Stability checks
    private int aimStableCount = 0;

    // Timers
    private double stateStartTime = 0;
    private double shootStartTime = 0;

    // Vision
    private VisionPortal visionPortal;
    private AprilTagProcessor tagProcessor;

    // ---------------- Autonomous State Machine ----------------
    private enum AutoState {
        BACK_58,
        AUTOSHOOT_1,
        TURN_RIGHT,
        FORWARD_INTAKE,
        RETURN_BACK,
        AIM_FINAL,
        AUTOSHOOT_2,
        STRAFE_RIGHT_20,
        DONE
    }

    private AutoState state = AutoState.BACK_58;
    private AutoState lastState = null;

    // Odometry movement helpers
    private double lastX = 0, lastY = 0;
    private long wrongWayCountX = 0, wrongWayCountY = 0;

    // Target for final strafe
    private double strafeTargetY = 0.0;

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

        if (leftFront != null)  leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        if (leftBack != null)   leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        if (rightFront != null) rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        if (rightBack != null)  rightBack.setDirection(DcMotorSimple.Direction.FORWARD);

        if (intake != null) intake.setDirection(DcMotorSimple.Direction.REVERSE);

        setBrake(leftFront, rightFront, leftBack, rightBack);

        // Setup launchers
        initLauncher(leftLauncher, true);
        initLauncher(rightLauncher, false);

        // ---------------- Pinpoint Init ----------------
        initPinpoint();

        // ---------------- Vision Init (RED TAG 24) ----------------
        initAprilTags();

        telemetry.addLine("Ready for Auto");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        resetAutoShoot();

        // ---------------- MAIN AUTO LOOP ----------------
        while (opModeIsActive() && state != AutoState.DONE) {

            if (pinpoint != null) pinpoint.update();

            if (state != lastState) {
                stateStartTime = getRuntime();
                lastState = state;
            }

            switch (state) {

                // ----------------------------------------------------
                // BACK 58 inches
                // ----------------------------------------------------
                case BACK_58:
                    if (moveToX(-58) || timeInState() > 6.0) {
                        stopDrive();
                        sleep(250);
                        state = AutoState.AUTOSHOOT_1;
                    }
                    break;

                // ----------------------------------------------------
                // FIRST AUTO SHOOT USING APRILTAG
                // ----------------------------------------------------
                case AUTOSHOOT_1:
                    if (runAutoShoot()) {
                        stopLaunch();
                        state = AutoState.TURN_RIGHT;
                    }
                    break;

                // ----------------------------------------------------
                // TURN RIGHT -45°
                // ----------------------------------------------------
                case TURN_RIGHT:
                    if (turnToHeading(-45) || timeInState() > 3.0) {
                        stopDrive();
                        sleep(200);
                        state = AutoState.FORWARD_INTAKE;
                    }
                    break;

                // ----------------------------------------------------
                // FORWARD INTAKE to X = -28
                // ----------------------------------------------------
                case FORWARD_INTAKE:
                    if (intake != null) intake.setPower(1.0);
                    if (moveToX(-28) || timeInState() > 3.0) {
                        if (intake != null) intake.setPower(0);
                        stopDrive();
                        sleep(200);
                        state = AutoState.RETURN_BACK;
                    }
                    break;

                // ----------------------------------------------------
                // RETURN to X = -58
                // ----------------------------------------------------
                case RETURN_BACK:
                    if (moveToX(-58) || timeInState() > 4.0) {
                        stopDrive();
                        sleep(200);
                        state = AutoState.AIM_FINAL;
                    }
                    break;

                // ----------------------------------------------------
                // TURN BACK TO FACE GOAL (~0°), THEN AUTOSHOOT_2
                // ----------------------------------------------------
                case AIM_FINAL:
                    // Use IMU + pinpoint to rotate back from -45° to 0°
                    if (turnToHeading(0.0) || timeInState() > 3.0) {
                        stopDrive();
                        sleep(200);
                        resetAutoShoot();          // prepare internal autoshoot FSM
                        // NEW STEP: Try to recover the tag if not visible
                        recoverTagIfMissing();
                        state = AutoState.AUTOSHOOT_2;
                    }
                    break;

                // ----------------------------------------------------
                // SECOND AUTO SHOOT (same autoshoot system)
                // ----------------------------------------------------
                case AUTOSHOOT_2:
                    if (runAutoShoot()) {
                        stopLaunch();
                        // Lock in strafe target Y = current Y + 20" to the right
                        strafeTargetY = getY() + 20.0;
                        state = AutoState.STRAFE_RIGHT_20;
                    }
                    break;

                // ----------------------------------------------------
                // STRAFE RIGHT +20 INCHES (Odometry Y)
                // ----------------------------------------------------
                case STRAFE_RIGHT_20:
                    if (moveToY(strafeTargetY) || timeInState() > 3.0) {
                        stopDrive();
                        state = AutoState.DONE;
                    }
                    break;

                default:
                    state = AutoState.DONE;
            }

            telemetry.addData("State", state);
            telemetry.addData("X", getX());
            telemetry.addData("Y", getY());
            telemetry.update();
        }

        stopDrive();
        stopLaunch();
    }

    // =========================================================================
    //                        AUTO SHOOT SYSTEM
    // =========================================================================

    private void resetAutoShoot() {
        autoShootState = AutoShootState.FIND;
        aimStableCount = 0;
        smoothedDistance = -1;
        smoothedBearing = 0;
        lastRawDistance = -1;
        shootStartTime = getRuntime();
    }

    /**
     * Runs embedded AutoShoot state machine.
     * Returns TRUE when shooting is completed.
     */
    private boolean runAutoShoot() {

        double now = getRuntime();

        switch (autoShootState) {

            // ----------------------------------------------------
            // FIND TAG
            // ----------------------------------------------------
            case FIND: {
                AprilTagDetection tag = getRedTag();
                if (tag != null && tag.ftcPose != null) {
                    autoShootState = AutoShootState.AIM;
                    shootStartTime = now;
                } else if (now - shootStartTime > 2.0) {
                    // Fail-safe: skip if no tag
                    return true;
                }
                break;
            }

            // ----------------------------------------------------
            // AIM AT TAG
            // ----------------------------------------------------
            case AIM:
                if (!updateTagSmoothing()) {
                    // Tag lost: try FIND again (or timeout will bail)
                    break;
                }

                double turnPower = aimTurnPower(smoothedBearing);
                drive(0, 0, turnPower);

                // Stability checks
                if (Math.abs(smoothedBearing) < 4.0) aimStableCount++;
                else aimStableCount = 0;

                // If stable, pick RPM + spin up
                if (aimStableCount >= 2) {
                    pickRPM(smoothedDistance);
                    startLaunch();
                    autoShootState = AutoShootState.SPINUP;
                    shootStartTime = now;
                }

                // Timeout fallback: just shoot with whatever we have
                if (now - shootStartTime > 2.5) {
                    pickRPM(smoothedDistance);
                    startLaunch();
                    autoShootState = AutoShootState.SPINUP;
                    shootStartTime = now;
                }
                break;

            // ----------------------------------------------------
            // WAIT FOR RPM
            // ----------------------------------------------------
            case SPINUP:
                if (readyToShoot() && now - shootStartTime > 0.35) {
                    if (intake != null) intake.setPower(1.0);
                    autoShootState = AutoShootState.FEED;
                    shootStartTime = now;
                }
                if (now - shootStartTime > 2.5) {
                    autoShootState = AutoShootState.STOP;
                }
                break;

            // ----------------------------------------------------
            // FEED NOTE
            // ----------------------------------------------------
            case FEED:
                if (now - shootStartTime > 1.0) {
                    if (intake != null) intake.setPower(0);
                    autoShootState = AutoShootState.STOP;
                    shootStartTime = now;
                }
                break;

            // ----------------------------------------------------
            // STOP LAUNCH
            // ----------------------------------------------------
            case STOP:
                stopLaunch();
                return true;
        }

        return false;
    }

    // ---------- AIMING + SMOOTHING ----------
    private boolean updateTagSmoothing() {
        AprilTagDetection tag = getRedTag();
        if (tag == null || tag.ftcPose == null) return false;

        // FTC ftcPose.range is in INCHES; convert to meters
        double rawDist = tag.ftcPose.range * 0.0254;
        double rawBear = tag.ftcPose.bearing;

        // Reject large spikes
        if (lastRawDistance > 0 && Math.abs(rawDist - lastRawDistance) > MAX_DISTANCE_JUMP) {
            return false;
        }
        lastRawDistance = rawDist;

        if (smoothedDistance < 0)
            smoothedDistance = rawDist;
        else
            smoothedDistance = 0.7 * smoothedDistance + 0.3 * rawDist;

        smoothedBearing = 0.7 * smoothedBearing + 0.3 * rawBear;

        return true;
    }

    private void pickRPM(double distM) {
        // short: under 3 ft (0.9144 m)
        // mid:   3 ft to 9 ft (0.9144 to 2.7432 m)
        // long:  9+ ft (2.7432+ m)
        if (distM < 0.9144) {
            setRPM(SHORT_RPM);
        } else if (distM < 2.7432) {
            setRPM(MID_RPM);
        } else {
            setRPM(LONG_RPM);
        }
    }

    private void setRPM(double rpm) {
        double tps = rpmToTPS(rpm);
        currentTargetTPS = tps;

        if (leftLauncher != null)  leftLauncher.setVelocity(tps);
        if (rightLauncher != null) rightLauncher.setVelocity(tps);
    }

    private boolean readyToShoot() {
        if (leftLauncher == null || rightLauncher == null) return true;

        double lv = leftLauncher.getVelocity();
        double rv = rightLauncher.getVelocity();
        double target = currentTargetTPS;

        return Math.abs(lv - target) < 200 && Math.abs(rv - target) < 200;
    }

    private void startLaunch() {
        // Run launchers at the velocity corresponding to currentTargetTPS
        if (leftLauncher != null)  leftLauncher.setVelocity(currentTargetTPS);
        if (rightLauncher != null) rightLauncher.setVelocity(currentTargetTPS);
    }

    private void stopLaunch() {
        if (leftLauncher != null)  leftLauncher.setPower(0);
        if (rightLauncher != null) rightLauncher.setPower(0);
    }

    private double aimTurnPower(double bearingDeg) {
        double kP = 0.035;
        double turn = -kP * bearingDeg;

        double min = 0.14;
        double max = 0.45;

        if (Math.abs(turn) < min && Math.abs(bearingDeg) > 3)
            turn = Math.signum(turn) * min;

        if (Math.abs(bearingDeg) < 10)
            turn *= 0.6;

        return Math.max(-max, Math.min(max, turn));
    }

    private AprilTagDetection getRedTag() {
        if (tagProcessor == null) return null;
        List<AprilTagDetection> list = new ArrayList<>(tagProcessor.getDetections());
        for (AprilTagDetection tag : list) {
            if (tag.id == RED_TAG_ID && tag.ftcPose != null) {
                return tag;
            }
        }
        return null;
    }

    // =========================================================================
    // MOVEMENT HELPERS (X, Y, TURN)
    // =========================================================================

    private double getX() { return pinpoint != null ? pinpoint.getPosX(DistanceUnit.INCH) : 0.0; }
    private double getY() { return pinpoint != null ? pinpoint.getPosY(DistanceUnit.INCH) : 0.0; }

    private boolean moveToX(double targetX) {
        if (pinpoint == null) return true;

        double current = getX();
        double error = targetX - current;
        double power = kDrive * error;

        power = Math.max(-MAX_DRIVE_POWER, Math.min(MAX_DRIVE_POWER, power));

        drive(power, 0, 0);

        double dx = current - lastX;
        if (Math.abs(power) > 0.05 && Math.abs(dx) > 0.01) {
            boolean wrong = Math.signum(power) != Math.signum(dx);
            wrongWayCountX = wrong ? wrongWayCountX + 1 : 0;
            if (wrongWayCountX > 15) return true;
        }
        lastX = current;

        return Math.abs(error) < 2.0;
    }

    private boolean moveToY(double targetY) {
        if (pinpoint == null) return true;

        double current = getY();
        double error = targetY - current;
        double power = 0.025 * error;

        power = Math.max(-0.45, Math.min(0.45, power));

        // Strafe-only mecanum power
        double fl = +power;
        double bl = -power;
        double fr = -power;
        double br = +power;
        setPower(fl, fr, bl, br);

        double dy = current - lastY;
        if (Math.abs(power) > 0.05 && Math.abs(dy) > 0.01) {
            boolean wrong = Math.signum(power) != Math.signum(dy);
            wrongWayCountY = wrong ? wrongWayCountY + 1 : 0;
            if (wrongWayCountY > 15) return true;
        }
        lastY = current;

        return Math.abs(error) < 2.0;
    }

    private boolean turnToHeading(double targetDeg) {
        if (pinpoint == null) return true;

        double current = normalize(pinpoint.getHeading(AngleUnit.DEGREES));
        double error = normalize(targetDeg - current);

        double power = kTurn * error;

        if (Math.abs(power) < TURN_MIN_POWER && Math.abs(error) > 2)
            power = Math.copySign(TURN_MIN_POWER, power);

        power = Math.max(-MAX_TURN_POWER, Math.min(MAX_TURN_POWER, power));

        // tank-style turn
        setPower(-power, power, -power, power);

        return Math.abs(error) < 5;
    }

    private double normalize(double a) {
        while (a > 180) a -= 360;
        while (a <= -180) a += 360;
        return a;
    }

    /**
     * Attempts to find the RED AprilTag (ID 24) by performing a small right-left wiggle.
     * Returns TRUE when the tag becomes visible.
     */
    private boolean recoverTagIfMissing() {

        AprilTagDetection tag = getRedTag();
        if (tag != null) return true;

        double wigglePower = 0.22;   // slightly stronger
        int wiggleSteps = 40;        // 0.8s each direction

        // ---- PRE-PAUSE: allow camera to settle after turn ----
        stopDrive();
        sleep(120);

        // =====================================================
        // 1) WIGGLE RIGHT FIRST (RED ALLIANCE)
        // =====================================================
        for (int i = 0; i < wiggleSteps; i++) {

            // Rotate RIGHT
            setPower(-wigglePower, +wigglePower, -wigglePower, +wigglePower);
            sleep(20);

            // brief camera refresh
            stopDrive();
            sleep(40);

            if ((tag = getRedTag()) != null) {
                stopDrive();
                return true;
            }
        }

        // =====================================================
        // 2) WIGGLE LEFT SECOND
        // =====================================================
        for (int i = 0; i < wiggleSteps; i++) {

            // Rotate LEFT
            setPower(+wigglePower, -wigglePower, +wigglePower, -wigglePower);
            sleep(20);

            // brief camera refresh
            stopDrive();
            sleep(40);

            if ((tag = getRedTag()) != null) {
                stopDrive();
                return true;
            }
        }

        stopDrive();
        return getRedTag() != null;
    }

    // =========================================================================
    // BASIC ROBOT HELPERS
    // =========================================================================

    private double timeInState() { return getRuntime() - stateStartTime; }

    private void drive(double fwd, double strafe, double turn) {
        double fl = fwd + strafe + turn;
        double bl = fwd - strafe + turn;
        double fr = fwd - strafe - turn;
        double br = fwd + strafe - turn;
        setPower(fl, fr, bl, br);
    }

    private void setPower(double fl, double fr, double bl, double br) {
        if (leftFront != null) leftFront.setPower(fl);
        if (rightFront != null) rightFront.setPower(fr);
        if (leftBack != null) leftBack.setPower(bl);
        if (rightBack != null) rightBack.setPower(br);
    }

    private void stopDrive() {
        setPower(0, 0, 0, 0);
    }

    private void setBrake(DcMotor... motors) {
        for (DcMotor m : motors) {
            if (m != null) m.setZeroPowerBehavior(BRAKE);
        }
    }

    private void initLauncher(DcMotorEx m, boolean reverse) {
        if (m == null) return;
        m.setDirection(reverse ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        m.setZeroPowerBehavior(BRAKE);
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
            for (int i=0; i<25; i++) {
                pinpoint.update();
                sleep(20);
            }
            pinpoint.resetPosAndIMU();
        } catch (Exception e) {
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
    // SAFE HARDWARE GETTERS
    // =========================================================================

    private DcMotor getMotor(String name) {
        try { return hardwareMap.get(DcMotor.class, name); }
        catch (Exception e) { return null; }
    }

    private DcMotorEx getMotorEx(String name) {
        try { return hardwareMap.get(DcMotorEx.class, name); }
        catch (Exception e) { return null; }
    }
}
