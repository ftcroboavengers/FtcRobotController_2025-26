package org.firstinspires.ftc.teamcode.auto;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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

@Autonomous(name = "Back + AutoShoot Blue", group = "RoboAvengers")
public class BackFireBlueDec6 extends LinearOpMode {

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
    private static final double TPR = 28.0;
    private static double rpmToTPS(double rpm){ return rpm * TPR / 60.0; }

    private static final double SHORT_RPM = 2900;
    private static final double MID_RPM   = 3000;
    private static final double LONG_RPM  = 3500;

    private static final double FIXED_F = 18.0;

    private double currentTargetTPS = rpmToTPS(MID_RPM);

    // ---------------- Target Tag (BLUE) ----------------
    private static final int BLUE_TAG_ID = 20;

    // ---------------- AutoShoot State ----------------
    private enum AutoShootState { FIND, AIM, SPINUP, FEED, STOP }
    private AutoShootState autoShootState;

    // Smoothing
    private double smoothedDistance = -1;
    private double smoothedBearing  = 0;
    private double lastRawDistance  = -1;
    private static final double MAX_DISTANCE_JUMP = 0.20;

    // Stability checks
    private int aimStableCount = 0;

    // Timers
    private double stateStartTime = 0;
    private double shootStartTime = 0;

    // Vision
    private VisionPortal visionPortal;
    private AprilTagProcessor tagProcessor;

    // ---------------- Autonomous States ----------------
    private enum AutoState {
        BACK_58,
        AUTOSHOOT_1,
        TURN_LEFT,
        FORWARD_INTAKE,
        RETURN_BACK,
        AIM_FINAL,
        AUTOSHOOT_2,
        STRAFE_RIGHT_20,
        DONE
    }

    private AutoState state = AutoState.BACK_58;
    private AutoState lastState = null;

    // Odometry helpers
    private double lastX = 0, lastY = 0;
    private long wrongWayCountX = 0, wrongWayCountY = 0;

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

        initLauncher(leftLauncher, true);
        initLauncher(rightLauncher, false);

        initPinpoint();
        initAprilTags();

        telemetry.addLine("Ready for Blue Auto");
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

                case BACK_58:
                    if (moveToX(-58) || timeInState() > 6.0) {
                        stopDrive();
                        sleep(250);
                        state = AutoState.AUTOSHOOT_1;
                    }
                    break;

                case AUTOSHOOT_1:
                    if (runAutoShoot()) {
                        stopLaunch();
                        state = AutoState.TURN_LEFT;
                    }
                    break;

                // BLUE TURN IS +45°, not -45°
                case TURN_LEFT:
                    if (turnToHeading(+45) || timeInState() > 3.0) {
                        stopDrive();
                        sleep(200);
                        state = AutoState.FORWARD_INTAKE;
                    }
                    break;

                case FORWARD_INTAKE:
                    if (intake != null) intake.setPower(1.0);
                    if (moveToX(-28) || timeInState() > 3.0) {
                        if (intake != null) intake.setPower(0);
                        stopDrive();
                        sleep(200);
                        state = AutoState.RETURN_BACK;
                    }
                    break;

                case RETURN_BACK:
                    if (moveToX(-58) || timeInState() > 4.0) {
                        stopDrive();
                        sleep(200);
                        state = AutoState.AIM_FINAL;
                    }
                    break;

                // turn back to 0°
                case AIM_FINAL:
                    if (turnToHeading(0) || timeInState() > 3.0) {
                        stopDrive();
                        sleep(200);
                        resetAutoShoot();
                        // NEW STEP: Try to recover the tag if not visible
                        recoverTagIfMissing();

                        state = AutoState.AUTOSHOOT_2;
                    }
                    break;

                case AUTOSHOOT_2:
                    if (runAutoShoot()) {
                        stopLaunch();
                        strafeTargetY = getY() - 20.0;
                        state = AutoState.STRAFE_RIGHT_20;
                    }
                    break;

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
    // AUTO SHOOT SYSTEM  (unchanged from Red)
    // =========================================================================
    private void resetAutoShoot() {
        autoShootState = AutoShootState.FIND;
        aimStableCount = 0;
        smoothedDistance = -1;
        smoothedBearing = 0;
        lastRawDistance = -1;
        shootStartTime = getRuntime();
    }

    private boolean runAutoShoot() {
        double now = getRuntime();

        switch (autoShootState) {

            case FIND: {
                AprilTagDetection tag = getBlueTag();
                if (tag != null && tag.ftcPose != null) {
                    autoShootState = AutoShootState.AIM;
                    shootStartTime = now;
                } else if (now - shootStartTime > 2.0) {
                    return true;
                }
                break;
            }

            case AIM:
                if (!updateTagSmoothingBlue()) break;

                double turnPower = aimTurnPower(smoothedBearing);
                drive(0, 0, turnPower);

                if (Math.abs(smoothedBearing) < 4.0) aimStableCount++;
                else aimStableCount = 0;

                if (aimStableCount >= 2) {
                    pickRPM(smoothedDistance);
                    startLaunch();
                    autoShootState = AutoShootState.SPINUP;
                    shootStartTime = now;
                }

                if (now - shootStartTime > 2.5) {
                    pickRPM(smoothedDistance);
                    startLaunch();
                    autoShootState = AutoShootState.SPINUP;
                    shootStartTime = now;
                }
                break;

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

            case FEED:
                if (now - shootStartTime > 1.0) {
                    if (intake != null) intake.setPower(0);
                    autoShootState = AutoShootState.STOP;
                }
                break;

            case STOP:
                stopLaunch();
                return true;
        }

        return false;
    }

    // =========================================================================
    // SMOOTHING + TAG SELECTION (BLUE)
    // =========================================================================
    private AprilTagDetection getBlueTag() {
        List<AprilTagDetection> list = new ArrayList<>(tagProcessor.getDetections());
        for (AprilTagDetection tag : list) {
            if (tag.id == BLUE_TAG_ID && tag.ftcPose != null) {
                return tag;
            }
        }
        return null;
    }

    private boolean updateTagSmoothingBlue() {
        AprilTagDetection tag = getBlueTag();
        if (tag == null || tag.ftcPose == null) return false;

        double rawDist = tag.ftcPose.range * 0.0254;
        double rawBear = tag.ftcPose.bearing;

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

    // =========================================================================
    // MOVEMENT + HELPER FUNCTIONS (same as Red)
    // =========================================================================

    private double getX() { return pinpoint.getPosX(DistanceUnit.INCH); }
    private double getY() { return pinpoint.getPosY(DistanceUnit.INCH); }

    private boolean readyToShoot() {
        double lv = leftLauncher.getVelocity();
        double rv = rightLauncher.getVelocity();
        return Math.abs(lv - currentTargetTPS) < 200 && Math.abs(rv - currentTargetTPS) < 200;
    }

    private void pickRPM(double distM) {
        if (distM < 0.9144) setRPM(SHORT_RPM);
        else if (distM < 2.7432) setRPM(MID_RPM);
        else setRPM(LONG_RPM);
    }

    private void setRPM(double rpm) {
        currentTargetTPS = rpmToTPS(rpm);
        leftLauncher.setVelocity(currentTargetTPS);
        rightLauncher.setVelocity(currentTargetTPS);
    }

    private void startLaunch() {
        leftLauncher.setVelocity(currentTargetTPS);
        rightLauncher.setVelocity(currentTargetTPS);
    }

    private void stopLaunch() {
        leftLauncher.setPower(0);
        rightLauncher.setPower(0);
    }

    // ---------- Mecanum drive ----------
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

    private double timeInState() { return getRuntime() - stateStartTime; }

    // ---------- Odometry Move X ----------
    private boolean moveToX(double targetX) {
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

    // ---------- Odometry Move Y ----------
    private boolean moveToY(double targetY) {
        double current = getY();
        double error = targetY - current;
        double power = 0.025 * error;

        power = Math.max(-0.45, Math.min(0.45, power));

        double fl = +power, fr = -power, bl = -power, br = +power;
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

    // ---------- Turning using IMU ----------
    private boolean turnToHeading(double targetDeg) {
        double current = normalize(pinpoint.getHeading(AngleUnit.DEGREES));
        double error = normalize(targetDeg - current);

        double power = kTurn * error;
        if (Math.abs(power) < TURN_MIN_POWER && Math.abs(error) > 2)
            power = Math.copySign(TURN_MIN_POWER, power);

        power = Math.max(-MAX_TURN_POWER, Math.min(MAX_TURN_POWER, power));
        setPower(-power, power, -power, power);

        return Math.abs(error) < 2;
    }

    private double normalize(double a) {
        while (a > 180) a -= 360;
        while (a <= -180) a += 360;
        return a;
    }

    private double aimTurnPower(double bearingDeg) {
        double kP = 0.035;      // proportional turning gain
        double turn = -kP * bearingDeg;

        double minPower = 0.14; // minimum turn power so robot actually moves
        double maxPower = 0.45; // max turn power so robot doesn't overshoot

        // If very small output but robot is still off target → use minimum power
        if (Math.abs(turn) < minPower && Math.abs(bearingDeg) > 3.0) {
            turn = Math.signum(turn) * minPower;
        }

        // Slow down as we get close to center (<10 degrees)
        if (Math.abs(bearingDeg) < 10.0) {
            turn *= 0.6;
        }

        // Cap power to maxPower
        if (Math.abs(turn) > maxPower) {
            turn = Math.signum(turn) * maxPower;
        }

        return turn;
    }

    private boolean recoverTagIfMissing() {
        AprilTagDetection tag = getBlueTag();
        if (tag != null) return true;

        double wigglePower = 0.22;     // slightly stronger
        int wiggleSteps = 40;          // longer time: 0.8s each direction

        // ---- PRE-PAUSE: let camera update ----
        stopDrive();
        sleep(120);

        // --- WIGGLE LEFT FIRST (BLUE) ---
        for (int i = 0; i < wiggleSteps; i++) {

            // rotate LEFT
            setPower(+wigglePower, -wigglePower, +wigglePower, -wigglePower);

            sleep(20);

            // STOP and let camera refresh
            stopDrive();
            sleep(40);

            if ((tag = getBlueTag()) != null) {
                stopDrive();
                return true;
            }
        }

        // --- WIGGLE RIGHT ---
        for (int i = 0; i < wiggleSteps; i++) {

            // rotate RIGHT
            setPower(-wigglePower, +wigglePower, -wigglePower, +wigglePower);

            sleep(20);

            // STOP and let camera refresh
            stopDrive();
            sleep(40);

            if ((tag = getBlueTag()) != null) {
                stopDrive();
                return true;
            }
        }

        stopDrive();
        return getBlueTag() != null;
    }


    // ---------- Basic Helpers ----------
    private void setBrake(DcMotor... motors) {
        for (DcMotor m : motors)
            if (m != null) m.setZeroPowerBehavior(BRAKE);
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

    private DcMotor getMotor(String name) {
        try { return hardwareMap.get(DcMotor.class, name); }
        catch (Exception e) { return null; }
    }

    private DcMotorEx getMotorEx(String name) {
        try { return hardwareMap.get(DcMotorEx.class, name); }
        catch (Exception e) { return null; }
    }
}
