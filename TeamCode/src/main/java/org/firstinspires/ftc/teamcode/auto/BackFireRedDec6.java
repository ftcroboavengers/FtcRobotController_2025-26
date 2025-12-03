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

@Autonomous(name = "Back + Fire Red dec", group = "RoboAvengers")
public class BackFireRedDec6 extends LinearOpMode {

    // ---------------- Drive and Launch Motors ----------------
    private DcMotor leftFront, rightFront, leftBack, rightBack;
    private DcMotor intake;
    private DcMotorEx leftLauncher, rightLauncher;
    private GoBildaPinpointDriver pinpoint;

    // ---------------- Motion Tuning ----------------
    private static final double MAX_DRIVE_POWER = 0.5;
    private static final double MAX_TURN_POWER  = 0.4;
    private static final double kDrive = 0.025;
    private static final double kTurn  = 0.012;
    private static final double TURN_MIN_POWER = 0.20;

    // ---------------- Launcher Power Settings ----------------
    private static double LEFT_LAUNCH_POWER  = 0.6;
    private static double RIGHT_LAUNCH_POWER = 0.6;

    // ---------------- State Machine ----------------
    private enum AutoState {
        BACK,
        FIRE,
        TURN_RIGHT,
        FORWARD,
        BACKUP,       // ← NEW
        FACE_GOAL,    // ← NEW
        FIRE_FINAL,   // ← NEW
        DONE
    }

    private AutoState state = AutoState.BACK;
    private AutoState lastState = null;

    private int stableCount = 0;
    private double lastX = 0;
    private long wrongWayCount = 0;

    private boolean xForwardIsPositive = true;

    @Override
    public void runOpMode() throws InterruptedException {

        // --- Hardware map (UNCHANGED) ---
        leftFront  = firstMotor("front_left_drive",  "frontLeftMotor");
        rightFront = firstMotor("front_right_drive", "frontRightMotor");
        leftBack   = firstMotor("back_left_drive",   "backLeftMotor");
        rightBack  = firstMotor("back_right_drive",  "backRightMotor");
        intake        = getMotor("intake");
        leftLauncher  = getMotorEx("left_launcher");
        rightLauncher = getMotorEx("right_launcher");

        if (leftFront  != null) leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        if (leftBack   != null) leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        if (rightFront != null) rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        if (rightBack  != null) rightBack.setDirection(DcMotorSimple.Direction.FORWARD);
        if (intake     != null) intake.setDirection(DcMotorSimple.Direction.REVERSE);

        setBrake(leftFront, rightFront, leftBack, rightBack);

        if (leftLauncher != null) {
            leftLauncher.setDirection(DcMotorSimple.Direction.REVERSE);
            leftLauncher.setZeroPowerBehavior(BRAKE);
            leftLauncher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        if (rightLauncher != null) {
            rightLauncher.setDirection(DcMotorSimple.Direction.FORWARD);
            rightLauncher.setZeroPowerBehavior(BRAKE);
            rightLauncher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        // --- Pinpoint init (UNCHANGED) ---
        try {
            pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
            pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
            pinpoint.setEncoderDirections(
                    GoBildaPinpointDriver.EncoderDirection.FORWARD,
                    GoBildaPinpointDriver.EncoderDirection.FORWARD
            );
            pinpoint.setOffsets(0, 0, DistanceUnit.MM);
            pinpoint.resetPosAndIMU();

            for (int i = 0; i < 30; i++) {
                pinpoint.update();
                sleep(30);
            }
            pinpoint.resetPosAndIMU();

        } catch (Exception e) {
            pinpoint = null;
        }

        waitForStart();
        xForwardIsPositive = true;

        // ---------------- MAIN LOOP ----------------
        while (opModeIsActive() && state != AutoState.DONE) {

            if (pinpoint != null) pinpoint.update();

            if (state != lastState) {
                resetRuntime();
                stableCount = 0;
                lastState = state;
            }

            switch (state) {

                // -----------------------------------------
                // 1) BACKWARD 58"
                // -----------------------------------------
                case BACK:
                    if (moveToX(mapForwardInchesToPinpointX(-58.0)) || getRuntime() > 6.0) {
                        stopDrive();
                        sleep(500);
                        state = AutoState.FIRE;
                    }
                    break;

                // -----------------------------------------
                // 2) FIRE FIRST SHOT
                // -----------------------------------------
                case FIRE:
                    startLaunchers();
                    sleep(3000);
                    if (intake != null) intake.setPower(0.8);
                    sleep(3000);
                    if (intake != null) intake.setPower(0);
                    stopLaunchers();
                    state = AutoState.TURN_RIGHT;
                    break;

                // -----------------------------------------
                // 3) TURN RIGHT -45°
                // -----------------------------------------
                case TURN_RIGHT:
                    if (turnToHeading(-45.0) || getRuntime() > 3.0) {
                        stopDrive();
                        sleep(500);
                        state = AutoState.FORWARD;
                    }
                    break;

                // -----------------------------------------
                // 4) NEW: MOVE FORWARD WHILE RUNNING INTAKE
                // -----------------------------------------
                case FORWARD:
                    if (intake != null) intake.setPower(1.0);   // ← AUTO-IN TAKE

                    if (moveToX(mapForwardInchesToPinpointX(-28.0)) || getRuntime() > 3.0) {
                        stopDrive();
                        if (intake != null) intake.setPower(0);
                        sleep(500);
                        state = AutoState.BACKUP;   // ← NEW
                    }
                    break;

                // -----------------------------------------
                // 5) NEW: BACK UP TO ORIGINAL LOCATION
                // -----------------------------------------
                case BACKUP:
                    if (moveToX(mapForwardInchesToPinpointX(-58.0)) || getRuntime() > 4.0) {
                        stopDrive();
                        sleep(400);
                        state = AutoState.FACE_GOAL;   // ← NEW
                    }
                    break;

                // -----------------------------------------
                // 6) NEW: TURN BACK TO FACE GOAL
                // -----------------------------------------
                case FACE_GOAL:
                    if (turnToHeading(0.0) || getRuntime() > 3.0) {
                        stopDrive();
                        sleep(400);
                        state = AutoState.FIRE_FINAL;  // ← NEW
                    }
                    break;

                // -----------------------------------------
                // 7) NEW: FINAL FIRE
                // -----------------------------------------
                case FIRE_FINAL:
                    startLaunchers();
                    sleep(3000);
                    if (intake != null) intake.setPower(1.0);
                    sleep(2000);
                    if (intake != null) intake.setPower(0);
                    stopLaunchers();

                    state = AutoState.DONE;
                    break;

                default:
                    state = AutoState.DONE;
                    break;
            }

            telemetry.addData("State", state);
            telemetry.update();
        }

        stopLaunchers();
        stopDrive();
    }

    // ---------------- Motion Helpers (UNCHANGED) ----------------
    private double mapForwardInchesToPinpointX(double forwardInches) {
        return xForwardIsPositive ? forwardInches : -forwardInches;
    }

    private boolean moveToX(double targetXInches) {
        if (pinpoint == null) return true;
        double currentX = pinpoint.getPosX(DistanceUnit.INCH);
        double error = targetXInches - currentX;
        double power = kDrive * error;
        power = Math.max(-MAX_DRIVE_POWER, Math.min(MAX_DRIVE_POWER, power));
        setPower(power, power, power, power);
        double dx = currentX - lastX;
        if (Math.abs(power) > 0.05 && Math.abs(dx) > 0.01) {
            boolean goingOpposite = Math.signum(power) != Math.signum(dx);
            wrongWayCount = goingOpposite ? wrongWayCount + 1 : 0;
            if (wrongWayCount > 15) {
                stopDrive();
                wrongWayCount = 0;
                return true;
            }
        } else wrongWayCount = 0;
        lastX = currentX;
        return Math.abs(error) < 2.0;
    }

    private double normalize180(double angle) {
        while (angle > 180) angle -= 360;
        while (angle <= -180) angle += 360;
        return angle;
    }

    private boolean turnToHeading(double targetDeg) {
        double current = normalize180(pinpoint.getHeading(AngleUnit.DEGREES));
        double error = normalize180(targetDeg - current);
        double power = kTurn * error;
        if (Math.abs(power) < TURN_MIN_POWER && Math.abs(error) > 2)
            power = Math.copySign(TURN_MIN_POWER, power);
        power = Math.max(-MAX_TURN_POWER, Math.min(MAX_TURN_POWER, power));
        setPower(-power, power, -power, power);
        boolean onTarget = Math.abs(error) < 5.0;
        if (onTarget) stableCount++; else stableCount = 0;
        return stableCount > 10;
    }

    private void startLaunchers() {
        if (leftLauncher != null)  leftLauncher.setPower(LEFT_LAUNCH_POWER);
        if (rightLauncher != null) rightLauncher.setPower(RIGHT_LAUNCH_POWER);
    }

    private void stopLaunchers() {
        if (leftLauncher != null)  leftLauncher.setPower(0);
        if (rightLauncher != null) rightLauncher.setPower(0);
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

    private void setBrake(DcMotor... motors) {
        for (DcMotor m : motors)
            if (m != null) m.setZeroPowerBehavior(BRAKE);
    }

    private DcMotor firstMotor(String primary, String alt) {
        DcMotor m = getMotor(primary);
        if (m == null) m = getMotor(alt);
        return m;
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
