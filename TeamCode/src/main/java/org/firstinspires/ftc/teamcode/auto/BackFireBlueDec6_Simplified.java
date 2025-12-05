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

@Autonomous(name = "Back + Shoot Blue", group = "RoboAvengers")
public class BackFireBlueDec6_Simplified extends LinearOpMode {

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

    // Global speed scales
    private static final double DRIVE_SPEED_SCALE = 0.5;  // 0.5 = 50% of normal drive speed
    private static final double TURN_SPEED_SCALE  = 0.5;  // 0.5 = 50% of normal turn speed

    // ---------------- Launcher RPM ----------------
    private static final double TPR = 28.0;
    private static double rpmToTPS(double rpm) { return rpm * TPR / 60.0; }

    private static final double FIXED_RPM = 3000;
    private static final double FIXED_F   = 18.0;

    private double targetTPS = rpmToTPS(FIXED_RPM);

    // ---------------- Simple Shoot State ----------------
    private enum ShootState { SPINUP, FEED, DONE }
    private ShootState shootState;
    private double shootStartTime = 0;

    // ---------------- Auto States ----------------
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

    private AutoState state     = AutoState.BACK_58;
    private AutoState lastState = null;

    private double stateStartTime = 0;

    // Odometry tracking
    private double lastX = 0, lastY = 0;
    private long wrongX = 0, wrongY = 0;

    private double strafeTargetY = 0;

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
        if (leftFront != null)  leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        if (leftBack != null)   leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        if (rightFront != null) rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        if (rightBack != null)  rightBack.setDirection(DcMotorSimple.Direction.FORWARD);

        if (intake != null) intake.setDirection(DcMotorSimple.Direction.REVERSE);

        setBrake(leftFront, rightFront, leftBack, rightBack);

        // Launchers
        initLauncher(leftLauncher, true);
        initLauncher(rightLauncher, false);

        // Pinpoint
        initPinpoint();

        telemetry.addLine("Ready (3000 RPM, Slow, No Tags)");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        resetShoot();

        // ---------------- MAIN AUTO LOOP ----------------
        while (opModeIsActive() && state != AutoState.DONE) {

            if (pinpoint != null) {
                pinpoint.update();
            }

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
                    if (runSimpleShoot()) {
                        stopLaunch();
                        state = AutoState.TURN_LEFT;
                    }
                    break;

                // BLUE TURN IS +45°
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

                // Turn back to 0°
                case AIM_FINAL:
                    if (turnToHeading(0) || timeInState() > 3.0) {
                        stopDrive();
                        sleep(200);
                        resetShoot();
                        state = AutoState.AUTOSHOOT_2;
                    }
                    break;

                case AUTOSHOOT_2:
                    if (runSimpleShoot()) {
                        stopLaunch();
                        strafeTargetY = getY() - 40.0;
                        state = AutoState.STRAFE_RIGHT_20;
                    }
                    break;

                case STRAFE_RIGHT_20:
                    if (moveToY(strafeTargetY) || timeInState() > 3.0) {
                        stopDrive();
                        state = AutoState.DONE;
                    }
                    break;
            }

            telemetry.addData("State", state);
            telemetry.addData("Heading", pinpoint != null ? pinpoint.getHeading(AngleUnit.DEGREES) : 0);
            telemetry.addData("X", getX());
            telemetry.addData("Y", getY());
            telemetry.update();
        }

        stopDrive();
        stopLaunch();
    }

    // =========================================================================
    // SIMPLE 3000 RPM SHOOTING
    // =========================================================================
    private void resetShoot() {
        shootState = ShootState.SPINUP;
        shootStartTime = getRuntime();
        targetTPS = rpmToTPS(FIXED_RPM);
    }

    private boolean runSimpleShoot() {
        double now = getRuntime();

        switch (shootState) {
            case SPINUP:
                startLaunch();
                if (now - shootStartTime > 0.6) {   // spin-up time
                    shootState = ShootState.FEED;
                    shootStartTime = now;
                }
                break;

            case FEED:
                if (intake != null) intake.setPower(1.0);
                if (now - shootStartTime > 1.0) {   // feed time
                    if (intake != null) intake.setPower(0);
                    shootState = ShootState.DONE;
                }
                break;

            case DONE:
                return true;
        }

        return false;
    }

    private void startLaunch() {
        if (leftLauncher != null)  leftLauncher.setVelocity(targetTPS);
        if (rightLauncher != null) rightLauncher.setVelocity(targetTPS);
    }

    private void stopLaunch() {
        if (leftLauncher != null)  leftLauncher.setPower(0);
        if (rightLauncher != null) rightLauncher.setPower(0);
    }

    // =========================================================================
    // TURNING (with TURN_SPEED_SCALE)
    // =========================================================================
    private boolean turnToHeading(double targetDeg) {
        if (pinpoint == null) return true;

        double current = normalize(pinpoint.getHeading(AngleUnit.DEGREES));
        double error   = normalize(targetDeg - current);

        double power = kTurn * error;

        if (Math.abs(power) < TURN_MIN_POWER && Math.abs(error) > 2) {
            power = Math.copySign(TURN_MIN_POWER, power);
        }

        // Clamp to max and apply global TURN_SPEED_SCALE
        power = Math.max(-MAX_TURN_POWER, Math.min(MAX_TURN_POWER, power));
        power *= TURN_SPEED_SCALE;

        setPower(-power, power, -power, power);

        return Math.abs(error) < 2;
    }

    private double normalize(double a) {
        while (a > 180)  a -= 360;
        while (a <= -180) a += 360;
        return a;
    }

    // =========================================================================
    // ODOMETRY & MOVEMENT (with DRIVE_SPEED_SCALE)
    // =========================================================================
    private double getX() {
        return (pinpoint != null) ? pinpoint.getPosX(DistanceUnit.INCH) : 0;
    }

    private double getY() {
        return (pinpoint != null) ? pinpoint.getPosY(DistanceUnit.INCH) : 0;
    }

    private double timeInState() {
        return getRuntime() - stateStartTime;
    }

    private void stopDrive() {
        setPower(0, 0, 0, 0);
    }

    private boolean moveToX(double targetX) {
        double x = getX();
        double error = targetX - x;
        double power = kDrive * error;

        // Clip then scale
        power = Math.max(-MAX_DRIVE_POWER, Math.min(MAX_DRIVE_POWER, power));
        power *= DRIVE_SPEED_SCALE;

        drive(power, 0, 0);

        double dx = x - lastX;
        if (Math.abs(power) > 0.05 && Math.abs(dx) > 0.01) {
            if (Math.signum(power) != Math.signum(dx)) wrongX++;
            else wrongX = 0;
            if (wrongX > 15) return true;
        }
        lastX = x;

        return Math.abs(error) < 2.0;
    }

    private boolean moveToY(double targetY) {
        double y = getY();
        double error = targetY - y;

        double power = 0.025 * error;

        // Clip then scale
        power = Math.max(-0.45, Math.min(0.45, power));
        power *= DRIVE_SPEED_SCALE;

        double fl = +power, fr = -power, bl = -power, br = +power;
        setPower(fl, fr, bl, br);

        double dy = y - lastY;
        if (Math.abs(power) > 0.05 && Math.abs(dy) > 0.01) {
            if (Math.signum(power) != Math.signum(dy)) wrongY++;
            else wrongY = 0;
            if (wrongY > 15) return true;
        }
        lastY = y;

        return Math.abs(error) < 2.0;
    }

    private void drive(double fwd, double strafe, double turn) {
        double fl = fwd + strafe + turn;
        double fr = fwd - strafe - turn;
        double bl = fwd - strafe + turn;
        double br = fwd + strafe - turn;
        setPower(fl, fr, bl, br);
    }

    private void setBrake(DcMotor... motors) {
        for (DcMotor m : motors) {
            if (m != null) {
                m.setZeroPowerBehavior(BRAKE);
            }
        }
    }

    private void setPower(double fl, double fr, double bl, double br) {
        if (leftFront != null)  leftFront.setPower(fl);
        if (rightFront != null) rightFront.setPower(fr);
        if (leftBack != null)   leftBack.setPower(bl);
        if (rightBack != null)  rightBack.setPower(br);
    }

    // =========================================================================
    // INIT HELPERS
    // =========================================================================
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
                    GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD
            );

            // Same as your working file
            pinpoint.setEncoderDirections(
                    GoBildaPinpointDriver.EncoderDirection.FORWARD,
                    GoBildaPinpointDriver.EncoderDirection.FORWARD
            );

            pinpoint.setOffsets(0, 0, DistanceUnit.MM);

            pinpoint.resetPosAndIMU();
            for (int i = 0; i < 25; i++) {
                pinpoint.update();
                sleep(20);
            }
            pinpoint.resetPosAndIMU();

        } catch (Exception e) {
            pinpoint = null;
        }
    }

    private DcMotor getMotor(String name) {
        try {
            return hardwareMap.get(DcMotor.class, name);
        } catch (Exception e) {
            return null;
        }
    }

    private DcMotorEx getMotorEx(String name) {
        try {
            return hardwareMap.get(DcMotorEx.class, name);
        } catch (Exception e) {
            return null;
        }
    }
}
