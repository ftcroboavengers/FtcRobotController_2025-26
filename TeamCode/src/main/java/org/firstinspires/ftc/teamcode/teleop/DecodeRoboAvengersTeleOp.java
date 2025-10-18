package org.firstinspires.ftc.teamcode.teleop;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.List;

@TeleOp(name = "Decode RoboAvengers TeleOp 2025 (Pinpoint)", group = "RoboAvengers")
public class DecodeRoboAvengersTeleOp extends LinearOpMode {

    // ----------------- Drive / Pinpoint -----------------
    private DcMotor leftFront, rightFront, leftBack, rightBack;
    private GoBildaPinpointDriver pinpoint;
    private boolean fieldCentric = true;
    private double headingOffsetRad = 0.0;

    // ----------------- Intake / Launch / Servos -----------------
    private DcMotor intake;
    private DcMotorEx leftLauncher, rightLauncher;
    private CRServo leftFeeder, rightFeeder;
    private Servo diverter;

    // ----------------- Distance Sensor -----------------
    private DistanceSensor frontDistance;
    private static final double AUTO_STOP_INTAKE_CM = 6.0; // set as needed

    // ----------------- Launcher state -----------------
    private enum LaunchState { IDLE, SPIN_UP, LAUNCH, LAUNCHING }
    private LaunchState leftState = LaunchState.IDLE;
    private LaunchState rightState = LaunchState.IDLE;
    private final ElapsedTime leftFeedTimer = new ElapsedTime();
    private final ElapsedTime rightFeedTimer = new ElapsedTime();

    private static final double LAUNCH_CLOSE_TARGET = 1200;
    private static final double LAUNCH_CLOSE_MIN    = 1175;
    private static final double LAUNCH_FAR_TARGET   = 1350;
    private static final double LAUNCH_FAR_MIN      = 1325;
    private double launcherTarget = LAUNCH_CLOSE_TARGET;
    private double launcherMin    = LAUNCH_CLOSE_MIN;

    private static final double FEED_TIME_S = 0.80;

    private static final double DIVERTER_LEFT  = 0.2962;
    private static final double DIVERTER_RIGHT = 0.0;

    // ----------------- Vision / AprilTag -----------------
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    @Override
    public void runOpMode() throws InterruptedException {
        // ---- Map drive motors ----
        leftFront  = firstMotor("front_left_drive", "frontLeftMotor");
        rightFront = firstMotor("front_right_drive", "frontRightMotor");
        leftBack   = firstMotor("back_left_drive",  "backLeftMotor");
        rightBack  = firstMotor("back_right_drive", "backRightMotor");

        if (leftFront == null || rightFront == null || leftBack == null || rightBack == null) {
            telemetry.addLine("ERROR: One or more drive motors not found.");
            telemetry.update();
        }

        intake       = getMotor("intake");
        leftLauncher = getMotorEx("left_launcher");
        rightLauncher= getMotorEx("right_launcher");
        leftFeeder   = getCRServo("left_feeder");
        rightFeeder  = getCRServo("right_feeder");
        diverter     = getServo("diverter");
        frontDistance= getDistance("frontDistance");

        // ---- Motor directions ----
        if (leftFront != null)  leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        if (leftBack  != null)  leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        if (rightFront!= null)  rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        if (rightBack != null)  rightBack.setDirection(DcMotorSimple.Direction.FORWARD);
        setBrake(leftFront, rightFront, leftBack, rightBack);

        // ---- Pinpoint setup ----
        try {
            pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

            // Configure your odometry pod type and encoder directions
            pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
            pinpoint.setEncoderDirections(
                    GoBildaPinpointDriver.EncoderDirection.FORWARD,
                    GoBildaPinpointDriver.EncoderDirection.REVERSED
            );

            // Set offsets (mm) if needed to match your robot geometry
            pinpoint.setOffsets(0, 0, DistanceUnit.MM);
            pinpoint.resetPosAndIMU();
            zeroHeading();
        } catch (Exception e) {
            telemetry.addLine("Pinpoint not found — field-centric disabled.");
            fieldCentric = false;
        }

        // ---- Vision setup ----
        try {
            aprilTag = new AprilTagProcessor.Builder().build();
            VisionPortal.Builder builder = new VisionPortal.Builder();
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
            builder.addProcessor(aprilTag);
            visionPortal = builder.build();
        } catch (Exception ignored) {
            telemetry.addLine("No camera / AprilTag; continuing without vision.");
        }

        telemetry.addLine("RoboAvengers TeleOp READY. Press Play.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            // --------------- DRIVE ---------------
            double y = -gamepad1.left_stick_y;
            double x =  gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            if (gamepad1.x) fieldCentric = !fieldCentric;
            if (gamepad1.y) zeroHeading();

            double driveScale = gamepad1.right_bumper ? 1.0 : (gamepad1.left_bumper ? 0.4 : 0.7);
            mecanumDrive(y, x, rx, driveScale);

            // --------------- INTAKE ---------------
            if (intake != null) {
                double in = gamepad2.right_trigger;
                double out = gamepad2.left_trigger;
                double p = in - out;

                if (frontDistance != null && in > 0.1) {
                    double cm = frontDistance.getDistance(org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.CM);
                    if (cm > 0 && cm < AUTO_STOP_INTAKE_CM) p = 0.0;
                }
                intake.setPower(p);
            }

            // --------------- LAUNCHER ---------------
            if (leftLauncher != null && rightLauncher != null) {
                if (gamepad2.a) { launcherTarget = LAUNCH_CLOSE_TARGET; launcherMin = LAUNCH_CLOSE_MIN; }
                if (gamepad2.b) { launcherTarget = LAUNCH_FAR_TARGET;   launcherMin = LAUNCH_FAR_MIN;   }
                if (gamepad2.x) stopLaunchers();
                boolean shot = gamepad2.right_bumper;
                doLaunch(leftLauncher, leftFeeder, true,  shot);
                doLaunch(rightLauncher,rightFeeder,false, shot);
            }

            // --------------- DIVERTER ---------------
            if (diverter != null) {
                if (gamepad2.dpad_left)  diverter.setPosition(DIVERTER_LEFT);
                if (gamepad2.dpad_right) diverter.setPosition(DIVERTER_RIGHT);
            }

            // --------------- VISION TELEMETRY ---------------
            if (aprilTag != null) {
                List<AprilTagDetection> dets = aprilTag.getDetections();
                telemetry.addData("Tags", dets.size());
                if (!dets.isEmpty()) {
                    AprilTagDetection d = dets.get(0);
                    telemetry.addData("Tag ID", d.id);
                    telemetry.addData("Pose X,Y,Z (m)", "%.2f, %.2f, %.2f", d.ftcPose.x, d.ftcPose.y, d.ftcPose.z);
                }
            }

            // --------------- TELEMETRY ---------------
            if (pinpoint != null) {
                pinpoint.update();
                double yaw = getYawRad();
                telemetry.addData("Heading(deg)", Math.toDegrees(yaw));
                telemetry.addData("FieldCentric", fieldCentric);
                telemetry.addData("X(mm)", pinpoint.getPosX(DistanceUnit.MM));
                telemetry.addData("Y(mm)", pinpoint.getPosY(DistanceUnit.MM));
            }
            telemetry.update();
        }

        stopLaunchers();
        setPower(0,0,0,0);
    }

    // ----------------- Helpers -----------------
    private void mecanumDrive(double y, double x, double rx, double scale) {
        if (leftFront == null) return;
        double rotX = x;
        double rotY = y;
        if (fieldCentric && pinpoint != null) {
            double yaw = getYawRad();
            double cosA = Math.cos(-yaw);
            double sinA = Math.sin(-yaw);
            rotX = x * cosA - y * sinA;
            rotY = x * sinA + y * cosA;
        }

        double denom = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1.0);
        double fl = (rotY + rotX + rx) / denom * scale;
        double bl = (rotY - rotX + rx) / denom * scale;
        double fr = (rotY - rotX - rx) / denom * scale;
        double br = (rotY + rotX - rx) / denom * scale;
        setPower(fl, fr, bl, br);
    }

    private void setPower(double fl, double fr, double bl, double br) {
        if (leftFront != null)  leftFront.setPower(fl);
        if (rightFront != null) rightFront.setPower(fr);
        if (leftBack != null)   leftBack.setPower(bl);
        if (rightBack != null)  rightBack.setPower(br);
    }

    private void setBrake(DcMotor... motors) {
        for (DcMotor m : motors) if (m != null) m.setZeroPowerBehavior(BRAKE);
    }

    private void zeroHeading() {
        if (pinpoint != null) {
            pinpoint.resetPosAndIMU();
            headingOffsetRad = 0.0;
        }
    }

    private double getYawRad() {
        if (pinpoint == null) return 0.0;
        // ✅ Specify AngleUnit.RADIANS explicitly
        return pinpoint.getHeading(AngleUnit.RADIANS) - headingOffsetRad;
    }

    private void doLaunch(DcMotorEx launcher, CRServo feeder, boolean isLeft, boolean shotRequested) {
        if (launcher == null) return;
        LaunchState state = isLeft ? leftState : rightState;
        switch (state) {
            case IDLE:
                if (shotRequested) state = LaunchState.SPIN_UP;
                break;
            case SPIN_UP:
                launcher.setVelocity(launcherTarget);
                if (Math.abs(launcher.getVelocity()) >= launcherMin) {
                    state = LaunchState.LAUNCH;
                }
                break;
            case LAUNCH:
                if (feeder != null) feeder.setPower(1.0);
                if (isLeft) leftFeedTimer.reset(); else rightFeedTimer.reset();
                state = LaunchState.LAUNCHING;
                break;
            case LAUNCHING:
                double t = isLeft ? leftFeedTimer.seconds() : rightFeedTimer.seconds();
                if (t >= FEED_TIME_S) {
                    if (feeder != null) feeder.setPower(0.0);
                    state = LaunchState.IDLE;
                }
                break;
        }
        if (isLeft) leftState = state; else rightState = state;
    }

    private void stopLaunchers() {
        if (leftLauncher  != null) leftLauncher.setPower(0);
        if (rightLauncher != null) rightLauncher.setPower(0);
        if (leftFeeder    != null) leftFeeder.setPower(0);
        if (rightFeeder   != null) rightFeeder.setPower(0);
        leftState = LaunchState.IDLE;
        rightState = LaunchState.IDLE;
    }

    // -------- Safe hardware getters --------
    private DcMotor firstMotor(String primary, String alt) {
        DcMotor m = getMotor(primary);
        if (m == null) m = getMotor(alt);
        return m;
    }

    private DcMotor getMotor(String name) {
        try { return hardwareMap.get(DcMotor.class, name); } catch (Exception e) { return null; }
    }

    private DcMotorEx getMotorEx(String name) {
        try { return hardwareMap.get(DcMotorEx.class, name); } catch (Exception e) { return null; }
    }

    private CRServo getCRServo(String name) {
        try { return hardwareMap.get(CRServo.class, name); } catch (Exception e) { return null; }
    }

    private Servo getServo(String name) {
        try { return hardwareMap.get(Servo.class, name); } catch (Exception e) { return null; }
    }

    private DistanceSensor getDistance(String name) {
        try { return hardwareMap.get(DistanceSensor.class, name); } catch (Exception e) { return null; }
    }
}
