package org.firstinspires.ftc.teamcode.teleop;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Decode Launch Test", group = "Test")
public class DecodeRoboAvengersLaunch extends LinearOpMode {

        // ----------------- Drive / Pinpoint -----------------
        private DcMotor leftFront, rightFront, leftBack, rightBack;
        private GoBildaPinpointDriver pinpoint;
        private boolean fieldCentric = true;
        private double headingOffsetRad = 0.0;

        // ----------------- Intake / Launch -----------------
        private DcMotor intake;
        private DcMotorEx leftLauncher, rightLauncher;

        // ----------------- Launcher Targets -----------------
        private static final double LAUNCH_CLOSE_TARGET = 7500;  // ≈4020 RPM
        private static final double LAUNCH_FAR_TARGET   = 8800;  // ≈4710 RPM
        private double launcherTarget = LAUNCH_CLOSE_TARGET;

        // ----------------- Velocity / tolerance -----------------
        private static final int VEL_TOL = 150;
        private static final int READY_CYCLES = 5;
        private int leftReadyCount = 0, rightReadyCount = 0;

        // ----------------- Per-Side Launch States -----------------
        private enum LaunchState { IDLE, SPIN_UP, LAUNCHING, STOPPING }
        private LaunchState leftState = LaunchState.IDLE;
        private LaunchState rightState = LaunchState.IDLE;

        private ElapsedTime leftTimer = new ElapsedTime();
        private ElapsedTime rightTimer = new ElapsedTime();
        private static final double FEED_TIME_SECONDS = 0.8;
        private static final double FEED_POWER = 0.8;

        @Override
        public void runOpMode() throws InterruptedException {

            // ---- Map drive motors ----
            leftFront = firstMotor("front_left_drive", "frontLeftMotor");
            rightFront = firstMotor("front_right_drive", "frontRightMotor");
            leftBack  = firstMotor("back_left_drive",  "backLeftMotor");
            rightBack = firstMotor("back_right_drive", "backRightMotor");

            intake = getMotor("intake");
            leftLauncher  = getMotorEx("left_launcher");
            rightLauncher = getMotorEx("right_launcher");

            // ---- Launcher setup ----
            if (leftLauncher != null) {
                leftLauncher.setZeroPowerBehavior(BRAKE);
                leftLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                leftLauncher.setVelocityPIDFCoefficients(25.0, 0.0, 5.0, 12.0);
            }
            if (rightLauncher != null) {
                rightLauncher.setZeroPowerBehavior(BRAKE);
                rightLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightLauncher.setVelocityPIDFCoefficients(25.0, 0.0, 5.0, 12.0);
            }

            // ---- Intake ----
            if (intake != null) intake.setDirection(DcMotorSimple.Direction.REVERSE);

            // ---- Drive directions ----
            if (leftFront != null) leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
            if (leftBack  != null) leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
            if (rightFront!= null) rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
            if (rightBack != null) rightBack.setDirection(DcMotorSimple.Direction.FORWARD);
            setBrake(leftFront, rightFront, leftBack, rightBack);

            // ---- Pinpoint setup ----
            try {
                pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
                pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
                pinpoint.setEncoderDirections(
                        GoBildaPinpointDriver.EncoderDirection.FORWARD,
                        GoBildaPinpointDriver.EncoderDirection.FORWARD
                );
                pinpoint.setOffsets(0, 0, DistanceUnit.MM);
                pinpoint.resetPosAndIMU();
                zeroHeading();
            } catch (Exception e) {
                telemetry.addLine("Pinpoint not found — field-centric disabled.");
                fieldCentric = false;
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

                // --------------- INTAKE (manual control from both gamepads) ---------------
                if (intake != null) {
                    double in1 = gamepad1.right_trigger;
                    double out1 = gamepad1.left_trigger;
                    double in2 = gamepad2.right_trigger;
                    double out2 = gamepad2.left_trigger;

                    double in = Math.max(in1, in2);
                    double out = Math.max(out1, out2);

                    double p = 0;
                    if (out > 0.01) {
                        p = -out; // reverse
                    } else if (in > 0.01) {
                        p = in; // forward
                    }
                    intake.setPower(p);
                }

                // --------------- LAUNCHER DISTANCE PRESETS ---------------
                if (gamepad2.a) launcherTarget = LAUNCH_CLOSE_TARGET;
                if (gamepad2.b) launcherTarget = LAUNCH_FAR_TARGET;

                // --------------- PER-SIDE SHOOT CONTROLS ---------------
                launchLeft(gamepad2.left_bumper);
                launchRight(gamepad2.right_bumper);

                // --------------- TELEMETRY ---------------
                if (pinpoint != null) {
                    pinpoint.update();
                    double yaw = getYawRad();
                    telemetry.addData("Heading(deg)", Math.toDegrees(yaw));
                    telemetry.addData("FieldCentric", fieldCentric);
                    telemetry.addData("X(mm)", pinpoint.getPosX(DistanceUnit.MM));
                    telemetry.addData("Y(mm)", pinpoint.getPosY(DistanceUnit.MM));
                }

                telemetry.addData("Launcher Target", launcherTarget);
                telemetry.addData("Left Vel",  leftLauncher != null ? leftLauncher.getVelocity() : 0);
                telemetry.addData("Right Vel", rightLauncher != null ? rightLauncher.getVelocity() : 0);
                telemetry.addData("Left State", leftState);
                telemetry.addData("Right State", rightState);
                telemetry.update();
            }

            stopLaunchers();
            setPower(0, 0, 0, 0);
        }

        // ----------------- DRIVE HELPERS -----------------
        private void mecanumDrive(double y, double x, double rx, double scale) {
            if (leftFront == null) return;
            double rotX = x, rotY = y;
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
            if (leftFront  != null) leftFront.setPower(fl);
            if (rightFront != null) rightFront.setPower(fr);
            if (leftBack   != null) leftBack.setPower(bl);
            if (rightBack  != null) rightBack.setPower(br);
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
            return pinpoint.getHeading(AngleUnit.RADIANS) - headingOffsetRad;
        }

        // ----------------- LAUNCHER HELPERS -----------------
        private void startLeftLauncher() {
            if (leftLauncher == null) return;
            double adjusted = getVoltageCompensatedVelocity(launcherTarget);
            leftLauncher.setVelocity(adjusted);
        }

        private void startRightLauncher() {
            if (rightLauncher == null) return;
            double adjusted = getVoltageCompensatedVelocity(launcherTarget);
            rightLauncher.setVelocity(adjusted);
        }

        private void stopLeftLauncher() {
            if (leftLauncher != null) leftLauncher.setPower(0);
        }

        private void stopRightLauncher() {
            if (rightLauncher != null) rightLauncher.setPower(0);
        }

        private void stopLaunchers() {
            stopLeftLauncher();
            stopRightLauncher();
        }

        private double getVoltageCompensatedVelocity(double targetTicksPerSec) {
            double nominalVoltage = 13.0;
            double currentVoltage = 12.0;
            try {
                currentVoltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
            } catch (Exception e) {
                telemetry.addLine("Voltage sensor not found — using 12V default");
            }
            return targetTicksPerSec * (nominalVoltage / currentVoltage);
        }

        private boolean leftReady() {
            if (leftLauncher == null) return false;
            boolean inTol = Math.abs(leftLauncher.getVelocity() - launcherTarget) <= VEL_TOL;
            leftReadyCount = inTol ? Math.min(READY_CYCLES, leftReadyCount + 1) : 0;
            return leftReadyCount >= READY_CYCLES;
        }

        private boolean rightReady() {
            if (rightLauncher == null) return false;
            boolean inTol = Math.abs(rightLauncher.getVelocity() - launcherTarget) <= VEL_TOL;
            rightReadyCount = inTol ? Math.min(READY_CYCLES, rightReadyCount + 1) : 0;
            return rightReadyCount >= READY_CYCLES;
        }

        // ----------------- PER-SIDE STATE MACHINES -----------------
        private void launchLeft(boolean shootRequested) {
            if (leftLauncher == null || intake == null) return;

            switch (leftState) {
                case IDLE:
                    if (shootRequested) {
                        startLeftLauncher();
                        leftState = LaunchState.SPIN_UP;
                    }
                    break;

                case SPIN_UP:
                    if (leftReady()) {
                        intake.setPower(FEED_POWER);
                        leftTimer.reset();
                        leftState = LaunchState.LAUNCHING;
                    }
                    break;

                case LAUNCHING:
                    if (leftTimer.seconds() > FEED_TIME_SECONDS) {
                        intake.setPower(0);
                        leftTimer.reset();                 // reuse timer
                        leftState = LaunchState.STOPPING;  // <-- new delay state
                    }
                    break;

                case STOPPING:
                    if (leftTimer.seconds() > 0.25) {      // <-- small delay before stop
                        stopLeftLauncher();
                        leftState = LaunchState.IDLE;
                    }
                    break;
            }
        }

        private void launchRight(boolean shootRequested) {
            if (leftLauncher == null || intake == null) return;

            switch (rightState) {
                case IDLE:
                    if (shootRequested) {
                        startRightLauncher();
                        rightState = LaunchState.SPIN_UP;
                    }
                    break;

                case SPIN_UP:
                    if (rightReady()) {
                        intake.setPower(FEED_POWER);
                        rightTimer.reset();
                        leftState = LaunchState.LAUNCHING;
                    }
                    break;

                case LAUNCHING:
                    if (rightTimer.seconds() > FEED_TIME_SECONDS) {
                        intake.setPower(0);
                        rightTimer.reset();                 // reuse timer
                        rightState = LaunchState.STOPPING;  // <-- new delay state
                    }
                    break;

                case STOPPING:
                    if (rightTimer.seconds() > 0.25) {      // <-- small delay before stop
                        stopRightLauncher();
                        rightState = LaunchState.IDLE;
                    }
                    break;
            }
        }

        // ----------------- Safe Hardware Getters -----------------
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
