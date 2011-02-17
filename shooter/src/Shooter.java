import lejos.nxt.*;
import lejos.nxt.addon.TiltSensor;
import lejos.util.Delay;

public class Shooter {
    private final static int POWER_MAX = 100;
    private final static int LEFT_LIMIT_TO_ZERO = 180;
    private final static int RIGHT_LIMIT_TO_ZERO = 90;
    private final static int PID_LOOP_WAIT = 10;
    private final static int PID_P = 2;
    private final static int PID_D = 5;

    private NXTRegulatedMotor leftMotor = Motor.A;
    private NXTRegulatedMotor rightMotor = Motor.C;

    private int leftZero, rightZero, leftTarget, rightTarget;

    public static void main(String[] args) {
//        RConsole.open();

        Shooter shooter = new Shooter();
        shooter.goToStartPosition();
        TaskMovementMotors taskMovementMotors = new TaskMovementMotors(shooter);
        taskMovementMotors.start();
        Delay.msDelay(5000);
        ReadAccelSensor accelSensor = new ReadAccelSensor(shooter);
        accelSensor.start();
        ShootBall shootBall = new ShootBall();
        shootBall.start();
    }

    public int getLeftTarget() {
        return leftTarget;
    }

    public int getRightTarget() {
        return rightTarget;
    }

    public void setLeftTarget(int leftTarget) {
        this.leftTarget = leftTarget;
    }

    public void setRightTarget(int rightTarget) {
        this.rightTarget = rightTarget;
    }

    public NXTRegulatedMotor getLeftMotor() {
        return leftMotor;
    }

    public NXTRegulatedMotor getRightMotor() {
        return rightMotor;
    }

    public int getLeftZero() {
        return leftZero;
    }

    public int getRightZero() {
        return rightZero;
    }

    private static class ShootBall extends Thread {
        TouchSensor touchSensor = new TouchSensor(SensorPort.S1);

        @Override
        public void run() {
            while (!Button.ESCAPE.isPressed()) {
                if (touchSensor.isPressed()) {
                    Motor.B.resetTachoCount();
                    Motor.B.setSpeed(400);
                    Motor.B.forward();
                    int count = 0;
                    while (count < 360) count = Motor.B.getTachoCount();
                    Motor.B.stop();
                }
                Delay.msDelay(100);
            }
        }
    }

    private static class ReadAccelSensor extends Thread {
        private final Shooter shooter;
        private final TiltSensor accelSensor;

        public ReadAccelSensor(Shooter shooter) {
            super();
            this.shooter = shooter;
            this.accelSensor = new TiltSensor(SensorPort.S2);
        }

        @Override
        public void run() {
            int x, y, z;

            int h, v;
            int eLT, eRT;

            LCD.clearDisplay();
            LCD.drawString(accelSensor.getProductID() + " " + accelSensor.getSensorType(), 0, 4);

            while (!Button.ESCAPE.isPressed()) {
                x = accelSensor.getXTilt();
                y = accelSensor.getYTilt();
                z = accelSensor.getZTilt();

//                LCD.drawString("x|y|z - " + format(x) + "|" + format(y) + "|" + format(z), 0 , 1);

                if (y > 199) {
                    h = -2 * (y - 200);
                } else {
                    h = 2 * y;
                }

                // Set vertical value (map sensor z range 80 to -20 to range 0 to 100)
                if (z > 199) {
                    v = z - 170;
                } else {
                    v = z - 20;
                }

                // Calc left and right target, do not do math directly with
                // target variable as compiler may set transient values to variable
                // which can affect servo task
                eLT = (v + h) / 2;
                eRT = (v - h) / 2;
                shooter.setLeftTarget(eLT);
                shooter.setRightTarget(eRT);

//                LCD.drawString("Target : " + format(eLT) + "|" + format(eRT), 0, 1);

                Delay.msDelay(10);
            }
        }

        private String format(int z) {
            String s = Integer.toString(z);
            while (s.length() < 3 && s.charAt(0) != '-') {
                s = "0" + s;
            }
            return s;
        }

    }

    public static class TaskMovementMotors extends Thread {
        int errLeftPrev, errRightPrev;
        int eLeft, eRight;
        int errLeft, errRight;
        int pwrLeft, pwrRight;

        private NXTRegulatedMotor leftMotor;
        private NXTRegulatedMotor rightMotor;
        private int leftZero, rightZero, leftTarget, rightTarget;
        private Shooter shooter;

        public TaskMovementMotors(Shooter shooter) {
            this.leftMotor = shooter.getLeftMotor();
            this.rightMotor = shooter.getRightMotor();
            this.leftZero = shooter.getLeftZero();
            this.rightZero = shooter.getRightZero();
            this.shooter = shooter;
        }

        @Override
        public void run() {
            eLeft = leftMotor.getPosition() - leftZero;
            eRight = rightMotor.getPosition() - rightZero;

            errLeft = shooter.getLeftTarget() - eLeft;
            errRight = rightTarget - eRight;

            while (!Button.ESCAPE.isPressed()) {
//                RConsole.print(errLeft + "|" + errRight + "|");
                Delay.msDelay(PID_LOOP_WAIT);
                errLeftPrev = errLeft;
                errRightPrev = errRight;

                eLeft = leftMotor.getPosition() - leftZero;
                eRight = rightMotor.getPosition() - rightZero;

                errLeft = shooter.getLeftTarget() - eLeft;
                errRight = shooter.getRightTarget() - eRight;

                pwrLeft = errLeft * PID_P + (errLeft - errLeftPrev) * PID_D;
                pwrRight = errRight * PID_P + (errRight - errRightPrev) * PID_D;

                if (pwrLeft < -POWER_MAX) pwrLeft = -POWER_MAX;
                if (pwrLeft > POWER_MAX) pwrLeft = POWER_MAX;
                if (pwrRight < -POWER_MAX) pwrRight = -POWER_MAX;
                if (pwrRight > POWER_MAX) pwrRight = POWER_MAX;

//                RConsole.println(pwrLeft + "|" + pwrRight);

                leftMotor.setSpeed(pwrLeft);
                rightMotor.setSpeed(pwrRight);
                leftMotor.forward();
                rightMotor.forward();
            }
//            RConsole.close();
        }

    }

    private void goToStartPosition() {
        goToStartPosition(leftMotor, true, "Left", 1);
        goToStartPosition(rightMotor, false, "Right", 2);
        leftZero = leftMotor.getPosition() - LEFT_LIMIT_TO_ZERO;
        rightZero = rightMotor.getPosition() + RIGHT_LIMIT_TO_ZERO;

        leftTarget = 0;
        rightTarget = 0;
    }

    private void goToStartPosition(NXTRegulatedMotor motor, boolean forward, String name, int display) {
        motor.setSpeed(40);
        motor.resetTachoCount();

        int curPosition = motor.getTachoCount();
        int prevPosition;

        if (forward) {
            motor.forward();
        } else {
            motor.backward();
        }

        boolean done = false;
        while (!done) {
            Delay.msDelay(100);
            prevPosition = curPosition;
            curPosition = motor.getTachoCount();
            if ((forward && !(curPosition > prevPosition)) || (!forward && !(curPosition < prevPosition))) {
                motor.stop();
                done = true;
                LCD.drawString("Done with " + name, 0, display);
            } else {
                LCD.drawString(name + " : " + curPosition, 0, display);
            }
        }
    }
}
