package frc.robot;

public class Drivetrain {
    private double[] motorSpeeds = new double[2]; // left and right

    public void arcadeDrive(double forward, double turn) {
        System.out.println("arcadeDrive() called with forward=" + forward + ", turn=" + turn);

        double left = forward + turn;
        double right = forward - turn;

        motorSpeeds[0] = left;
        motorSpeeds[1] = right;

        motorSpeeds[2] = (left + right) / 2.0;

        System.out.println("Motor speeds set to: L=" + left + ", R=" + right);
    }
}
