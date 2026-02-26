package org.firstinspires.ftc.teamcode.pedroPathing;

public class typeshi {

    static byte[] shootOrder = new byte[] { 1, 1, 2 }; // verde verde violet
    static byte[] loadedState = new byte[] { 1, 2, 1 }; //ce am in robot
    static byte shootingPos = 1;

    public static void main(String[] args) {
        shootAll();
        reload();
        shootAll();
    }

    static void shootAll() {
        for (byte i = 0; i < shootOrder.length; i++) {
            prepareShoot(i);
            shootOne(shootingPos);
        }
    }

    static void prepareShoot(byte shootItem) {
        byte color = shootOrder[shootItem];
        if (loadedState[shootingPos] == color) {
            //ready to shoot
        }
        else
        {
            int maxRotations = loadedState.length;
            int rotationsCount = 0;
            while (loadedState[shootingPos] != color && rotationsCount < maxRotations)
            {
                rotateRight();
                rotationsCount++;
            }
        }
    }

    static void shootOne(int barrelPos) {
        loadedState[barrelPos] = 0;
    }

    static void rotateRight() {
        byte last = loadedState[loadedState.length - 1];
        for (int i = loadedState.length - 1; i > 0; i--) {
            loadedState[i] = loadedState[i - 1];
        }
        loadedState[0] = last;

        try {
            Thread.sleep(1000);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    static void reload() {
        loadedState = new byte[] { 0, 0, 0 };
        loadedState[0] = 2; //GetBallColorCode();
        rotateRight();

        loadedState[0] = getBallColorCode();
        rotateRight();

        loadedState[0] = getBallColorCode();
    }

    static byte getBallColorCode() {
        return 1;
    }
}
