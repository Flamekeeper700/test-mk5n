package frc.robot.util;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class AutomationHelpers {
    public static class ShooterTools {
        InterpolatingDoubleTreeMap shooterTable = new InterpolatingDoubleTreeMap();
        
        public ShooterTools() {
            shooterTable.put(0.0, 0.0);
            shooterTable.put(2.0, 1000.0);
            shooterTable.put(5.0, 2500.0);
            shooterTable.put(10.0, 5000.0);
        }

        public double getRPMFor(double desiredDist) {
            return shooterTable.get(desiredDist);
        }
    }
}
