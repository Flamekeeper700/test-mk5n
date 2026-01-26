package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class GridDistanceProcessing {

    private byte[] map;        // heatmap (cost field)
    private double[] angles;   // FLOW FIELD angles (radians, field coords)

    private int mapLength;     // X cells
    private int mapWidth;      // Y cells

    private double fieldLength = 16.6;
    private double fieldWidth  = 8.1;

    private double lengthModifier;
    private double widthModifier;

    private static final int LOOKAHEAD_STEPS = 6;   // follow vector several cells
    private static final double STEP_FRACTION = 0.7; // fraction of cell per step

    public GridDistanceProcessing(byte[] InputMap, double[] InputAngles) {

        /* ---------------- HEAT MAP ---------------- */
        map = InputMap;
        angles = InputAngles;
        mapLength = 66;
        mapWidth  = 32;

        lengthModifier = fieldLength / mapLength;
        widthModifier  = fieldWidth  / mapWidth;
    }

    /* ---------------- Coordinate Conversion ---------------- */

    private int indexAt(Pose2d pose) {
        return indexAt(pose.getX(), pose.getY());
    }

    private int indexAt(double xMeters, double yMeters) {

        if (xMeters < 0 || yMeters < 0 ||
            xMeters >= fieldLength || yMeters >= fieldWidth) {
            return -1;
        }

        int gx = (int) (xMeters / lengthModifier);
        int gy = (int) (yMeters / widthModifier);

        if (gx < 0 || gx >= mapLength || gy < 0 || gy >= mapWidth) {
            return -1;
        }

        return gx + gy * mapLength;
    }

    private int xAt(int idx) { return idx % mapLength; }
    private int yAt(int idx) { return idx / mapLength; }

    private byte valueAt(int idx) {
        if (idx < 0 || idx >= map.length) return -1;
        return map[idx];
    }

    private double angleAt(int idx){
        if(idx < 0 || idx >= angles.length) return 0.0;
        return angles[idx];
    }

    /* ---------------- Flow Field Follow ---------------- */

    public Pose2d bestAdjacent(Pose2d originalPose) {

        double x = originalPose.getX();
        double y = originalPose.getY();

        double stepX = lengthModifier * STEP_FRACTION;
        double stepY = widthModifier  * STEP_FRACTION;

        for(int i=0;i<LOOKAHEAD_STEPS;i++){

            int idx = indexAt(x,y);
            if(idx < 0) break;

            if(valueAt(idx) == -1) break; // wall

            double ang = angleAt(idx);

            x += Math.cos(ang) * stepX;
            y += Math.sin(ang) * stepY;
        }

        return new Pose2d(x, y, originalPose.getRotation());
    }

    /* ---------------- Utility ---------------- */

    public int heatAt(Pose2d pose) {
        int idx = indexAt(pose);
        if (idx < 0) return -1;
        return valueAt(idx);
    }

    public Rotation2d flowDirection(Pose2d pose){
        int idx = indexAt(pose);
        if(idx < 0) return new Rotation2d();
        return new Rotation2d(angleAt(idx));
    }
}
