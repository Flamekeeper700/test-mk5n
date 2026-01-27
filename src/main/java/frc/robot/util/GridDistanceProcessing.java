package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import org.littletonrobotics.junction.Logger;

public class GridDistanceProcessing {

    private final byte[] heatMap;
    private final double[] flowX;
    private final double[] flowY;

    private final int mapLength;
    private final int mapWidth;

    private final double fieldLength = 16.6;
    private final double fieldWidth  = 8.1;

    private final double lengthModifier;
    private final double widthModifier;

    /* ================= Tunables ================= */

    private static final int LOOKAHEAD_STEPS = 5;
    private static final double STEP_FRACTION = 0.8;

    private static final int WALL_SAMPLE_RADIUS = 2;     // cells
    private static final double WALL_REPEL_GAIN = 2.5;   // steering strength
    private static final double WALL_REPEL_EXP  = 1.5;   // stronger when closer

    /* ============================================ */

    public GridDistanceProcessing(
            byte[] heatMap,
            double[] flowX,
            double[] flowY,
            int mapLength,
            int mapWidth) {

        this.heatMap = heatMap;
        this.flowX = flowX;
        this.flowY = flowY;

        this.mapLength = mapLength;
        this.mapWidth  = mapWidth;

        lengthModifier = fieldLength / mapLength;
        widthModifier  = fieldWidth  / mapWidth;

        Logger.recordOutput("Pathing/FlowField", buildFlowPoseField());
    }

    /* ================= Coordinate Conversion ================= */

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

    private byte heatAtIndex(int idx) {
        if (idx < 0 || idx >= heatMap.length) return -1;
        return heatMap[idx];
    }

    /* ================= Public API ================= */

    public int heatAt(Pose2d pose) {
        int idx = indexAt(pose.getX(), pose.getY());
        if (idx < 0) return -1;
        return heatAtIndex(idx);
    }

    /**
     * Flow vector with active wall avoidance.
     */
    public double[] flowVector(Pose2d pose) {

        double x = pose.getX();
        double y = pose.getY();

        double vx = 0;
        double vy = 0;
        int samples = 0;

        double stepX = lengthModifier * STEP_FRACTION;
        double stepY = widthModifier  * STEP_FRACTION;

        /* ---- lookahead integration of flow field ---- */

        for (int i = 0; i < LOOKAHEAD_STEPS; i++) {

            int idx = indexAt(x, y);
            if (idx < 0) break;
            if (heatAtIndex(idx) < 0) break;

            double fx = flowX[idx];
            double fy = flowY[idx];

            vx += fx;
            vy += fy;
            samples++;

            x += fx * stepX;
            y += fy * stepY;
        }

        if (samples > 0) {
            vx /= samples;
            vy /= samples;
        }

        /* ---- wall repulsion ---- */

        int cx = (int) (pose.getX() / lengthModifier);
        int cy = (int) (pose.getY() / widthModifier);

        double rx = 0;
        double ry = 0;

        for (int dy = -WALL_SAMPLE_RADIUS; dy <= WALL_SAMPLE_RADIUS; dy++) {
            for (int dx = -WALL_SAMPLE_RADIUS; dx <= WALL_SAMPLE_RADIUS; dx++) {

                if (dx == 0 && dy == 0) continue;

                int nx = cx + dx;
                int ny = cy + dy;

                if (nx < 0 || ny < 0 || nx >= mapLength || ny >= mapWidth)
                    continue;

                int ni = nx + ny * mapLength;
                if (heatMap[ni] >= 0) continue; // not wall

                double dist = Math.hypot(dx, dy);
                if (dist < 1e-3) continue;

                double w = Math.pow(
                        (WALL_SAMPLE_RADIUS + 1 - dist) / (WALL_SAMPLE_RADIUS + 1),
                        WALL_REPEL_EXP);

                rx -= dx / dist * w;
                ry -= dy / dist * w;
            }
        }

        vx += rx * WALL_REPEL_GAIN;
        vy += ry * WALL_REPEL_GAIN;

        /* ---- normalize ---- */

        double mag = Math.hypot(vx, vy);
        if (mag > 1e-6) {
            vx /= mag;
            vy /= mag;
        }

        return new double[]{vx, vy};
    }

    /**
     * Builds a Pose2d for every grid cell showing flow direction.
     * Pose is centered in the cell, rotation matches vector direction.
     */
    public Pose2d[] buildFlowPoseField() {

        Pose2d[] poses = new Pose2d[mapLength * mapWidth];

        double halfX = lengthModifier * 0.5;
        double halfY = widthModifier  * 0.5;

        for (int y = 0; y < mapWidth; y++) {
            for (int x = 0; x < mapLength; x++) {

                double fieldX = x * lengthModifier + halfX;
                double fieldY = y * widthModifier  + halfY;

                Pose2d samplePose = new Pose2d(fieldX, fieldY, new Rotation2d());

                double[] v = flowVector(samplePose);

                double angle = Math.atan2(v[1], v[0]);

                poses[x + y * mapLength] =
                        new Pose2d(fieldX, fieldY, new Rotation2d(angle));
            }
        }

        return poses;
    }

}
