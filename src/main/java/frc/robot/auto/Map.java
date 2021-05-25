package frc.robot.auto;

public class Map {
    
    // Rotations are field relative, not relative to robot heading, 
    // thus 0 degrees is downfield from start position (we'll call eastward).
    // 90 is Northward, -90 is Southward, 180 is Westward.
    // Start with the fixed nav points, then empirically tune by 
    // adding/subtracting inhes/degrees as necessary.

    final static double kEastward = 0.0;
    final static double kWesthward = 180.0;
    final static double kSouthward = -90.0;
    final static double kNorthward = 90.0;

    final static double kNavPointInches = 30.0;
    final static double kDiag45Inches = Math.sqrt(Math.pow(Map.kNavPointInches,2.0) * 2.0);
}
