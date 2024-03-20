package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LimelightConstants;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

// import frc.robot.Constants.DriveConstants;

public class LimelightSubsystem extends SubsystemBase {

  private static LimelightSubsystem instance = null;

  public ShuffleboardTab limelightTab = Shuffleboard.getTab("limelight");
  public ShuffleboardTab swerveTab = Shuffleboard.getTab("Swerve Subsystem");

  //Declaring objects that are used for retrieving data from the limelight.

  private GenericEntry txEntry = limelightTab.add("tx", 0).getEntry();
  private GenericEntry txEntry2 = swerveTab.add("tx", 0).withPosition(6, 0).getEntry();
  private GenericEntry tyEntry = limelightTab.add("ty", 0).getEntry();
  private GenericEntry taEntry = limelightTab.add("ta", 0).getEntry();
  private GenericEntry tvEntry = limelightTab.add("tv", 0).getEntry();
  private GenericEntry distanceEntry = limelightTab.add("distance", 0).getEntry();

  private static NetworkTable table;
  private static NetworkTableEntry tx, ty, tv, ta, priorityid;
  private static NetworkTableEntry camMode;
  private static NetworkTableEntry ledMode;

  private static double x, y, v, a;

  private LimelightSubsystem ()
  {
    table = NetworkTableInstance.getDefault().getTable("limelight");
    
    System.out.println("tx: " + x + "   ty: " + y + "   tv: " + v);

    tx = table.getEntry("tx"); // Horizontal offset from crosshair to target (-29.8 to 29.8 degrees).
    ty = table.getEntry("ty"); // Vertical offset from crosshair to target (-24.85 to 24.85 degrees).
    tv = table.getEntry("tv"); // Whether the limelight has any valid targets (0 or 1).
    ta = table.getEntry("ta"); // Target area (0% of image to 100% of image).
    priorityid = table.getEntry("priorityid"); // Preffered id of the april tag

    ledMode = table.getEntry("ledMode"); // limelight's LED state (0-3).
    camMode = table.getEntry("camMode"); // limelight's operation mode (0-1).

  }

  @Override 
  public void periodic() {
    x = tx.getDouble(0.0);
    y = ty.getDouble(0.0);
    v = tv.getDouble(0.0);
    a = ta.getDouble(0.0);

    txEntry.setDouble(x);
    txEntry2.setDouble(x);  
    tyEntry.setDouble(y);
    tvEntry.setDouble(v);
    taEntry.setDouble(a);
    distanceEntry.setDouble(getDistance());


     // limelightTab.add("tx", x);
    // limelightTab.add("ty", y);
    // limelightTab.add("tv", v);
    // limelightTab.add("ta", a);
   
  }

  public void setPrefferedID(int id) {
    priorityid.setDouble(id);
  }

  /**
   * Horizontal offset from crosshair to target.
   * @return offset from -29.8 to 29.8 degrees.
   */
  public double getTargetOffsetX()
  {
    return tx.getDouble(0.0);
  }

  /**
   * Vertical offset from crosshair to target.
   * @return offset from -24.85 to 24.85 degrees.
   */
  public double getTargetOffsetY()
  {
    return ty.getDouble(0.0);
  }

  /**
   * Get whether or not a target is detected.
   * @return true if target is found and false if target is not found.
   */
  public boolean isTargetAvailable()
  {
    return tv.getNumber(0).intValue() == 1 ? true : false;
  }

  /**
   * Get area of detected target.                                                                                                                        
   * @return target area from 0% to 100%.
   */
  public double getTargetArea()
  {
    return ta.getDouble(0.0);
  }

  public double getDistance () {
    double angleToGoalDegrees = LimelightConstants.kLimelightMountAngleDegrees + y;
    double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

    // calculate distance
    return (LimelightConstants.kHeightOfTagInches - LimelightConstants.kLimelightLensHeightInches) / Math.tan(angleToGoalRadians);
  }

  /**
   * Method for other classes to use this class' methods.
   * @return the limelight instance object.
   */
  public static LimelightSubsystem getInstance()
  {
    if (instance == null)
      instance = new LimelightSubsystem();
    
    return instance;
  }
}
