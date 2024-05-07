//CB-9 "Vengence" Remaster

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//import javax.lang.model.util.ElementScanner14;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.cameraserver.CameraServer;
//import edu.wpi.first.math.controller.PIDController;
//import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String NoAutoSelected = "No Auto Selected";
  private static final String ShootDontMove = "Shoot and Don't Move";
  private static final String ShootCrossLine = "Shoot and Cross Line ";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */

  //Motors
  private final PWMVictorSPX LeftMotor = new PWMVictorSPX(1);
  private final PWMVictorSPX RightMotor = new PWMVictorSPX(2);
  //private final PWMVictorSPX frontLeftMotor = new PWMVictorSPX(1);
  //private final PWMVictorSPX backLeftMotor = new PWMVictorSPX(7);
  //private final PWMVictorSPX frontRightMotor = new PWMVictorSPX(2);
  //private final PWMVictorSPX backRightMotor = new PWMVictorSPX(8); 
  private final PWMVictorSPX pivotMotor = new PWMVictorSPX(3);
  private final PWMVictorSPX intakeMotor = new PWMVictorSPX(4);
  private final PWMVictorSPX stagingMotor = new PWMVictorSPX(5);
  private final PWMVictorSPX exitMotor = new PWMVictorSPX(6);
  
  private final TalonFX leftshootMotor = new TalonFX(1);
  private final TalonFX rightshootMotor = new TalonFX(2);
  

  private DifferentialDrive m_drive;

  //Controls
  private final Joystick rightstick = new Joystick(0);
  private final Joystick leftstick = new Joystick(1);
  private final XboxController xbox = new XboxController(2);

  private final Timer autoTimer = new Timer();

  //FUNCTIONS
  //PIVOT
  public void setPivotSpeed(double speed){
    pivotMotor.set(speed);
  }
  public void stopPivot(){
    pivotMotor.set(0);
  }

  //INTAKE
  public void setIntakeSpeed(double speed){
    intakeMotor.set(speed);
  }
  public void stopIntake(){
    intakeMotor.set(0);
  }

  //STAGING
  public void setStagingSpeed(double speed){
    stagingMotor.set(speed);
  }
  public void stopStaging(){
    stagingMotor.set(0);
  }

  //SHOOTER
  public void setShooterSpeed(double speed){
    leftshootMotor.set(speed);
  }
  public void stopShooter(){
    leftshootMotor.set(0);
  }

  //SHOOTER EXIT
  public void setExitSpeed(double speed){
    exitMotor.set(speed);  
  }
  public void stopExit(){
    exitMotor.set(0);
  }

  public void ShootAndCrossLine(){
    if (autoTimer.get() > 9){
      m_drive.tankDrive(0, 0);
    } else if (autoTimer.get() > 5){
      stopShooter();
      stopStaging();
      stopExit();
      m_drive.tankDrive(.6, .6);
    } else if (autoTimer.get() > 2){
      setStagingSpeed(.5);
    } else if (autoTimer.get() > 0){
      setShooterSpeed(.5);
      setExitSpeed(.75);
    }
   }

  public void ShootAndDoNotMove(){
    if (autoTimer.get() > 5){
      stopShooter();
      stopStaging();
      stopExit();
    } else if (autoTimer.get() > 2){
      setStagingSpeed(.5);
    } else if (autoTimer.get() > 0){
      setShooterSpeed(.9);
      setExitSpeed(.75);
    }
  }

  @Override
  public void robotInit() {
    m_chooser.setDefaultOption(NoAutoSelected, NoAutoSelected);
    //m_chooser.addOption(ShootDontMove, ShootDontMove);
    //m_chooser.addOption(ShootCrossLine, ShootCrossLine);
    SmartDashboard.putData("Auto Chooser", m_chooser);
    //Camera
    CameraServer.startAutomaticCapture();

    //DRIVE
    //frontRightMotor.setInverted(true);
    //frontLeftMotor.addFollower(backLeftMotor);
    //frontRightMotor.addFollower(backRightMotor);
    //m_drive = new DifferentialDrive(frontLeftMotor, frontRightMotor);

    LeftMotor.setInverted(true);
    m_drive = new DifferentialDrive(LeftMotor, RightMotor);

    //PIVOT
    pivotMotor.setInverted(true);

    //INTAKE
    intakeMotor.setInverted(true);

    //STAGING
    stagingMotor.setInverted(false);

    //SHOOTER
    leftshootMotor.setInverted(false);
    rightshootMotor.setControl(new Follower(1, true));

    //EXIT SHOOTER
    exitMotor.setInverted(true);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    System.out.println("Auto selected: " + m_autoSelected);

    autoTimer.reset();
    autoTimer.start();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case ShootDontMove:
        ShootAndDoNotMove();
        break;
      case ShootCrossLine:
        ShootAndCrossLine();
        break;
      case NoAutoSelected:
        //System.out.println("buwomp");
        break;
      default:
        break;
      
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    autoTimer.stop();
    autoTimer.reset();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    //DRIVE
    m_drive.arcadeDrive(leftstick.getY(), -rightstick.getX());

    //PIVOT
    if (xbox.getYButton()){
    setPivotSpeed(xbox.getLeftY()*.45);
    } else {
      stopPivot();
    }

    //INTAKE
    if (xbox.getRightBumper()){
      setIntakeSpeed(.5);
    } else if (xbox.getLeftBumper()){
      setIntakeSpeed(-.5);
    } else {
      stopIntake();
    }

    //STAGING
    if (xbox.getAButton()){
      setShooterSpeed(.5);
    } else if (xbox.getBButton()){
      setShooterSpeed(-.4);
    }  else {
      stopStaging();
      }

    //SHOOTER
    if (xbox.getXButton()){
      setShooterSpeed(.50);
    } else if (xbox.getYButton()){
      setShooterSpeed(-.3);
    } else if (xbox.getStartButton()){
      setShooterSpeed(.90);
    } else if (xbox.getBackButton()){
      setShooterSpeed(.70);
    } else {
     stopShooter();
     stopShooter();
    }

    //EXIT SHOOTER
    if (xbox.getXButton()){
      setExitSpeed(.75);
    } else {
     stopExit();
    }
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}