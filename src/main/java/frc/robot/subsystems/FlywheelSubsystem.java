// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//https://github.com/wpilibsuite/allwpilib/blob/main/wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/flywheelbangbangcontroller/Robot.java
//https://www.chiefdelphi.com/t/rev-sparkmax-simulation/399889/4

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FlywheelSubsystem extends SubsystemBase {

  private CANSparkMax m_flywheelLeft = new CANSparkMax(1, MotorType.kBrushless);
  private CANSparkMax m_flywheelRight = new CANSparkMax(2, MotorType.kBrushless);


  // 1/2*M*R^2
  private static final double kFlywheelMomentOfInertia =
          0.5 * Units.lbsToKilograms(1.5) * Math.pow(Units.inchesToMeters(4), 2);
  private FlywheelSim m_FlywheelSim = new FlywheelSim(DCMotor.getNEO(2), 
                                                    1, 
                                                    kFlywheelMomentOfInertia);
  private double voltage = 0;                                                                         
  

  /** Creates a new FlywheelSubsystem. */
  public FlywheelSubsystem() {
    m_flywheelLeft.setIdleMode(CANSparkBase.IdleMode.kCoast);
    m_flywheelRight.setIdleMode(CANSparkBase.IdleMode.kCoast);

    m_flywheelRight.follow(m_flywheelLeft, true);
  }

  public void start() {
    voltage = 9;
    m_flywheelLeft.setVoltage(voltage);
  }

  public void stop() {
    voltage = 0;
    m_flywheelLeft.setVoltage(voltage);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("leftsparkrpm", m_flywheelLeft.getEncoder().getVelocity());
    SmartDashboard.putNumber("rightsparkrpm", m_flywheelLeft.getEncoder().getVelocity());    
    SmartDashboard.putNumber("leftPos", m_flywheelLeft.getEncoder().getPosition());
  } 

  public void simulationInit() {
    REVPhysicsSim.getInstance().addSparkMax(m_flywheelLeft, DCMotor.getNEO(1));
    REVPhysicsSim.getInstance().addSparkMax(m_flywheelRight, DCMotor.getNEO(1));
  }

  public void simulationPeriodic() {
    m_FlywheelSim.setInputVoltage(voltage);
    m_FlywheelSim.update(.02);
  
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(m_FlywheelSim.getCurrentDrawAmps()));
    
    double simRadPerSec = m_FlywheelSim.getAngularVelocityRadPerSec();
    double simRadPerMinute = simRadPerSec * 60;
    double simDegPerMinute = Math.toDegrees(simRadPerMinute);
    double simRotPerMinute = simDegPerMinute/360;
    SmartDashboard.putNumber("flywheelRpm", simRotPerMinute);
    SmartDashboard.putNumber("currentDraw", m_FlywheelSim.getCurrentDrawAmps());            
  }
}
