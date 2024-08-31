// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  private final TalonFX m_leftMotor = new TalonFX(0);
  private final TalonFX m_rightMotor = new TalonFX(1);
  private final Follower m_rightFollow = new Follower(0, true);
  private final PositionVoltage m_PositionVoltage = new PositionVoltage(0).withSlot(0).withFeedForward(0);
  
  private DCMotor m_motor = DCMotor.getKrakenX60(2).withReduction(24);
  private LinearSystem<N2,N1,N1>  m_elevatorPlant; 
  private ElevatorSim m_simulator ;
  
  /** Creates a new Elevator. */
  public Elevator() {
    TalonFXConfiguration leftConfig = new TalonFXConfiguration();
    TalonFXConfiguration rightConfig = new TalonFXConfiguration();
    
    leftConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    rightConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    leftConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    leftConfig.Feedback.SensorToMechanismRatio = 24.0;
    leftConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    
    m_leftMotor.getConfigurator().apply(leftConfig);
    m_rightMotor.getConfigurator().apply(rightConfig);
    m_rightMotor.setControl(m_rightFollow);
    m_leftMotor.setPosition(0);
    m_rightMotor.setPosition(0);

    //simulation
    m_elevatorPlant = LinearSystemId.createElevatorSystem(m_motor, 5.0, 0.5, 1.0);
    m_simulator = new ElevatorSim(m_elevatorPlant, m_motor, 0.20, 1.2, true, 0.20);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // In this method, we update our simulation of our elevator.
    // We set the inputs of the elevator plant to the voltages, and update the simulation.
    // We then set the simulated encoder distance and velocity in the elevator object.

  }
}
