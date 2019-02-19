// /*----------------------------------------------------------------------------*/
// /* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
// /* Open Source Software - may be modified and shared by FRC teams. The code   */
// /* must be accompanied by the FIRST BSD license file in the root directory of */
// /* the project.                                                               */
// /*----------------------------------------------------------------------------*/

// package frc.robot.commands.combos;

// import edu.wpi.first.wpilibj.command.CommandGroup;
// import frc.robot.Robot;
// import frc.robot.commands.elevator.ElevatorMotionMagic;
// import frc.robot.commands.intakeArm.IntakeArmPosition;
// import frc.robot.subsystems.Elevator;
// import frc.robot.subsystems.IntakeArm;

// public class ElevatorArm extends CommandGroup {

//   /**
//    * Add your docs here.
//    */
//   public ElevatorArm(double targetElevatorPosition, double targetIntakePosition) {
    
//     double targetIntakePositionDouble = targetIntakePosition;

//     if(!Robot.intakeArm.isArmSafe(IntakeArm.Angle.SAFE)) {
//       System.out.println("Notsafe");
//       addSequential(new IntakeArmPosition(IntakeArm.Angle.SAFE));
//     }

//     System.out.println("Safe");


//     addSequential(new ElevatorMotionMagic(targetElevatorPosition));

//     if(targetElevatorPosition <= Elevator.CONFLICT_LOWER_BOUND) {
//       if (targetIntakePositionDouble < IntakeArm.Angle.INSIDE) {
//         targetIntakePositionDouble = IntakeArm.Angle.INSIDE;
//       }
//     } else if (targetElevatorPosition <= Elevator.CONFLICT_UPPER_BOUND) {
//       if (targetIntakePositionDouble < IntakeArm.Angle.SAFE) {
//         targetIntakePositionDouble = IntakeArm.Angle.SAFE;
//       }
//     }

//     addSequential(new IntakeArmPosition(targetIntakePositionDouble));

//     //   // UNDER CONFLICT ZONE
//     //   if (Robot.elevator.getCurrentHeight() <= Elevator.CONFLICT_LOWER_BOUND) {
//     //     //GO UNDER CONFLICT ZONE
//     //     if (targetElevatorPosition <= Elevator.CONFLICT_LOWER_BOUND) {
          
//     //       if (targetIntakePosition.getValue() >= IntakeArm.Angle.INSIDE) {
//     //         addParallel(new ElevatorMotionMagic(targetElevato(Position));
//     //         addSequential(new IntakeArmPosition(targetIntakePosition.getValue()));
//     //       } else {
//     //         addSequential(new ElevatorMotionMagic(targetElevatorPosition));
//     //       }
//     //     //GO IN CONFLICT ZONE
//     //     } else if(targetElevatorPosition <= Elevator.CONFLICT_UPPER_BOUND) {
//     //       if (targetIntakePosition.getValue() >= IntakeArm.Angle.SAFE) {
//     //         addSequential(new IntakeArmPosition(targetIntakePosition.getValue()));
//     //         addSequential(new ElevatorMotionMagic(Elevator.SAFE_POSITION));
//     //         addSequential(new ElevatorMotionMagic(targetElevatorPosition));
//     //       } else {
//     //         addParallel(new IntakeArmPosition(IntakeArm.Position.INTAKE));
//     //         addSequential(new ElevatorMotionMagic(Elevator.SAFE_POSITION));
//     //         addSequential(new ElevatorMotionMagic(targetElevatorPosition));
//     //       }
//     //     // GO ABOVE CONFLICT ZONE
//     //     } else {
//     //       addParallel(new IntakeArmPosition(IntakeArm.Position.INTAKE));
//     //       addSequential(new ElevatorMotionMagic(Elevator.SAFE_POSITION));
//     //       addSequential(new ElevatorMotionMagic(targetElevatorPosition));
//     //       addSequential(new IntakeArmPosition(targetIntakePosition.getValue()));
//     //     }
//     //   // IN CONFLICT ZONE
//     //   } else if (Robot.elevator.getCurrentHeight() <= Elevator.CONFLICT_UPPER_BOUND) {
//     //     if (targetElevatorPosition <= Elevator.CONFLICT_LOWER_BOUND) {
//     //       addSequential(new ElevatorMotionMagic(targetElevatorPosition));
//     //       if (targetIntakePosition.getValue() >= IntakeArm.Position.INSIDE) {
//     //         addSequential(new IntakeArmPosition(targetIntakePosition.getValue()));
//     //       }
//     //     } else if(targetElevatorPosition <= Elevator.CONFLICT_UPPER_BOUND) {
//     //       addParallel(new IntakeArmPosition(IntakeArm.Position.INTAKE));
//     //       addSequential(new ElevatorMotionMagic(Elevator.SAFE_POSITION));
//     //       addSequential(new ElevatorMotionMagic(targetElevatorPosition));
//     //       if (targetIntakePosition.getValue() >= IntakeArm.Position.SAFE) {
//     //         addSequential(new IntakeArmPosition(targetIntakePosition.getValue()));
//     //       }
//     //     } else {
//     //       addParallel(new IntakeArmPosition(IntakeArm.Position.INTAKE));
//     //       addSequential(new ElevatorMotionMagic(Elevator.SAFE_POSITION));
//     //       addSequential(new ElevatorMotionMagic(targetElevatorPosition));
//     //       addSequential(new IntakeArmPosition(targetIntakePosition.getValue()));
//     //     }
//     //   // ABOVE CONFLICT ZONE
//     //   } else {
//     //     if (targetElevatorPosition <= Elevator.CONFLICT_LOWER_BOUND) {

//     //     } else if(targetElevatorPosition <= Elevator.CONFLICT_UPPER_BOUND) {
//     //       addParallel(new IntakeArmPosition(IntakeArm.Position.INTAKE.getValue()));
//     //       addSequential(new ElevatorMotionMagic(Elevator.SAFE_POSITION));
//     //       addSequential(new ElevatorMotionMagic(targetElevatorPosition));
//     //     } else {
//     //       addSequential(new ElevatorMotionMagic(targetElevatorPosition));        
//     //     }
//     //   }
//     // }

//   }
// }
