package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class BallsCmd extends CommandBase {
    private final DriveSubsystem driveSubsystem;
    private final double ballTake;
    private final double ballThrow;

    public BallsCmd(DriveSubsystem driveSubsystem,double ballTake,double ballThrow) {
        this.driveSubsystem = driveSubsystem;
        this.ballThrow = ballThrow;
        this.ballTake = ballTake;
        addRequirements(driveSubsystem);
    }
    
    
    @Override
    public void initialize() {
        System.out.println("Balls started!");
    }

    @Override
    public void execute() {
    driveSubsystem.TakeBall(ballTake);
    driveSubsystem.ThrowBall(ballThrow);
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.TakeBall(0);
        driveSubsystem.ThrowBall(0);
        System.out.println("Balls ended!");
    }

    @Override
    public boolean isFinished() {
        return false;
    }


    
}
