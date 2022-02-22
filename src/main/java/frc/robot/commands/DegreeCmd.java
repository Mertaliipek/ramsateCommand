package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class DegreeCmd extends CommandBase {
    private final DriveSubsystem driveSubsystem;
    private final double KP;
    private final int Degree;

    public DegreeCmd(DriveSubsystem driveSubsystem,double KP,int Degree) {
        this.driveSubsystem = driveSubsystem;
        this.KP = KP;
        this.Degree = Degree;
        addRequirements(driveSubsystem);
    }
    
    
    @Override
    public void initialize() {
        System.out.println("Degree started!");
    }

    @Override
    public void execute() {
        driveSubsystem.setDegree(KP,Degree);
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.setDegree(0,0);
        driveSubsystem.setDegree(0, 0);
        System.out.println("Degree ended!");
    }

    @Override
    public boolean isFinished() {
        return false;
    }


    
}
