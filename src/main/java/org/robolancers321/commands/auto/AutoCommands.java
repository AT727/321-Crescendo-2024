package org.robolancers321.commands.auto;

import org.robolancers321.commands.Intake;
import org.robolancers321.subsystems.intake.Retractor;
import org.robolancers321.subsystems.intake.Sucker;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class AutoCommands {
    public static Command pathAndIntake(Retractor retractor, Sucker sucker, PathPlannerPath path){
        return Commands.sequence(
            retractor.moveToIntake().withTimeout(0.5),
            Commands.race(
                AutoBuilder.followPath(path), new Intake()
            )
        );
        }

    
    public static Command pathAndRetract(Retractor retractor, PathPlannerPath path){
        return Commands.parallel(
            AutoBuilder.followPath(path),
            retractor.moveToRetracted()
        );
        }

    
    }

