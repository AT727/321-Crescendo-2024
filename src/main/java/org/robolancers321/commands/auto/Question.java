package org.robolancers321.commands.auto;

import java.util.function.Function;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class Question {
    public interface FutureCommand {
        Command resolve();
    }
    
    public static <T extends Enum<T> & Response> FutureCommand poseQuestion(String question, T defaultChoice, Function<T, Command> getCommand) {
        // set up publisher / logging using `question`
        var chooser = new SendableChooser<T>();

        chooser.setDefaultOption(defaultChoice.toString(), defaultChoice);

        // getting all the enum options (it's a bit cursed)
        for (var choice : defaultChoice.getDeclaringClass().getEnumConstants()) {
            chooser.addOption(choice.name(), choice);
        }

        SmartDashboard.putData(question, chooser);

        return () -> getCommand.apply(chooser.getSelected());
    }
}