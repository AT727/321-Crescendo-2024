package org.robolancers321.commands.auto;

import static org.robolancers321.commands.auto.Question.poseQuestion;

import java.util.Set;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.robolancers321.commands.auto.Response.FirstNote;
import org.robolancers321.commands.auto.Response.StartLocation;
import org.robolancers321.commands.auto.Response.YesNo;
import org.robolancers321.subsystems.drivetrain.Drivetrain;
import org.robolancers321.subsystems.intake.Retractor;
import org.robolancers321.subsystems.intake.Sucker;
import org.robolancers321.subsystems.launcher.Flywheel;
import org.robolancers321.subsystems.launcher.Indexer;
import org.robolancers321.subsystems.launcher.Pivot;

/*
   * auto:
   * - a segment (command)
   * - another segment
   * - question-based segment <- supplied by the questionnaire
   * - a segment (command)
   * - another segment
   * - question-based segment <- supplied by the questionnaire
   * 
   * questionnaire --> making the auto command
   * question -> provides mapping answer to command
   * 
   * plugging in
   * -  deferredcommand
   * 
   * 
   * 
   */
public class AutoDealer {
    public interface AutoRoutine {
        Supplier<Command> init();
    }

    private Supplier<Command> leDeal;

    //pre-made playstyles / unique questions
    public static final AutoRoutine kOffense = () -> {
        final var startLocation = poseQuestion("What's the starting location", StartLocation.MID, (StartLocation choice) -> switch (choice){
            case AMP -> Commands.none();
            case MID -> Commands.none();
            case SOURCE -> Commands.none();
        });

        final var firstNote = poseQuestion("What's the first note to intake?", FirstNote.SUBWOOFER, (FirstNote choice) -> switch (choice){
            case CENTER1 -> Commands.none();
            case CENTER2 -> Commands.none();
            case SUBWOOFER -> Commands.none();
        });
        
        return () -> Commands.sequence(
            startLocation.resolve(),
            firstNote.resolve()
        );
    };

    // public static final AutoRoutine kDefense = () -> {
    //     final var firstNote = poseQuestion("What's the first note?", FirstNote.TOP, (FirstNote choice) -> switch (choice) {
    //         case TOP -> Commands.print("this is a top auto");
    //         case MID -> Commands.none();
    //         case BOT -> Commands.print("hi");
    //     });

    //     return () -> Commands.sequence(
    //         Commands.waitSeconds(2),
    //         firstNote.resolve(),
    //         Commands.waitSeconds(1)
    //     );
    // };
        
    /**
     * Presents auto with questionnaire
     * @param autoStyle supplier that returns a sequence of commands with questions to resolve
     * @return a command that resolves at runtime with the question responses, requiring all subsystems (since it's auto)
     */
    public void request(AutoRoutine routine) {
        leDeal = routine.init();
    }

    public Command getAutoRoutine(){
        return Commands.defer(leDeal, Set.of(
            Drivetrain.getInstance(), Retractor.getInstance(), Sucker.getInstance(), Indexer.getInstance(), Flywheel.getInstance(), Pivot.getInstance()
        )); 
    }

    
}