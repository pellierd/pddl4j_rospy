import fr.uga.pddl4j.planners.hsp.*;

import java.io.FileOutputStream;
import java.io.IOException;
import java.io.OutputStreamWriter;
import java.io.Writer;


public class Main {
		
	public static String HOME_PATH = "./";
	public static String DEFAULT_PATH = HOME_PATH + "src/jsonFiles/JSONPlan.json";
	
	public static void main(String[] args) {
				
		String[] argsPlanner = new String[4];
		int i = 0;
		while(i < args.length && !args[i].equals("-json")){
			argsPlanner[i] = args[i];
			i += 1;
		}
		
		String jsonPlan = HSP.resolveAsJsonPlan(argsPlanner);
		
		String jsonPath = "";		
		try {
			jsonPath = args[i + 1];
		} catch (IndexOutOfBoundsException exception){
			jsonPath = DEFAULT_PATH;
			exception.printStackTrace();
		}
				
		// Creation of the json files
        try (Writer writer = new OutputStreamWriter(new FileOutputStream(jsonPath), "UTF-8")) {
            // Editing the first json file
            writer.write(jsonPlan);
        } catch (IOException exception) {
            exception.printStackTrace();
        }
	}
}

/** TODO ADAPTER LA CLASSE SUIVANTE
package fr.uga.pddl4j.test.planner;

import fr.uga.pddl4j.encoding.CodedProblem;
import fr.uga.pddl4j.parser.ErrorManager;
import fr.uga.pddl4j.planners.Planner;
import fr.uga.pddl4j.planners.PlannerFactory;
import fr.uga.pddl4j.planners.ProblemFactory;
import fr.uga.pddl4j.encoding.AdapterJavaJson;
import fr.uga.pddl4j.util.Plan;

import java.io.IOException;

/**
 * This class test the planner interface.
 *
 *
 * @author Damien Pellier
 * @version 0.1 - 21.06.17
 */

/*
public class PlannerTest {


    public static void main(String[] args) throws IOException {


        // Creates the problem factory
        final ProblemFactory factory = ProblemFactory.getInstance();

        // Parses the PDDL domain and problem description
        ErrorManager errorManager = factory.parse(args[0], args[1]);
        if (!errorManager.isEmpty()) {
            errorManager.printAll();
            System.exit(0);
        }

        final CodedProblem pb = factory.encode();

        if (!pb.isSolvable()) {
            StringBuilder strb = new StringBuilder();
            strb.append(String.format("goal can be simplified to FALSE. no search will solve it%n%n"));
            System.out.println(strb);
            System.exit(0);
        }

        // Create the planner HSP can be a parameter of tje command line
        Planner hsp = PlannerFactory.getInstance().getPlanner(Planner.Name.HSP);
        // Searches for a solution plan
        final Plan plan = hsp.search(pb);
        AdapterJavaJson adapter = new AdapterJavaJson(pb);
        System.out.println(adapter.toJsonString(plan));


    }

}
*/
