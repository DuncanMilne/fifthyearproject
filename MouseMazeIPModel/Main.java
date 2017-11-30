import java.awt.Toolkit;

import gurobi.GRB;
import gurobi.GRBException;
import gurobi.GRBModel;
import gurobi.GRBVar;

public class Main {
	public static void main(String[] args) {

		if (args.length==0) {
			System.out.println("Please re-run with a correct set of arguments.");
			listCommands();
			System.exit(1);
		}

		int size = -1;
		int score = -1;
		int timeInMinutes = 0;
		String fileToPrintTo = "";
		String arg;
		for(int i = 0; i < args.length; i+=2) {
			arg = args[i];
			switch (arg) {
				case "h":  
					listCommands();
					i--; // Have placed this here as it is the only command where we do not want to increment i by 2.
					break;
				case "size": 
					try {
						size = Integer.parseInt(args[i+1]);
					} catch (NumberFormatException e) {
						System.out.println("The value after '-size' must be an integer! Run with the flag -h for help.");
					}
					break;
				case "score": 
					try {
						score = Integer.parseInt(args[i+1]);
					} catch (NumberFormatException e) {
						System.out.println("The value after '-score' must be an integer! Run with the flag -h for help.");
					}
					break;
				case "t": 
					try {
						timeInMinutes = Integer.parseInt(args[i+1]);
					} catch (NumberFormatException e) {
						System.out.println("The value after '-t' must be an integer! Run with the flag -h for help.");
					}
					break;
				case "f": 
					fileToPrintTo = args[i+1];
					break;
				default: System.out.println("Invalid flag " + arg);
			}
			

			// If user doesn't set size set it as 1
			if (size == -1) {
				size = 1;
			}
			MouseMazeIPModel mouseMazeIPModel = new MouseMazeIPModel(size, score, timeInMinutes, fileToPrintTo);
			
			try {
//				mouseMazeIPModel.GRBModel.getEnv().set(GRB.IntParam.OutputFlag, 0);
				
				mouseMazeIPModel.GRBModel.set(GRB.IntParam.DisplayInterval, 5);;
				
				mouseMazeIPModel.addConstraints();
				mouseMazeIPModel.setObjectiveFunction();
				long millis1 = System.currentTimeMillis();
				mouseMazeIPModel.GRBModel.optimize();
				long millis2 = System.currentTimeMillis();
				int status = mouseMazeIPModel.GRBModel.get(GRB.IntAttr.Status);

				if (status != GRB.Status.OPTIMAL) {
					Toolkit.getDefaultToolkit().beep();
					System.out.println("no solution found in the following instance, status was: " + status);
//					mouseMazeIPModel.GRBModel.computeIIS();
//					mouseMazeIPModel.GRBModel.write("model.ilp");
//					Toolkit.getDefaultToolkit().beep();
					
				} else {
//					mouseMazeIPModel.utilities.printToFile(mouseMazeIPModel.visits, mouseMazeIPModel.decisions, mouseMazeIPModel.grid);
					mouseMazeIPModel.utilities.printGrid(mouseMazeIPModel.grid);
//					mouseMazeIPModel.printTkMinusOneArray();
					System.out.println("t is " + (int) mouseMazeIPModel.T.getValue());
					System.out.println("took " + (millis2-millis1)/1000 + "." + (millis2 - millis1)%1000 + " seconds");
					if(mouseMazeIPModel.decisions.get(size).get(1).get((int) mouseMazeIPModel.upperBound-1).get(GRB.DoubleAttr.X) != 1) {
						System.out.println("UPPERBOUND NOT LARGE ENOUGH");
					}
					
				}
				mouseMazeIPModel.GRBModel.dispose();
				mouseMazeIPModel.GRBEnv.dispose();
			} catch (GRBException e) {
				e.printStackTrace();
			}
		}
	}

	private static void listCommands() {
		System.out.println("You can use the following flags when running Solver.java");
		System.out.println("-h - lists all commands");
		System.out.println("-size - takes one integer as a parameter representing the size of the rows and columns");
		System.out.println("-score - Takes one parameter representing the score the solver is aiming to beat. If this flag is not included the solver will solve for the optimal solution");
		System.out.println("-t - Takes one parameter specifying the time in minutes that the solver is to run for before termininating and giving the best score found thus far.");
		System.out.println("-f - Takes one string parameter, when the solver completes it's objective the results will be printed to this file.");
	}
}
