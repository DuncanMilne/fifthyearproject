import java.io.File;
import java.io.FileNotFoundException;
import java.util.Scanner;
import java.util.ArrayList;
import java.util.List;
import java.lang.ArrayIndexOutOfBoundsException;


// This verifier will take a file supplied by the user in a given format, this file will outline the maze configuration.
// The verifier will also take as an argument the suspected score the maze should give.
public class Verifier {

	// Takes two arguments, firstly a file containing the maze configuration in X format, secondly, the score
	// produced by the IP model
	public static void main(String[] args) {

		// Parse file and create maze
		File mazeFile = new File(System.getProperty("user.dir") + "/" + args[0]);

		Maze maze = new Maze(mazeFile);

		int score = 0;

		try {
			score = Integer.parseInt(args[1]);
		} catch (NumberFormatException e){
			System.out.println("Please enter an integer as your second argument");
			System.exit(0);
		} catch (ArrayIndexOutOfBoundsException e){
			System.out.println("Please ensure you enter two arguments, the second being an integer representing the score to beat.");
			System.exit(0);
		}

		maze.simulate(score);
	}


}

