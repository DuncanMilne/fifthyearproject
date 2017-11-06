import java.io.File;
import java.io.FileNotFoundException;
import java.util.Scanner;
import java.util.ArrayList;
import java.util.List;
import java.lang.ArrayIndexOutOfBoundsException;

// This verifier will take a file supplied by the user in a given format, this file will outline the maze configuration.
// The verifier will also take as an argument the suspected score the maze should give.
public class Verifier {

	private static File mazeFile;
	private static ArrayList<ArrayList<Integer>> maze;
	// Takes two arguments, firstly a file containing the maze configuration in X format, secondly, the score
	// produced by the IP model
	public static void main(String[] args) {

		// Parse file and create maze
		mazeFile = new File(System.getProperty("user.dir") + "/" + args[0]);

		maze = parseFileCreateMaze(mazeFile);
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
		System.out.println(maze.size());

		boolean success = simulateMaze(maze, score);
		if(success)
			System.out.println("Score was correct for this maze.");
		else
			System.out.println("Score was incorrect for this maze.");
	}

	private static ArrayList<ArrayList<Integer>> parseFileCreateMaze(File mazeFile) {

		ArrayList<ArrayList<Integer>> maze = new ArrayList<ArrayList<Integer>>();

		try {
			Scanner input = new Scanner(mazeFile);
			int row = 0;
			while(input.hasNext()) {
				maze.add(readLine(input.nextLine()));
				row++;
			}
		} catch (FileNotFoundException e) {
			System.out.println("File not found, please specify the filename (INCLUDING EXTENSION) in the directory you are running the verifier from.");
			Scanner scanner = new Scanner(System.in);
			String fileName = scanner.nextLine();
			File tryAgainFile = new File(System.getProperty("user.dir") + "/mazeFile.txt");
			parseFileCreateMaze(tryAgainFile);
		}
		return maze;
	}

	private static boolean simulateMaze(ArrayList<ArrayList<Integer>> maze, int score) {
		int steps = 0;

		// Initialise visits array
		ArrayList<ArrayList<Integer>> visits = createVisits2DArray(maze);

		Location location = new Location(0, 0, maze.size() - 1);

		// Checks to see if mouse is in finish cell
		while (!location.equal()) {
			Location nextCell = nextCell();
			moveMouse(nextCell);
			steps++;
		}

		if (steps == score)
			return true;
		else
			return false;
	}

	private static ArrayList<Integer> readLine(String line) {
		ArrayList<Integer> lineAsInts = new ArrayList<Integer>();
		for (String value: line.split(","))
			lineAsInts.add(Integer.parseInt(value));
		return lineAsInts;
	}

	// Creates 2d array of same size as maze but places 0 in all cells
	private static ArrayList<ArrayList<Integer>> createVisits2DArray(ArrayList<ArrayList<Integer>> maze){

		ArrayList<ArrayList<Integer>> visits = new ArrayList<ArrayList<Integer>>();

		for (int i = 0; i < maze.size(); i++){
			for (int value:maze.get(i)){
				visits.get(i).add(0);
			}
		}
		return visits;
	}

	public static Location nextCell() {
		//if blah
			//down
		//elif blah
			//right
		//elif blah
			//left
		//elif blah
			//up
		return null;
	}

	public static void moveMouse(String direction) {

	}

}

