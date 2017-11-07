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
	private static ArrayList<ArrayList<Integer>> visits;

	// Takes two arguments, firstly a file containing the maze configuration in X format, secondly, the score
	// produced by the IP model
	public static void main(String[] args) {

		// Parse file and create maze
		mazeFile = new File(System.getProperty("user.dir") + "/" + args[0]);

		maze = parseFileCreateMaze(mazeFile);

		// TODO - determine if visits(1,1) should start as 1 and if steps should start at 2
		visits = createVisits2DArray(maze);

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

		simulateMaze(maze, score);
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

			// Add line of 1's at start and end to represent edges
			ArrayList<Integer> ones = new ArrayList<Integer>();
			for (int i = 0; i < row+2; i++)
				ones.add(1);
			maze.add(0, ones);
			maze.add(ones);

		} catch (FileNotFoundException e) {
			System.out.println("File not found, please specify the filename (INCLUDING EXTENSION) in the directory you are running the verifier from.");
			Scanner scanner = new Scanner(System.in);
			String fileName = scanner.nextLine();
			File tryAgainFile = new File(System.getProperty("user.dir") + "/mazeFile.txt");
			parseFileCreateMaze(tryAgainFile);
		}
		// TODO - add check here to ensure all rows and columns are same size
		return maze;
	}

	private static ArrayList<Integer> readLine(String line) {
		ArrayList<Integer> lineAsInts = new ArrayList<Integer>();
		lineAsInts.add(1);
		for (String value: line.split(","))
			lineAsInts.add(Integer.parseInt(value));
		lineAsInts.add(1);
		return lineAsInts;
	}

	// Creates 2d array of same size as maze but places 0 in all cells
	private static ArrayList<ArrayList<Integer>> createVisits2DArray(ArrayList<ArrayList<Integer>> maze){

		ArrayList<ArrayList<Integer>> visits = new ArrayList<ArrayList<Integer>>();

		for (int i = 0; i < maze.size(); i++){
			visits.add(new ArrayList<Integer>());
			for (int value:maze.get(i)){
				if (value==0)
					visits.get(i).add(0);
				else
					visits.get(i).add(Integer.MAX_VALUE);
			}
		}

		return visits;
	}

	private static void simulateMaze(ArrayList<ArrayList<Integer>> maze, int score) {
		int steps = 2;

		Location location = new Location(1, 1);

		int newVisitsValue = 0;

		// Checks to see if mouse is in finish cell
		//System.out.println(location.row != maze.size() - 2);
		while (location.row != maze.size() - 2 || location.column != 1) {
			location = nextCell(location);
			newVisitsValue = visits.get(location.row).get(location.column) + 1;
			visits.get(location.row).set(location.column, newVisitsValue);
			steps++;
		}

		if(steps == score)
			System.out.println("Score was correct for this maze.");
		else
			System.out.println("Score was incorrect for this maze, got " + steps + " steps.");
	}

	public static Location nextCell(Location location) {

		Location down = new Location(location.row+1, location.column);
		Location right = new Location(location.row, location.column+1);
		Location left = new Location(location.row, location.column-1);
		Location up = new Location(location.row-1, location.column);

		if (compareLocations(down, right) && compareLocations(down, left) && compareLocations(down, up))
			return down;
		else if (compareLocations(right, left) && compareLocations(right, up))
			return right;
		else if (compareLocations(left, up))
			return left;
		else
			return up;
	}

	// returns true if x has <= y's visits, false otherwise
	public static boolean compareLocations(Location x, Location y) {
		if (visits.get(x.row).get(x.column) <= visits.get(y.row).get(y.column))
			return true;
		else
			return false;
	}
}

