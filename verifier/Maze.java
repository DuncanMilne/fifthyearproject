import java.io.File;
import java.io.FileNotFoundException;
import java.util.Scanner;
import java.util.ArrayList;
import java.util.List;
import java.lang.ArrayIndexOutOfBoundsException;

public class Maze {

    private ArrayList<ArrayList<Integer>> maze;
    private ArrayList<ArrayList<Integer>> visits;

    public Maze(File mazeFile){

        this.maze = this.parseFileCreateMaze(mazeFile);

        createVisits2DArray();
    }

    protected void simulate(int score) {

        // Starts at two based on the way the game on Kongregate works. Squeaky needs to move onto the start vertex
        // and move off of the finish vertex, simply starting at 2 simulates these steps.
        int steps = 2;

        Location location = new Location(1, 1);

        int newVisitsValue = 0;

        // Checks to see if mouse is in finish cell
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

    private ArrayList<ArrayList<Integer>> parseFileCreateMaze(File mazeFile) {

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
        return maze;
    }

    private ArrayList<Integer> readLine(String line) {
        ArrayList<Integer> lineAsInts = new ArrayList<Integer>();
        lineAsInts.add(1);
        for (String value: line.split(","))
            lineAsInts.add(Integer.parseInt(value));
        lineAsInts.add(1);
        return lineAsInts;
    }

    // Creates 2d array of same size as maze but places 0 in all cells
    private void createVisits2DArray(){

        visits = new ArrayList<ArrayList<Integer>>();

        for (int i = 0; i < maze.size(); i++){
            visits.add(new ArrayList<Integer>());
            for (int value:maze.get(i)){
                if (value==0)
                    visits.get(i).add(0);
                else
                    visits.get(i).add(Integer.MAX_VALUE);
            }
        }

        // Squeaky starts by moving to the start square. Easier to simply start this value at one than to factor in additional vertices with different movement conditions.
        visits.get(1).set(1, 1);
    }

    // returns true if x has <= y's visits, false otherwise
    public boolean compareLocations(Location x, Location y) {
        if (visits.get(x.row).get(x.column) <= visits.get(y.row).get(y.column))
            return true;
        else
            return false;
    }

    public Location nextCell(Location location) {

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

}