public class Location<X, Y> {
    // n is the size of the rows and columns;
    public int row, column;

    public Location(int row, int column) {
        this.row = row;
        this.column = column;
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
}