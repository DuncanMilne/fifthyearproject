public class Location<X, Y, Z> {
    // n is the size of the rows and columns;
    public int row, column, n;

    public Location(int row, int column, int n) {
        this.row = row;
        this.column = column;
        this.n = n;
    }

    public boolean equal() {
        if(this.row==this.n && this.column==0)
            return true;
        else
            return false;
    }
}