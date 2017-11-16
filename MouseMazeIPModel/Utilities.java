import java.util.ArrayList;

import gurobi.GRB;
import gurobi.GRBException;
import gurobi.GRBModel;
import gurobi.GRBVar;

public class Utilities {

	private final int size, upperBound;
	private final GRBModel GRBModel;

	public Utilities(int size, GRBModel GRBModel, int upperBound) {
		this.size = size;
		this.GRBModel = GRBModel;
		this.upperBound = upperBound;
	}

	public ArrayList<ArrayList<GRBVar>> createGrid() {

		ArrayList<ArrayList<GRBVar>> grid = new ArrayList<ArrayList<GRBVar>>();
		ArrayList<GRBVar> tempRow;
		GRBVar currentVar = null;

		for (int row = 0; row < size + 2; row++) {
			tempRow = new ArrayList<GRBVar>();
			for (int column = 0; column < size + 2; column++) {
				try { // this if statement can probably be removed when proper feasiblity constraints are added
					if (atEdge(row, column)) {
						currentVar = GRBModel.addVar(1.0, 1.0, 0.0, GRB.BINARY, "grid row is "+ row + " column is " + column);
					} else {
						currentVar = GRBModel.addVar(0.0, 1.0, 0.0, GRB.BINARY, "grid row is "+ row + " column is " + column);
					}
				} catch (GRBException e) {
					e.printStackTrace();
				}
				tempRow.add(currentVar);
			}
			grid.add(tempRow);
		}

		return grid;
	}

	public ArrayList<ArrayList<ArrayList<GRBVar>>> createVisits() {

		ArrayList<ArrayList<ArrayList<GRBVar>>> visits = new ArrayList<ArrayList<ArrayList<GRBVar>>>();
		ArrayList<ArrayList<GRBVar>> tempRow;
		ArrayList<GRBVar> tempColumn;
		GRBVar currentVar = null;

		for (int row = 0; row < size + 2; row++) {
			tempRow = new ArrayList<ArrayList<GRBVar>>();
			for (int column = 0; column < size + 2; column++) {
				tempColumn = new ArrayList<GRBVar>();
				for (int timestep = 0; timestep < upperBound; timestep++) {
					currentVar = determineVariableType(row, column, timestep);
					tempColumn.add(currentVar);
				}
				tempRow.add(tempColumn);
			}
			visits.add(tempRow);
		}

		return visits;
	}

	public GRBVar determineVariableType(int row, int column, int timestep) {
		GRBVar currentVar = null;
		
		try {
			if (atEdge(row, column))
				currentVar = GRBModel.addVar(upperBound + 1, upperBound + 1, 0.0, GRB.INTEGER, "visits row is "+ row + " column is " + column + " timestep is " + timestep);
			else if (settingStartMove(row, column, timestep))
				currentVar = GRBModel.addVar(1.0, 1.0, 0.0, GRB.BINARY, "");
			else
				currentVar = GRBModel.addVar(0.0, upperBound, 0.0, GRB.INTEGER,  "visits row is "+ row + " column is " + column + " timestep is " + timestep);
		} catch (GRBException e) {
			e.printStackTrace();
		}
		return currentVar;
	}
	
	public boolean atEdge(int row, int column) {
		return (row == 0 || row == size + 1 || column == 0 || column == size + 1);
	}

	public boolean settingStartMove(int row, int column, int timestep) {
		return (row == 1 && column == 1 && timestep == 0);
	}
	
	
	public ArrayList<ArrayList<ArrayList<GRBVar>>> createDecisions() {
		ArrayList<ArrayList<ArrayList<GRBVar>>> decisions = new ArrayList<ArrayList<ArrayList<GRBVar>>>();
		ArrayList<ArrayList<GRBVar>> tempRow;
		ArrayList<GRBVar> tempColumn;
		GRBVar currentVar = null;
		
		for (int row = 0; row < size + 2; row++) {
			tempRow = new ArrayList<ArrayList<GRBVar>>();
			for (int column = 0; column < size + 2; column++) {
				tempColumn = new ArrayList<GRBVar>();
				for (int timestep = 0; timestep < upperBound; timestep++) {
						try {
							if(settingStartMove(row, column, timestep))
								currentVar = GRBModel.addVar(1.0, 1.0, 0.0, GRB.BINARY, "");
							else
								currentVar = GRBModel.addVar(0.0, 1.0, 0.0, GRB.BINARY, "");
						} catch (GRBException e) {
							e.printStackTrace();
						}
					tempColumn.add(currentVar);
				}
				tempRow.add(tempColumn);
			}
			decisions.add(tempRow);
		}
				
		return decisions;
	}
	
	public void printToFile(ArrayList<ArrayList<ArrayList<GRBVar>>> visits, ArrayList<ArrayList<ArrayList<GRBVar>>> decisions,  ArrayList<ArrayList<GRBVar>> grid) {
		// printing values of visits for now

		for (int timestep = 0; timestep < upperBound; timestep++) {
			for (int row = 0; row < size + 2; row++) {
				for (int column = 0; column < size + 2; column++) {
						try {
							 System.out.print(decisions.get(row).get(column).get(timestep).get(GRB.DoubleAttr.X) + " "); // up until timestep 9, squeaky is at all four non edge cells
						} catch (GRBException e) {
							e.printStackTrace();
						}
					
				}
				System.out.println();
			}
			System.out.println();
		}
	}
	
	public void printGrid(ArrayList<ArrayList<GRBVar>> grid) {

		System.out.println("NOW WE PRINT GRID");
		for (int i = 0; i<size+2;i++) {
			for (int j=0; j <size+2; j++) {
				try {
					System.out.print(grid.get(i).get(j).get(GRB.DoubleAttr.X) + " ");
				} catch (GRBException e) {
					e.printStackTrace();
				}
			}
			System.out.println();
		}
		System.out.println();
	}
	
}
