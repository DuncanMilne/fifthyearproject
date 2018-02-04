import java.util.ArrayList;

import gurobi.GRB;
import gurobi.GRBConstr;
import gurobi.GRBException;
import gurobi.GRBLinExpr;
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

	public ArrayList<ArrayList<GRBVar>> createGrid(int obs) {

		ArrayList<ArrayList<GRBVar>> grid = new ArrayList<ArrayList<GRBVar>>();
		ArrayList<GRBVar> tempRow;
		GRBVar currentVar = null;

		GRBLinExpr setNumberOfObstacles = new GRBLinExpr(); 
		for (int row = 0; row < size; row++) {
			tempRow = new ArrayList<GRBVar>();
			for (int column = 0; column < size; column++) {
				try { // this if statement can probably be removed when proper feasiblity constraints are added
//					if (atEdge(row, column)) {
//						currentVar = GRBModel.addVar(1.0, 1.0, 0.0, GRB.BINARY, "grid_row_is_"+ row + "_column_is_" + column);
//					} else {
					if ((row==0 && column==0)||(row==size-1 && column == 0)) {
						currentVar = GRBModel.addVar(0.0, 0.0, 0.0, GRB.BINARY, "grid_row_is_"+ row + "_column_is_" + column);
				    } else {
						currentVar = GRBModel.addVar(0.0, 1.0, 0.0, GRB.BINARY, "grid_row_is_"+ row + "_column_is_" + column);
						setNumberOfObstacles.addTerm(1.0, currentVar);
					}
				} catch (GRBException e) {
					e.printStackTrace();
				}
				tempRow.add(currentVar);
			}
			grid.add(tempRow);
		}

		try {
			GRBModel.addConstr(setNumberOfObstacles, GRB.LESS_EQUAL, (size*size)-size-1, "numobsmustbelessthan" + ((size*size)-size-1));
			if(obs>0)
				GRBModel.addConstr(setNumberOfObstacles, GRB.EQUAL, obs, "numobsmustbe" + obs);
		} catch (GRBException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		return grid;
	}

	public ArrayList<ArrayList<ArrayList<GRBVar>>> createVisits() {

		ArrayList<ArrayList<ArrayList<GRBVar>>> visits = new ArrayList<ArrayList<ArrayList<GRBVar>>>();
		ArrayList<ArrayList<GRBVar>> tempRow;
		ArrayList<GRBVar> tempColumn;
		GRBVar currentVar = null;

		for (int row = 0; row < size; row++) {
			tempRow = new ArrayList<ArrayList<GRBVar>>();
			for (int column = 0; column < size; column++) {
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
//			if (atEdge(row, column))
//				currentVar = GRBModel.addVar(upperBound + 1, upperBound + 1, 0.0, GRB.INTEGER, "visits_row_is_"+ row + "_column_is_" + column + "_timestep_is_" + timestep);
//			else if (settingStartMove(row, column, timestep))
			if (settingStartMove(row, column, timestep))
				currentVar = GRBModel.addVar(1.0, 1.0, 0.0, GRB.BINARY, "visits_row_is_"+ row + "_column_is_" + column + "_timestep_is_" + timestep);
			else
				currentVar = GRBModel.addVar(0.0, timestep+2, 0.0, GRB.INTEGER,  "visits_row_is_"+ row + "_column_is_" + column + "_timestep_is_" + timestep); // 
		} catch (GRBException e) {
			e.printStackTrace();
		}
		return currentVar;
	}
	
	public boolean atEdge(int row, int column) {
		return (row == 0 || row == size - 1 || column == 0 || column == size - 1);
	}

	public boolean settingStartMove(int row, int column, int timestep) {
		return (row == 0 && column == 0 && timestep == 0);
	}
	
	
	public ArrayList<ArrayList<ArrayList<GRBVar>>> createDecisions() {
		ArrayList<ArrayList<ArrayList<GRBVar>>> decisions = new ArrayList<ArrayList<ArrayList<GRBVar>>>();
		ArrayList<ArrayList<GRBVar>> tempRow;
		ArrayList<GRBVar> tempColumn;
		GRBVar currentVar = null;
		
		for (int row = 0; row < size; row++) {
			tempRow = new ArrayList<ArrayList<GRBVar>>();
			for (int column = 0; column < size; column++) {
				tempColumn = new ArrayList<GRBVar>();
				for (int timestep = 0; timestep < upperBound; timestep++) {
						try {
							if(settingStartMove(row, column, timestep))
								currentVar = GRBModel.addVar(1.0, 1.0, 0.0, GRB.BINARY, "decisions_row_is_"+ row + "_column_is_" + column + "_timestep_is_" + timestep);
							else
								currentVar = GRBModel.addVar(0.0, 1.0, 0.0, GRB.BINARY, "decisions_row_is_"+ row + "_column_is_" + column + "_timestep_is_" + timestep);
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
			for (int row = 0; row < size; row++) {
				for (int column = 0; column < size; column++) {
						try {
							 System.out.print(visits.get(row).get(column).get(timestep).get(GRB.DoubleAttr.X) + " "); // up until timestep 9, squeaky is at all four non edge cells
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
		for (int i = 0; i<size;i++) {
			for (int j=0; j <size; j++) {
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
