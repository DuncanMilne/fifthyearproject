import java.util.ArrayList;
import java.util.Arrays;

import gurobi.*;
public class MouseMazeIPModel {

	public GRBEnv GRBEnv;
	public GRBModel GRBModel;
	public Utilities utilities;
	private final int size;
	final double upperBound;
	
	// Objective function
	GRBLinExpr T;
	
	public ArrayList<ArrayList<GRBVar>> grid;
	public ArrayList<ArrayList<ArrayList<GRBVar>>> decisions;	
	public ArrayList<ArrayList<ArrayList<GRBVar>>> visits;
	
	int row, column, timestep;
	GRBVar[][] TkMinusOne;
	GRBVar[][][] TkMinusOneArray;
	GRBVar[][] Tk;

	// We have to compare things to one regularly so a linear expression containing one is useful.
	GRBLinExpr one = new GRBLinExpr();
	ArrayList<GRBVar> listOfNonAccessedVars;
	
	// TODO add proper feasibility constraints. I believe it does not work currently as the current constraint only says that at one timestep, squeaky must be at the finish.
	// This means infeasible mazes are allowed since we just make him not able to move then jump him to the end.
	
	public MouseMazeIPModel(int size, int score, int timeInMinutes, String fileToPrintTo) {
		one.addConstant(1.0);
		try {
			GRBEnv = new GRBEnv();
			GRBModel = new GRBModel(GRBEnv);
		} catch (GRBException e) {
			e.printStackTrace();
		}
		this.upperBound = 50;
		this.size = size;
		this.utilities = new Utilities(size, GRBModel, (int) upperBound);
		grid = utilities.createGrid();
		visits = utilities.createVisits();
		decisions = utilities.createDecisions();
		addConstraints();
	}
	
	public void addConstraints() {
		for (row = 1; row < size + 1; row++) {
			for(column = 1; column < size + 1; column++) {
				for(timestep = 0; timestep < upperBound; timestep++) {
					visitsConstraints();
					if(timestep != upperBound - 1) 
						movementConstraints();
					obstacleConstraints();
				}
			}
		}
//		
		try {
			GRBVar feasiblePath = mazeFeasibilityConstraints();
			GRBModel.addConstr(feasiblePath, GRB.EQUAL, 1, "");
		} catch (GRBException e) {
			e.printStackTrace();
		}
		oneMoveAtATime();
	}
	
	public void visitsConstraints() {
		
		GRBLinExpr sumOfDecisions = new GRBLinExpr();
		
		double[] ones;
		
		ones = new double[timestep+1];
		Arrays.fill(ones, 1.0);
		
		GRBVar[] decisionsUntilK = decisions.get(row).get(column).subList(0, timestep+1).toArray(new GRBVar[timestep+1]);
		
		try {
			sumOfDecisions = new GRBLinExpr();
			sumOfDecisions.addTerms(ones, decisionsUntilK);
			sumOfDecisions.addTerm(upperBound + 1, grid.get(row).get(column));
			GRBModel.addConstr(visits.get(row).get(column).get(timestep), GRB.EQUAL, sumOfDecisions, "");
		} catch (GRBException e) {
			e.printStackTrace();
		}
	}
	
	public void movementConstraints() {
		if(atFinish()) {
			stayStill();
		} else {
			try {
				// we could return whether or not constraints are satisfied for each move and then say down must be geq LRU etc...
				moveDown();
				moveRight();
				moveLeft();
				moveUp();
			} catch(GRBException e) {
				e.printStackTrace();
			}
		}
	}
	
	public boolean atFinish() {
		return (row == size && column == 1);
	}
	
	public void stayStill() {
		GRBLinExpr nextPos = new GRBLinExpr();
		nextPos.addTerm(1.0, decisions.get(row).get(column).get(timestep+1));
		GRBLinExpr currentPos = new GRBLinExpr();
		currentPos.addTerm(1.0, decisions.get(row).get(column).get(timestep));
		try {
			GRBModel.addConstr(nextPos, GRB.GREATER_EQUAL, currentPos, "");
		} catch (GRBException e) {
			e.printStackTrace();
		}
	}
	
	public void moveDown() throws GRBException {
		if (!oneAway("down")) {
			// leq means <=
			GRBVar downLeqRight = standardBoolean(), downLeqLeft = standardBoolean(), downLeqUp = standardBoolean();
			
			GRBLinExpr downLeqRightExpr = new GRBLinExpr(), downLeqLeftExpr = new GRBLinExpr(), downLeqUpExpr = new GRBLinExpr();

			addTermsForAddition("down", downLeqRightExpr, downLeqLeftExpr, downLeqUpExpr);
			
			addNegations("right", downLeqRightExpr, "left", downLeqLeftExpr, "up", downLeqUpExpr);
			
			GRBModel.addGenConstrIndicator(downLeqRight, 1, downLeqRightExpr, GRB.LESS_EQUAL, 0, "");
			GRBModel.addGenConstrIndicator(downLeqRight, 0, downLeqRightExpr, GRB.GREATER_EQUAL, 1, "");
			GRBModel.addGenConstrIndicator(downLeqLeft, 1, downLeqLeftExpr, GRB.LESS_EQUAL, 0, "");
			GRBModel.addGenConstrIndicator(downLeqLeft, 0, downLeqLeftExpr, GRB.GREATER_EQUAL, 1, "");
			GRBModel.addGenConstrIndicator(downLeqUp, 1, downLeqUpExpr, GRB.LESS_EQUAL, 0, "");
			GRBModel.addGenConstrIndicator(downLeqUp, 0, downLeqUpExpr, GRB.GREATER_EQUAL, 1, "");
		
			GRBVar[] toSatisfy = new GRBVar[] {downLeqRight, downLeqLeft, downLeqUp, decisions.get(row).get(column).get(timestep)/*, indicator*/};
			
			GRBVar satisfied = standardBoolean();
			
			GRBModel.addGenConstrAnd(satisfied, toSatisfy, "");
			
			GRBModel.addConstr(decisions.get(row + 1).get(column).get(timestep + 1), GRB.GREATER_EQUAL, satisfied, "");
		} else {
			moveToFinish();
		}
	}
	
	public void moveRight() throws GRBException {
//		if (!oneAway("right")) {
			// leq means <=
			GRBVar rightLessThanDown = standardBoolean(), rightLeqLeft= standardBoolean(), rightLeqUp = standardBoolean();
			
			GRBLinExpr rightLessThanDownExpr = new GRBLinExpr(), rightLeqLeftExpr = new GRBLinExpr(), rightLeqThanUpExpr = new GRBLinExpr();

			rightLessThanDownExpr.addConstant(1.0);

			addTermsForAddition("right", rightLessThanDownExpr, rightLeqLeftExpr, rightLeqThanUpExpr);
			
			
			addNegations("down", rightLessThanDownExpr, "left", rightLeqLeftExpr, "up", rightLeqThanUpExpr);
			GRBModel.addGenConstrIndicator(rightLessThanDown, 1, rightLessThanDownExpr, GRB.LESS_EQUAL, 0, "");
			GRBModel.addGenConstrIndicator(rightLessThanDown, 0, rightLessThanDownExpr, GRB.GREATER_EQUAL, 1, "");
			GRBModel.addGenConstrIndicator(rightLeqLeft, 1, rightLeqLeftExpr, GRB.LESS_EQUAL, 0, "");
			GRBModel.addGenConstrIndicator(rightLeqLeft, 0, rightLeqLeftExpr, GRB.GREATER_EQUAL, 1, "");
			GRBModel.addGenConstrIndicator(rightLeqUp, 1, rightLeqThanUpExpr, GRB.LESS_EQUAL, 0, "");
			GRBModel.addGenConstrIndicator(rightLeqUp, 0, rightLeqThanUpExpr, GRB.GREATER_EQUAL, 1, "");
			
			
			GRBVar[] toSatisfy = new GRBVar[] {rightLessThanDown, rightLeqLeft, rightLeqUp, decisions.get(row).get(column).get(timestep)/*, indicator*/};
			
			GRBVar satisfied = standardBoolean();
			
			GRBModel.addGenConstrAnd(satisfied, toSatisfy, "");
			
			GRBModel.addConstr(decisions.get(row).get(column + 1).get(timestep + 1), GRB.GREATER_EQUAL, satisfied, "");
//		} else {
//			moveToFinish();
//		}
	}
	
	public void moveLeft() throws GRBException {
//		if(!oneAway("left")) {
			GRBVar leftLessThanDown = standardBoolean(), leftLessThanRight = standardBoolean(), leftLeqUp = standardBoolean();
			
			GRBLinExpr leftLessThanDownExpr = new GRBLinExpr(), leftLessThanRightExpr = new GRBLinExpr(), leftLeqUpExpr = new GRBLinExpr();

			leftLessThanDownExpr.addConstant(1.0);
			leftLessThanRightExpr.addConstant(1.0);
			
			addTermsForAddition("left", leftLessThanDownExpr, leftLessThanRightExpr, leftLeqUpExpr);
			
			addNegations("down", leftLessThanDownExpr, "right", leftLessThanRightExpr, "up", leftLeqUpExpr);
			
//			
			GRBModel.addGenConstrIndicator(leftLessThanDown, 1, leftLessThanDownExpr, GRB.LESS_EQUAL, 0, "");
			GRBModel.addGenConstrIndicator(leftLessThanDown, 0, leftLessThanDownExpr, GRB.GREATER_EQUAL, 1, "");
			GRBModel.addGenConstrIndicator(leftLessThanRight, 1, leftLessThanRightExpr, GRB.LESS_EQUAL, 0, "");
			GRBModel.addGenConstrIndicator(leftLessThanRight, 0, leftLessThanRightExpr, GRB.GREATER_EQUAL, 1, "");
			GRBModel.addGenConstrIndicator(leftLeqUp, 1, leftLeqUpExpr, GRB.LESS_EQUAL, 0, "");
			GRBModel.addGenConstrIndicator(leftLeqUp, 0, leftLeqUpExpr, GRB.GREATER_EQUAL, 1, "");
				
			GRBVar[] toSatisfy = new GRBVar[] {	leftLessThanDown, leftLessThanRight, leftLeqUp, decisions.get(row).get(column).get(timestep), /*indicator*/};
			
			GRBVar satisfied = standardBoolean();
			
			GRBModel.addGenConstrAnd(satisfied, toSatisfy, "");
			
			GRBModel.addConstr(decisions.get(row).get(column -1).get(timestep + 1), GRB.GREATER_EQUAL, satisfied, "");
//		} else {
//			moveToFinish();
//		}
	}
	
	public void moveUp() throws GRBException {
//		if (!oneAway("up") && !oneAway("down")) {
			GRBVar upLessThanDown = standardBoolean(), upLessThanRight = standardBoolean(), upLessThanLeft = standardBoolean();
			
			GRBLinExpr upLessThanDownExpr = new GRBLinExpr(), upLessThanRightExpr = new GRBLinExpr(), upLessThanLeftExpr = new GRBLinExpr();
			
			upLessThanDownExpr.addConstant(1.0);
			upLessThanRightExpr.addConstant(1.0);
			upLessThanLeftExpr.addConstant(1.0);
			
			addTermsForAddition("up", upLessThanDownExpr, upLessThanRightExpr, upLessThanLeftExpr);
			
			addNegations("down", upLessThanDownExpr, "right", upLessThanRightExpr, "left", upLessThanLeftExpr);
			
			GRBModel.addGenConstrIndicator(upLessThanDown, 1, upLessThanDownExpr, GRB.LESS_EQUAL, 0, "");
			GRBModel.addGenConstrIndicator(upLessThanDown, 0, upLessThanDownExpr, GRB.GREATER_EQUAL, 1, "");
			GRBModel.addGenConstrIndicator(upLessThanRight, 1, upLessThanRightExpr, GRB.LESS_EQUAL, 0, "");
			GRBModel.addGenConstrIndicator(upLessThanRight, 0, upLessThanRightExpr, GRB.GREATER_EQUAL, 1, "");
			GRBModel.addGenConstrIndicator(upLessThanLeft, 1, upLessThanLeftExpr, GRB.LESS_EQUAL, 0, "");
			GRBModel.addGenConstrIndicator(upLessThanLeft, 0, upLessThanLeftExpr, GRB.GREATER_EQUAL, 1, "");
			
			GRBVar[] toSatisfy = new GRBVar[] {	upLessThanDown, upLessThanRight, upLessThanLeft, decisions.get(row).get(column).get(timestep), /*indicator*/};
			
			GRBVar satisfied = standardBoolean();
			
			GRBModel.addGenConstrAnd(satisfied, toSatisfy, "");
			
			GRBModel.addConstr(decisions.get(row-1).get(column).get(timestep + 1), GRB.GREATER_EQUAL, satisfied, "");
//		} else {
//			moveToFinish();
//		}
	}
	
	public void addNegations(String firstString, GRBLinExpr first, String secondString, GRBLinExpr second, String thirdString, GRBLinExpr third) {
		addTermForNegation(firstString, first);
		addTermForNegation(secondString, second);
		addTermForNegation(thirdString, third);
	}
	
	public void addTermsForAddition(String direction, GRBLinExpr first, GRBLinExpr second, GRBLinExpr third) {
		int tempRow, tempColumn;
		switch(direction) {
			case "down":
				tempRow = row + 1;
				tempColumn = column;
				break;
			case "right":
				tempRow = row;
				tempColumn = column + 1;
				break;
			case "left": 
				tempRow = row;
				tempColumn = column -1;
				break;
			case "up":
				tempRow = row - 1;
				tempColumn = column;
				break;
			default:
				tempRow = Integer.MAX_VALUE;
				tempColumn = Integer.MAX_VALUE;
				System.out.println("this should never happen");
		}
		first.addTerm(1.0, visits.get(tempRow).get(tempColumn).get(timestep));
		second.addTerm(1.0, visits.get(tempRow).get(tempColumn).get(timestep));
		third.addTerm(1.0, visits.get(tempRow).get(tempColumn).get(timestep));
	}
	
	public void addTermForNegation(String direction, GRBLinExpr toAddTo) {
		switch(direction) {
			case "down":
				toAddTo.addTerm(-1.0, visits.get(row+1).get(column).get(timestep));
				break;
			case "right":
				toAddTo.addTerm(-1.0, visits.get(row).get(column+1).get(timestep));
				break;
			case "left":
				toAddTo.addTerm(-1.0, visits.get(row).get(column-1).get(timestep));
				break;
			case "up":
				toAddTo.addTerm(-1.0, visits.get(row-1).get(column).get(timestep));
				break;
		}
	}
	
	public GRBVar standardBoolean() {
		try {
			return GRBModel.addVar(0.0, 1.0, 0.0, GRB.BINARY, "");
		} catch (GRBException e) {
			e.printStackTrace();
			return null;
		}
	}
	
	public boolean oneAway(String direction) {
		switch(direction) {
			case "down": 
				if (row == size-1 && column == 1) 
					return true;
			case "right":
				if (row == size && column == 0)
					return true;
			case "left":
				if (row == size && column == 2)
					return true;
			case "up":
				if (row == size + 1 && column == 1)
					return true;
			default:
				return false;
		}
	}
	
	public void moveToFinish() {
		GRBLinExpr finish = new GRBLinExpr();
		GRBLinExpr current = new GRBLinExpr();
		current.addTerm(1.0, decisions.get(row).get(column).get(timestep));
		finish.addTerm(1.0, decisions.get(size).get(1).get(timestep + 1));
		try {
			GRBModel.addConstr(finish, GRB.GREATER_EQUAL, current, "");
		} catch (GRBException e) {
			e.printStackTrace();
		}
	}
	
	// indicator variable is 1 if the sum is leq than 0, 0 if geq than 1
	public void greaterThanOneIndicator(GRBVar indicator, GRBLinExpr sum) {
		try {
			GRBModel.addGenConstrIndicator(indicator, 1, sum, GRB.LESS_EQUAL, 0.0, "");
			GRBModel.addGenConstrIndicator(indicator, 0, sum, GRB.GREATER_EQUAL, 1.0, "");
		} catch (GRBException e) {
			e.printStackTrace();
		}
	}
	
	public void obstacleConstraints() {
		GRBLinExpr obstaclePlusMovement = new GRBLinExpr();
		
		obstaclePlusMovement.addTerm(1.0, grid.get(row).get(column));
		
		try {
			obstaclePlusMovement.addTerm(1.0,  decisions.get(row).get(column).get(timestep));
			GRBModel.addConstr(obstaclePlusMovement, GRB.LESS_EQUAL, one, "");
		} catch (GRBException e) {
			e.printStackTrace();
		}
	}
	 
	public GRBVar mazeFeasibilityConstraints() throws GRBException {
		
		TkMinusOne = new GRBVar[size*size][size*size]; // pathtostart[0][i] (first row) represents the distance from 1,1 to all other vertices. when doing with size = 3, this will be a 9x9
		
		// Maintain list of vars that are not possible when k = 0 due to the cells not being adjacent
		listOfNonAccessedVars = new ArrayList<GRBVar>();
		
		for (int row = 0; row < size*size; row ++) {
			for (int column = 0; column < size*size; column ++) {
				if (row != column) {
					TkMinusOne[row][column] = GRBModel.addVar(0.0, 1.0, 0.0, GRB.BINARY, "FWmatrix row " + row + " column "+ column);
					listOfNonAccessedVars.add(TkMinusOne[row][column]);
				} else {
					TkMinusOne[row][column] = GRBModel.addVar(1.0, 1.0, 0.0, GRB.BINARY, "FWmatrix row " + row + " column " + column); // There is a path between i and i
				}
			}	
		}

		// do we have to say if we have set there to en edge, there is not an edge..  ??? for example at 1,1 there is no edge to 2,2 but we never say that it there is not an edge
		for (int row = 0; row < size * size; row++) {
			if (row < (size*size) - size) { // if we're not at the bottom row. this should not be true if we are at rows 2 and 3 for a 2x2
				addEdgeDown(TkMinusOne[row][row+size], row);		// there is an edge between each pair of adjacent cells unless there is an obstacle
				listOfNonAccessedVars.remove(TkMinusOne[row][row+size]);
			}
			if(row % size != size-1) { 
				addEdgeRight(TkMinusOne[row][row+1], row);
				listOfNonAccessedVars.remove(TkMinusOne[row][row+1]);
			}
			if(row % size != 0) {
				addEdgeLeft(TkMinusOne[row][row-1], row);
				listOfNonAccessedVars.remove(TkMinusOne[row][row-1]);
			}
			if(row >= size) {
				addEdgeUp(TkMinusOne[row][row-size], row);
				listOfNonAccessedVars.remove(TkMinusOne[row][row-size]);
			}
		}

		for (GRBVar var: listOfNonAccessedVars) {
			GRBModel.addConstr(var, GRB.LESS_EQUAL, 0, "");
		}
		
		TkMinusOneArray = new GRBVar[size*size+1][size*size][size*size];
		TkMinusOneArray[0] = TkMinusOne;		
		// an edge exists if they are adjacent and neither has an obstacle. if an edge exists, variable is 1.
		for (int k = 0; k < size*size; k ++) {	// for every vertex, try to use it as an intermediate vertex between every other pair of vertices
			Tk = new GRBVar[size*size][size*size]; // pathtostart[0][i] (first row) represents the distance from 1,1 to all other vertices. when doing with size = 3, this will be a 9x9
			for (int i = 0; i < size*size; i ++) {
				for (int j = 0; j < size*size; j ++) {
					Tk[i][j] = GRBModel.addVar(0.0, 1.0, 0.0, GRB.BINARY, "");
					GRBVar Cijk = conjunction(k, i, j);
					GRBModel.addGenConstrOr(Tk[i][j], new GRBVar[] {Cijk, TkMinusOne[i][j]}, "");
				}
			}
			TkMinusOne = Tk;

			TkMinusOneArray[k+1] = TkMinusOne;
		}
		return TkMinusOne[(size*size)-size	][0];
	}
	
	void printTkMinusOneArray() {
		for (int k = 0; k < size*size+1; k++) {
			for (int i = 0; i < size*size; i++) {
				for (int j = 0; j < size*size; j++) {
					try {
						System.out.print((int) TkMinusOneArray[k][i][j].get(GRB.DoubleAttr.X) + " ");
					} catch (GRBException e) {
						e.printStackTrace();
					}
				}
	
				System.out.println();
			}
			System.out.println();
		}
	}
	
	GRBVar conjunction(int k, int i, int j) throws GRBException {
		GRBVar Cijk = GRBModel.addVar(0.0, 1.0, 0.0, GRB.BINARY, "");
		GRBVar tikMinusOne = TkMinusOne[i][k];
		GRBVar tkjMinusOne = TkMinusOne[k][j];
		GRBModel.addGenConstrAnd(Cijk, new GRBVar[] {tikMinusOne, tkjMinusOne}, "");
		return Cijk;
	}
	
	void addEdgeDown(GRBVar currentVar, int row) throws GRBException {
		GRBLinExpr edgeExists = new GRBLinExpr(), edgeExists2= new GRBLinExpr(), edgeExists3= new GRBLinExpr(),edgeExists4= new GRBLinExpr();
		edgeExists.addConstant(1);
		edgeExists.addTerm(-1,grid.get(row/size + 2).get(row%size + 1));
		
		edgeExists2.addConstant(1);
		edgeExists2.addTerm(-1, grid.get(row/size + 1).get(row%size + 1));
		
		GRBModel.addConstr(currentVar, GRB.LESS_EQUAL, edgeExists, "duncan");
		GRBModel.addConstr(currentVar, GRB.LESS_EQUAL, edgeExists2, "duncan");

		edgeExists3.addConstant(1.0);
		edgeExists3.addTerms(new double[] {-1,-1}, new GRBVar[] {grid.get(row/size + 1).get(row%size + 1), grid.get(row/size + 2).get(row%size + 1)});
		GRBModel.addConstr(currentVar, GRB.GREATER_EQUAL, edgeExists3, "");
	}

	// grid contains edges adjacent to grid so just doing grid.get(row+whichRow).get(column) will not work as we get the two edges as well
	// if column%size == 0 || column % size+1 == 0 - nope
	void addEdgeRight(GRBVar currentVar, int row) throws GRBException {
		GRBLinExpr edgeExists = new GRBLinExpr(), edgeExists2= new GRBLinExpr(), edgeExists3= new GRBLinExpr(),edgeExists4= new GRBLinExpr();
		edgeExists.addConstant(1);
		edgeExists.addTerm(-1,grid.get(row/size + 1).get(row%size + 1));
		
		edgeExists2.addConstant(1);
		edgeExists2.addTerm(-1, grid.get(row/size + 1).get(row%size + 2));
		
		GRBModel.addConstr(currentVar, GRB.LESS_EQUAL, edgeExists, "duncan");
		GRBModel.addConstr(currentVar, GRB.LESS_EQUAL, edgeExists2, "duncan");
		
		edgeExists3.addConstant(1.0);
		edgeExists3.addTerms(new double[] {-1,-1}, new GRBVar[] {grid.get(row/size + 1).get(row%size + 1), grid.get(row/size + 1).get(row%size + 2)});
		GRBModel.addConstr(currentVar, GRB.GREATER_EQUAL, edgeExists3, "");
	}

	void addEdgeLeft(GRBVar currentVar, int row) throws GRBException {
		GRBLinExpr edgeExists = new GRBLinExpr(), edgeExists2= new GRBLinExpr(), edgeExists3= new GRBLinExpr(),edgeExists4= new GRBLinExpr();
		edgeExists.addConstant(1);
		edgeExists.addTerm(-1,grid.get(row/size + 1).get(row%size + 1));
		
		edgeExists2.addConstant(1);
		edgeExists2.addTerm(-1, grid.get(row/size + 1).get(row%size));
		
		GRBModel.addConstr(currentVar, GRB.LESS_EQUAL, edgeExists, "duncan");
		GRBModel.addConstr(currentVar, GRB.LESS_EQUAL, edgeExists2, "duncan");
		
		edgeExists3.addConstant(1.0);
		edgeExists3.addTerms(new double[] {-1,-1}, new GRBVar[] {grid.get(row/size + 1).get(row%size + 1), grid.get(row/size + 1).get(row%size)});
		GRBModel.addConstr(currentVar, GRB.GREATER_EQUAL, edgeExists3, "");
	}

	void addEdgeUp(GRBVar currentVar, int row) throws GRBException {		
		
		GRBLinExpr edgeExists = new GRBLinExpr(), edgeExists2= new GRBLinExpr(), edgeExists3= new GRBLinExpr(),edgeExists4= new GRBLinExpr();
		edgeExists.addConstant(1);
		edgeExists.addTerm(-1,grid.get(row/size + 1).get(row%size + 1));
		
		edgeExists2.addConstant(1);
		edgeExists2.addTerm(-1, grid.get(row/size).get(row%size + 1));
		
		GRBModel.addConstr(currentVar, GRB.LESS_EQUAL, edgeExists, "duncan");
		GRBModel.addConstr(currentVar, GRB.LESS_EQUAL, edgeExists2, "duncan");
		
		edgeExists3.addConstant(1.0);
		edgeExists3.addTerms(new double[] {-1,-1}, new GRBVar[] {grid.get(row/size + 1).get(row%size + 1), grid.get(row/size).get(row%size + 1)});
		GRBModel.addConstr(currentVar, GRB.GREATER_EQUAL, edgeExists3, "");
	}

	boolean isDownMost(int row) {
		return (row == size);
	}
	
	boolean isRightMost(int column) {
		return (column == size);
	}
	
	boolean isLeftMost(int column) {
		return (column == 1);
	}
	
	boolean isUpMost(int row) {
		return (row == 1);
	}
//	public void basicFeasibility() {
//		GRBLinExpr feasibilityConstraint = new GRBLinExpr();
//		feasibilityConstraint.addTerm(1.0,  decisions.get(size).get(1).get((int) upperBound-1));
//		
//		try {
//			GRBModel.addConstr(feasibilityConstraint, GRB.GREATER_EQUAL, one, "");
//		} catch (GRBException e) {
//			e.printStackTrace();
//		}
//	}
	
	// At timestep k we sum up all of squeakyDecision variables at k and make sure they are leq than 1
	public void oneMoveAtATime() {
		GRBLinExpr oneMovePerK;
		
		for(timestep = 0; timestep < upperBound; timestep++) {
			oneMovePerK = new GRBLinExpr();
			for (row = 1; row < size + 1; row++) {
				for (column = 1; column < size + 1; column++) {
					oneMovePerK.addTerm(1.0, decisions.get(row).get(column).get(timestep));
				}
			}
			try {
				GRBModel.addConstr(oneMovePerK, GRB.EQUAL, 1, "");
			} catch (GRBException e) {
				e.printStackTrace();
			}
		}
	}
	
	// Trying to maximise the number of timesteps squeaky is not at n,1
		public void setObjectiveFunction() {
			
			T = new GRBLinExpr();
			
			for (int timestep = 0; timestep < upperBound; timestep++) {
				T.addConstant(1.0);
				T.addTerm(-1.0, decisions.get(size).get(1).get(timestep));
			}
			T.addConstant(1.0);
			try {
				GRBModel.setObjective(T, GRB.MAXIMIZE);
			} catch (GRBException e) {
				e.printStackTrace();
			}
		}
}
