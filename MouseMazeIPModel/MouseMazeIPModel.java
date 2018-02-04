import java.util.ArrayList;
import java.util.Arrays;

import gurobi.*;

public class MouseMazeIPModel {

	public GRBEnv GRBEnv;
	public GRBModel GRBModel;
	public Utilities utilities;
	private final int size;
	final int score;
	final double upperBound;
	
	// Objective function
	GRBLinExpr objective;
	
	public ArrayList<ArrayList<GRBVar>> grid;
	public ArrayList<ArrayList<ArrayList<GRBVar>>> decisions;	
	public ArrayList<ArrayList<ArrayList<GRBVar>>> visits;
	
	int row, column, timestep;
	GRBVar[][] TkMinus1;
	GRBVar[][][] TkMinus1Array;
	GRBVar[][] Tk;

	ArrayList<GRBVar> listOfNonAccessedVars;
	
	public MouseMazeIPModel(int size, int score, int timeInMinutes, String fileToPrintTo, int upperBound, int obs) {
		try {
			GRBEnv = new GRBEnv();
			GRBModel = new GRBModel(GRBEnv);
		} catch (GRBException e) {
			e.printStackTrace();
		}
		this.score = score;
		this.upperBound =upperBound;
		this.size = size;
		this.utilities = new Utilities(size, GRBModel, (int) upperBound);
		grid = utilities.createGrid(obs);
		visits = utilities.createVisits();
		decisions = utilities.createDecisions();
		try {
			addConstraints();
		} catch (GRBException e) {
			e.printStackTrace();
		}
		setObjectiveFunction();
	}
	
	public void addConstraints() throws GRBException {
		for (row = 0; row < size; row++) {
			for(column = 0; column < size; column++) {
				for(timestep = 0; timestep < upperBound; timestep++) {
					visitsConstraints();
					if(timestep != upperBound - 1) 
						movementConstraints();
					obstacleConstraints();
				}
			}
		}
	    oneMoveAtATime();
	    atLeastOneObstacleInColumnZero();
//	    transitiveClosureFeasibilityConstraints();
//		basicMazeFeasibilityConstraints();
		orderNSquaredConstraints();
	}
	
	public void atLeastOneObstacleInColumnZero() {
		GRBLinExpr atLeastOneObstacleInColumnZero = new GRBLinExpr();
		for (int gridRowNumber = 1; gridRowNumber < size-1; gridRowNumber++) {
			atLeastOneObstacleInColumnZero.addTerm(1.0, grid.get(gridRowNumber).get(0));
		}
		try {
			GRBModel.addConstr(atLeastOneObstacleInColumnZero, GRB.GREATER_EQUAL, 1, "atleastoneobstacleincolumn0");
		} catch (GRBException e) {
			e.printStackTrace();
		}
	}
	public void visitsConstraints() {
		
		GRBLinExpr sumOfDecisions = new GRBLinExpr();
		
		double[] ones;
		
		ones = new double[timestep+1];
		Arrays.fill(ones, 1.0);
		
		GRBVar[] decisionsUntilK = decisions.get(row).get(column).subList(0, timestep+1).toArray(new GRBVar[timestep+1]); // decisions to move to this cell, up until this point. Size is b/c timestep starts at 0
		
		try {
			sumOfDecisions = new GRBLinExpr();
			sumOfDecisions.addTerms(ones, decisionsUntilK);
			sumOfDecisions.addTerm(timestep+2, grid.get(row).get(column));	// if obstacle present
			GRBModel.addConstr(visits.get(row).get(column).get(timestep), GRB.EQUAL, sumOfDecisions, "visitscount_" + timestep+"row"+row+"column"+column); 
		} catch (GRBException e) {
			e.printStackTrace();
			System.out.println("Abnormal exception - exiting now.");
			System.exit(1);
		}
	}
	
	public void movementConstraints() {
		if(atFinish()) {
			stayStill();
		} else {
			try {
				if (row!=size-1)
					moveDown();
				if (column!=size-1) 
					moveRight();
				if (column!=0)
					moveLeft();
				if (row!=0)
					moveUp();
			} catch(GRBException e) {
				e.printStackTrace();
			}
		}
	}
	
	public boolean atFinish() {
		return (row == size-1 && column == 0);
	}
	
	public void stayStill() {
		GRBLinExpr nextPos = new GRBLinExpr();
		nextPos.addTerm(1.0, decisions.get(row).get(column).get(timestep+1));
		GRBLinExpr currentPos = new GRBLinExpr();
		currentPos.addTerm(1.0, decisions.get(row).get(column).get(timestep));
		try {
			GRBModel.addConstr(nextPos, GRB.GREATER_EQUAL, currentPos, "stayStillAtTimestep_" +timestep);
		} catch (GRBException e) {
			e.printStackTrace();
		}
	}
	
	public void moveDown() throws GRBException {
			GRBVar downLeqRight = standardBoolean(), downLeqLeft = standardBoolean(), downLeqUp = standardBoolean();
			
			GRBLinExpr downLeqRightExpr = new GRBLinExpr(), downLeqLeftExpr = new GRBLinExpr(), downLeqUpExpr = new GRBLinExpr();

			addTermsForAddition("down", downLeqRightExpr, downLeqLeftExpr, downLeqUpExpr);
			if (row == 0 && column == 0) {
				downLeqRightExpr.addTerm(-1.0, visits.get(row).get(column+1).get(timestep));
				downLeqUpExpr.addConstant(-1 * upperBound);
				downLeqLeftExpr.addConstant(-1 *upperBound);
			}
			else if (row == 0 && column == size-1) { 
				downLeqLeftExpr.addTerm(-1.0, visits.get(row).get(column-1).get(timestep));
				downLeqUpExpr.addConstant(-1*upperBound);
				downLeqRightExpr.addConstant(-1*upperBound);
			}
			else if (row == 0) {
					addNegations("right", downLeqRightExpr, "left", downLeqLeftExpr);
					downLeqUpExpr.addConstant(-1*upperBound);
			}
			else if (column==0) {
				addNegations("right", downLeqRightExpr, "up", downLeqUpExpr);
				downLeqLeftExpr.addConstant(-1*upperBound);
			}
			else if (column ==size-1) {
				addNegations("left", downLeqLeftExpr, "up", downLeqUpExpr);
				downLeqRightExpr.addConstant(-1*upperBound);
			}
			else 
				addNegations("right", downLeqRightExpr, "left", downLeqLeftExpr, "up", downLeqUpExpr);
			
			// TODO think of a way to get rid of these indicator constraints
			GRBModel.addGenConstrIndicator(downLeqRight, 1, downLeqRightExpr, GRB.LESS_EQUAL, 0, "1indicatorconstraintrow"+ row + "column" + column + "timestep" + timestep);
			GRBModel.addGenConstrIndicator(downLeqRight, 0, downLeqRightExpr, GRB.GREATER_EQUAL, 1,  "2indicatorconstraintrow"+ row + "column" + column + "timestep" + timestep);
			GRBModel.addGenConstrIndicator(downLeqLeft, 1, downLeqLeftExpr, GRB.LESS_EQUAL, 0,  "3indicatorconstraintrow"+ row + "column" + column + "timestep" + timestep);
			GRBModel.addGenConstrIndicator(downLeqLeft, 0, downLeqLeftExpr, GRB.GREATER_EQUAL, 1,  "4indicatorconstraintrow"+ row + "column" + column + "timestep" + timestep);
			GRBModel.addGenConstrIndicator(downLeqUp, 1, downLeqUpExpr, GRB.LESS_EQUAL, 0,  "5indicatorconstraintrow"+ row + "column" + column + "timestep" + timestep);
			GRBModel.addGenConstrIndicator(downLeqUp, 0, downLeqUpExpr, GRB.GREATER_EQUAL, 1,  "6indicatorconstraintrow"+ row + "column" + column + "timestep" + timestep);
		
			GRBVar[] toSatisfy = new GRBVar[] {downLeqRight, downLeqLeft, downLeqUp, decisions.get(row).get(column).get(timestep)/*, indicator*/};
			
			GRBVar satisfied = standardBoolean();
			
			GRBModel.addGenConstrAnd(satisfied, toSatisfy,  "1andRow" +row + "column" + column +"timestep" +timestep);
			
			GRBModel.addConstr(decisions.get(row + 1).get(column).get(timestep + 1), GRB.GREATER_EQUAL, satisfied, "moveDownRow_"+ row +"_Column_" + column + "_Timestep_" + timestep);
	}
	
	public void moveRight() throws GRBException {
			GRBVar rightLessThanDown = standardBoolean(), rightLeqLeft= standardBoolean(), rightLeqUp = standardBoolean();
			
			GRBLinExpr rightLessThanDownExpr = new GRBLinExpr(), rightLeqLeftExpr = new GRBLinExpr(), rightLeqThanUpExpr = new GRBLinExpr();

			rightLessThanDownExpr.addConstant(1.0);

			addTermsForAddition("right", rightLessThanDownExpr, rightLeqLeftExpr, rightLeqThanUpExpr);
			
			if (row == 0 && column == 0) {
				rightLessThanDownExpr.addTerm(-1.0, visits.get(row+1).get(column).get(timestep));
				rightLeqLeftExpr.addConstant(-1*upperBound);
				rightLeqThanUpExpr.addConstant(-1*upperBound);
			}
			else if (row == size-1 && column == 0) {
				rightLeqThanUpExpr.addTerm(-1.0, visits.get(row-1).get(column).get(timestep));
				rightLessThanDownExpr.addConstant(-1*upperBound);
				rightLeqLeftExpr.addConstant(-1*upperBound);
			}
			else if (row==0) {
				addNegations("down", rightLessThanDownExpr, "left", rightLeqLeftExpr);
				rightLeqThanUpExpr.addConstant(-1*upperBound);
			}
			else if (column==0) {
				addNegations("up",  rightLeqThanUpExpr, "down", rightLessThanDownExpr);
				rightLeqLeftExpr.addConstant(-1*upperBound);
			}
			else if (row ==size-1) {
				addNegations("up", rightLeqThanUpExpr, "left", rightLeqLeftExpr);
				rightLessThanDownExpr.addConstant(-1*upperBound);
			}
			else
				addNegations("down", rightLessThanDownExpr, "left", rightLeqLeftExpr, "up", rightLeqThanUpExpr);
			
			GRBModel.addGenConstrIndicator(rightLessThanDown, 1, rightLessThanDownExpr, GRB.LESS_EQUAL, 0,  "7indicatorconstraintrow"+ row + "column" + column + "timestep" + timestep);
			GRBModel.addGenConstrIndicator(rightLessThanDown, 0, rightLessThanDownExpr, GRB.GREATER_EQUAL, 1, "8indicatorconstraintrow"+ row + "column" + column + "timestep" + timestep);
			GRBModel.addGenConstrIndicator(rightLeqLeft, 1, rightLeqLeftExpr, GRB.LESS_EQUAL, 0,  "9indicatorconstraintrow"+ row + "column" + column + "timestep" + timestep);
			GRBModel.addGenConstrIndicator(rightLeqLeft, 0, rightLeqLeftExpr, GRB.GREATER_EQUAL, 1,  "10indicatorconstraintrow"+ row + "column" + column + "timestep" + timestep);
			GRBModel.addGenConstrIndicator(rightLeqUp, 1, rightLeqThanUpExpr, GRB.LESS_EQUAL, 0,  "11indicatorconstraintrow"+ row + "column" + column + "timestep" + timestep);
			GRBModel.addGenConstrIndicator(rightLeqUp, 0, rightLeqThanUpExpr, GRB.GREATER_EQUAL, 1,  "12indicatorconstraintrow"+ row + "column" + column + "timestep" + timestep);
			
			
			GRBVar[] toSatisfy = new GRBVar[] {rightLessThanDown, rightLeqLeft, rightLeqUp, decisions.get(row).get(column).get(timestep)/*, indicator*/};
			
			GRBVar satisfied = standardBoolean();
			
			GRBModel.addGenConstrAnd(satisfied, toSatisfy, "2andRow" + row + "column" + column +"timestep" + timestep);
			
			GRBModel.addConstr(decisions.get(row).get(column + 1).get(timestep + 1), GRB.GREATER_EQUAL, satisfied,  "moveRightRow_"+ row +"_Column_" + column + "_Timestep_" + timestep);
	}
	
	public void moveLeft() throws GRBException {
			GRBVar leftLessThanDown = standardBoolean(), leftLessThanRight = standardBoolean(), leftLeqUp = standardBoolean();
			
			GRBLinExpr leftLessThanDownExpr = new GRBLinExpr(), leftLessThanRightExpr = new GRBLinExpr(), leftLeqUpExpr = new GRBLinExpr();

			leftLessThanDownExpr.addConstant(1.0);
			leftLessThanRightExpr.addConstant(1.0);
			
			addTermsForAddition("left", leftLessThanDownExpr, leftLessThanRightExpr, leftLeqUpExpr);
			
			if (row == 0 && column == size-1) {
				leftLessThanDownExpr.addTerm(-1.0, visits.get(row+1).get(column).get(timestep));
				leftLessThanRightExpr.addConstant(-1*upperBound);
				leftLeqUpExpr.addConstant(-1*upperBound);
			} else if (row == size-1 && column == size-1) {
				leftLeqUpExpr.addTerm(-1.0, visits.get(row-1).get(column).get(timestep));
				leftLessThanDownExpr.addConstant(-1*upperBound);
				leftLessThanRightExpr.addConstant(-1*upperBound);
			}
			else if (row==0) {
				addNegations("down", leftLessThanDownExpr, "right", leftLessThanRightExpr);
				leftLeqUpExpr.addConstant(-1*upperBound);
			}
			else if (column==size-1) {
				addNegations("up",  leftLeqUpExpr, "down", leftLessThanDownExpr);
				leftLessThanRightExpr.addConstant(-1*upperBound);
			}
			else if (row ==size-1) {
				addNegations("up",  leftLeqUpExpr, "right", leftLessThanRightExpr);
				leftLessThanDownExpr.addConstant(-1*upperBound);
			}
			else
				addNegations("down", leftLessThanDownExpr, "right", leftLessThanRightExpr, "up", leftLeqUpExpr);

			GRBModel.addGenConstrIndicator(leftLessThanDown, 1, leftLessThanDownExpr, GRB.LESS_EQUAL, 0, "13indicatorconstraintrow"+ row + "column" + column + "timestep" + timestep);
			GRBModel.addGenConstrIndicator(leftLessThanDown, 0, leftLessThanDownExpr, GRB.GREATER_EQUAL, 1, "14indicatorconstraintrow"+ row + "column" + column + "timestep" + timestep);
			GRBModel.addGenConstrIndicator(leftLessThanRight, 1, leftLessThanRightExpr, GRB.LESS_EQUAL, 0, "15indicatorconstraintrow"+ row + "column" + column + "timestep" + timestep);
			GRBModel.addGenConstrIndicator(leftLessThanRight, 0, leftLessThanRightExpr, GRB.GREATER_EQUAL, 1, "16indicatorconstraintrow"+ row + "column" + column + "timestep" + timestep);
			GRBModel.addGenConstrIndicator(leftLeqUp, 1, leftLeqUpExpr, GRB.LESS_EQUAL, 0, "17indicatorconstraintrow"+ row + "column" + column + "timestep" + timestep);
			GRBModel.addGenConstrIndicator(leftLeqUp, 0, leftLeqUpExpr, GRB.GREATER_EQUAL, 1, "18indicatorconstraintrow"+ row + "column" + column + "timestep" + timestep);
				
			GRBVar[] toSatisfy = new GRBVar[] {	leftLessThanDown, leftLessThanRight, leftLeqUp, decisions.get(row).get(column).get(timestep), /*indicator*/};
			
			GRBVar satisfied = standardBoolean();
			
			//TODO if we were to remove decisions as we have emailed david, moveleft could simply be what satisfied is now.
			
			GRBModel.addGenConstrAnd(satisfied, toSatisfy, "3andRow" + row + "column" + column +"timestep" + timestep);
			
			GRBModel.addConstr(decisions.get(row).get(column -1).get(timestep + 1), GRB.GREATER_EQUAL, satisfied,  "moveLeftRow_"+ row +"_Column_" + column + "_Timestep_" + timestep);
	}
	
	public void moveUp() throws GRBException {
			GRBVar upLessThanDown = standardBoolean(), upLessThanRight = standardBoolean(), upLessThanLeft = standardBoolean();
			
			GRBLinExpr upLessThanDownExpr = new GRBLinExpr(), upLessThanRightExpr = new GRBLinExpr(), upLessThanLeftExpr = new GRBLinExpr();
			
			upLessThanDownExpr.addConstant(1.0);
			upLessThanRightExpr.addConstant(1.0);
			upLessThanLeftExpr.addConstant(1.0);
			
			addTermsForAddition("up", upLessThanDownExpr, upLessThanRightExpr, upLessThanLeftExpr);
			
			if (row == size-1 && column == 0) {
				upLessThanRightExpr.addTerm(-1.0, visits.get(row).get(column+1).get(timestep));
				upLessThanDownExpr.addConstant(-1*upperBound);
				upLessThanLeftExpr.addConstant(-1*upperBound);
			}
			else if (row == size-1 && column == size-1) { 
				upLessThanLeftExpr.addTerm(-1.0, visits.get(row).get(column - 1).get(timestep));
				upLessThanRightExpr.addConstant(-1*upperBound);
				upLessThanDownExpr.addConstant(-1*upperBound);
			}
			else if (column==0) {
				addNegations("down", upLessThanDownExpr, "right", upLessThanRightExpr);
				upLessThanLeftExpr.addConstant(-1*upperBound);
			}
			else if (column==size-1) {
				addNegations("left",  upLessThanLeftExpr, "down", upLessThanDownExpr);
				upLessThanRightExpr.addConstant(-1*upperBound);
			}
			else if (row ==size-1) {
				addNegations("right",  upLessThanRightExpr, "left", upLessThanLeftExpr);
				upLessThanDownExpr.addConstant(-1*upperBound);
			}
			else
				addNegations("down", upLessThanDownExpr, "right", upLessThanRightExpr, "left", upLessThanLeftExpr);
			
			GRBModel.addGenConstrIndicator(upLessThanDown, 1, upLessThanDownExpr, GRB.LESS_EQUAL, 0,"19indicatorconstraintrow"+ row + "column" + column + "timestep" + timestep);
			GRBModel.addGenConstrIndicator(upLessThanDown, 0, upLessThanDownExpr, GRB.GREATER_EQUAL, 1, "20indicatorconstraintrow"+ row + "column" + column + "timestep" + timestep);
			GRBModel.addGenConstrIndicator(upLessThanRight, 1, upLessThanRightExpr, GRB.LESS_EQUAL, 0, "21indicatorconstraintrow"+ row + "column" + column + "timestep" + timestep);
			GRBModel.addGenConstrIndicator(upLessThanRight, 0, upLessThanRightExpr, GRB.GREATER_EQUAL, 1, "22indicatorconstraintrow"+ row + "column" + column + "timestep" + timestep);
			GRBModel.addGenConstrIndicator(upLessThanLeft, 1, upLessThanLeftExpr, GRB.LESS_EQUAL, 0, "23indicatorconstraintrow"+ row + "column" + column + "timestep" + timestep);
			GRBModel.addGenConstrIndicator(upLessThanLeft, 0, upLessThanLeftExpr, GRB.GREATER_EQUAL, 1, "24indicatorconstraintrow"+ row + "column" + column + "timestep" + timestep);
			
			GRBVar[] toSatisfy = new GRBVar[] {	upLessThanDown, upLessThanRight, upLessThanLeft, decisions.get(row).get(column).get(timestep), /*indicator*/};
			
			GRBVar satisfied = standardBoolean();
			
			GRBModel.addGenConstrAnd(satisfied, toSatisfy, "4andRow" + row + "column" + column +"timestep" + timestep);
			
			GRBModel.addConstr(decisions.get(row-1).get(column).get(timestep + 1), GRB.GREATER_EQUAL, satisfied,   "moveUpRow_"+ row +"_Column_" + column + "_Timestep_" + timestep);
	}
	
	public void addNegations(String firstString, GRBLinExpr first, String secondString, GRBLinExpr second, String thirdString, GRBLinExpr third) {
		addTermForNegation(firstString, first);
		addTermForNegation(secondString, second);
		addTermForNegation(thirdString, third);
	}
	
	public void addNegations(String firstString, GRBLinExpr first, String secondString, GRBLinExpr second) {
		addTermForNegation(firstString, first);
		addTermForNegation(secondString, second);
	}
	
	// Adds visits of direction we are adding movement constraint for to expressions
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
			return GRBModel.addVar(0.0, 1.0, 0.0, GRB.BINARY, "standardbool" + Math.random());
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
	
//	public void moveToFinish() {
//		GRBLinExpr finish = new GRBLinExpr();
//		GRBLinExpr current = new GRBLinExpr();
//		current.addTerm(1.0, decisions.get(row).get(column).get(timestep));
//		finish.addTerm(1.0, decisions.get(size).get(1).get(timestep + 1));
//		try {
//			GRBModel.addConstr(finish, GRB.GREATER_EQUAL, current, "");
//		} catch (GRBException e) {
//			e.printStackTrace();
//		}
//	}
	
	// indicator variable is 1 if the sum is leq than 0, 0 if geq than 1
	public void greaterThan1Indicator(GRBVar indicator, GRBLinExpr sum) {
		try {
			GRBModel.addGenConstrIndicator(indicator, 1, sum, GRB.LESS_EQUAL, 0.0, "leq0" + Math.random());
			GRBModel.addGenConstrIndicator(indicator, 0, sum, GRB.GREATER_EQUAL, 1.0, "leq1" + Math.random());
		} catch (GRBException e) {
			e.printStackTrace();
		}
	}
	
	public void obstacleConstraints() {
		GRBLinExpr obstaclePlusMovement = new GRBLinExpr();
		
		obstaclePlusMovement.addTerm(1.0, grid.get(row).get(column));
		
		try {
			obstaclePlusMovement.addTerm(1.0,  decisions.get(row).get(column).get(timestep));
			GRBModel.addConstr(obstaclePlusMovement, GRB.LESS_EQUAL, 1, "obsplusmovementrow" + row + "column" + column + "timestep" + timestep);
		} catch (GRBException e) {
			e.printStackTrace();
		}
	}
	 
	public void transitiveClosureFeasibilityConstraints() throws GRBException {
		
		TkMinus1 = new GRBVar[size*size][size*size]; // pathtostart[0][i] (first row) represents the distance from 1,1 to all other vertices. when doing with size = 3, this will be a 9x9
		
		listOfNonAccessedVars = new ArrayList<GRBVar>();

		
		for (int row = 0; row < size*size; row ++) {
			for (int column = 0; column < size*size; column ++) {
				if (row != column) {
					TkMinus1[row][column] = GRBModel.addVar(0.0, 1.0, 0.0, GRB.BINARY, "FWmatrix row " + row + " column "+ column);
					listOfNonAccessedVars.add(TkMinus1[row][column]);

				} else {
					TkMinus1[row][column] = GRBModel.addVar(1.0, 1.0, 0.0, GRB.BINARY, "FWmatrix row " + row + " column " + column); // There is a path between i and i
				}
			}	
		}

		// do we have to say if we have set there to en edge, there is not an edge..  ??? for example at 1,1 there is no edge to 2,2 but we never say that it there is not an edge
		for (int row = 0; row < size * size; row++) {
			if (row < (size*size) - size) { // if we're not at the bottom row. this should not be true if we are at rows 2 and 3 for a 2x2
				addEdgeDown(TkMinus1[row][row+size], row);		// there is an edge between each pair of adjacent cells unless there is an obstacle
				listOfNonAccessedVars.remove(TkMinus1[row][row+size]);

			}
			if(row % size != size-1) { 
				addEdgeRight(TkMinus1[row][row+1], row);
				listOfNonAccessedVars.remove(TkMinus1[row][row+1]);
			}
			if(row % size != 0) {
				addEdgeLeft(TkMinus1[row][row-1], row);				
				listOfNonAccessedVars.remove(TkMinus1[row][row-1]);
			}
			if(row >= size) {
				addEdgeUp(TkMinus1[row][row-size], row);				
				listOfNonAccessedVars.remove(TkMinus1[row][row-size]);
			}
		}

		for (GRBVar var: listOfNonAccessedVars) {
			GRBModel.addConstr(var, GRB.LESS_EQUAL, 0, "nonaccessedVarConstraint" + Math.random());
		}
		
		TkMinus1Array = new GRBVar[size*size+1][size*size][size*size];
		TkMinus1Array[0] = TkMinus1;		
		// an edge exists if they are adjacent and neither has an obstacle. if an edge exists, variable is 1.
		for (int k = 0; k < size*size; k ++) {	// for every vertex, try to use it as an intermediate vertex between every other pair of vertices
			Tk = new GRBVar[size*size][size*size]; // pathtostart[0][i] (first row) represents the distance from 1,1 to all other vertices. when doing with size = 3, this will be a 9x9
			for (int i = 0; i < size*size; i ++) {
				for (int j = 0; j < size*size; j ++) {
					Tk[i][j] = GRBModel.addVar(0.0, 1.0, 0.0, GRB.BINARY, "Tk_row" + "_"+i+"_column_"+j);
					GRBVar Cijk = conjunction(k, i, j);
					GRBModel.addGenConstrOr(Tk[i][j], new GRBVar[] {Cijk, TkMinus1[i][j]},  "Tk_row" + "_"+i+"_column_"+j);
				}
			}
			TkMinus1 = Tk;

			TkMinus1Array[k+1] = TkMinus1;
		}
		GRBModel.addConstr(TkMinus1[(size*size)-size	][0], GRB.EQUAL, 1, "transitiveclosureconstraint");
//		GRBModel.addConstr(lhs, sense, rhsExpr, name)
//		return TkMinus1[(size*size)-size	][0];
	}
	
	void printTkMinus1Array() {
		for (int k = 0; k < size*size+1; k++) {
			for (int i = 0; i < size*size; i++) {
				for (int j = 0; j < size*size; j++) {
					try {
						System.out.print((int) TkMinus1Array[k][i][j].get(GRB.DoubleAttr.X) + " ");
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
		GRBVar Cijk = GRBModel.addVar(0.0, 1.0, 0.0, GRB.BINARY, "Cijk_row_"+ row +"_column_" + column + "_timestep_" + timestep);
		GRBVar tikMinus1 = TkMinus1[i][k];
		GRBVar tkjMinus1 = TkMinus1[k][j];
		GRBModel.addGenConstrAnd(Cijk, new GRBVar[] {tikMinus1, tkjMinus1}, "CijkConstraint_row"+ row+ "_column_" + column +"_timestep_" + timestep);
		return Cijk;
	}
	
	void addEdgeDown(GRBVar currentVar, int row) throws GRBException {
		GRBLinExpr edgeExists = new GRBLinExpr(), edgeExists2= new GRBLinExpr(), edgeExists3= new GRBLinExpr();
		edgeExists.addConstant(1);
		edgeExists.addTerm(-1,grid.get(row/size + 1).get(row%size));
		
		edgeExists2.addConstant(1);
		edgeExists2.addTerm(-1, grid.get(row/size).get(row%size));
		
		GRBModel.addConstr(currentVar, GRB.LESS_EQUAL, edgeExists, "duncan");
		GRBModel.addConstr(currentVar, GRB.LESS_EQUAL, edgeExists2, "duncan");
		edgeExists3.addConstant(1.0);
		edgeExists3.addTerms(new double[] {-1,-1}, new GRBVar[] {grid.get(row/size).get(row%size), grid.get(row/size + 1).get(row%size)});
		GRBModel.addConstr(currentVar, GRB.GREATER_EQUAL, edgeExists3, "edgedown");
	}

	// grid contains edges adjacent to grid so just doing grid.get(row+whichRow).get(column) will not work as we get the two edges as well
	// if column%size == 0 || column % size+1 == 0 - nope
	void addEdgeRight(GRBVar currentVar, int row) throws GRBException {
		GRBLinExpr edgeExists = new GRBLinExpr(), edgeExists2= new GRBLinExpr(), edgeExists3= new GRBLinExpr();
		edgeExists.addConstant(1);
		edgeExists.addTerm(-1,grid.get(row/size).get(row%size));
		
		edgeExists2.addConstant(1);
		edgeExists2.addTerm(-1, grid.get(row/size).get(row%size + 1));
		
		GRBModel.addConstr(currentVar, GRB.LESS_EQUAL, edgeExists, "duncan");
		GRBModel.addConstr(currentVar, GRB.LESS_EQUAL, edgeExists2, "duncan");
		edgeExists3.addConstant(1.0);
		edgeExists3.addTerms(new double[] {-1,-1}, new GRBVar[] {grid.get(row/size).get(row%size), grid.get(row/size).get(row%size + 1)});
		GRBModel.addConstr(currentVar, GRB.GREATER_EQUAL, edgeExists3, "edgedown");
	}

	void addEdgeLeft(GRBVar currentVar, int row) throws GRBException {
		GRBLinExpr edgeExists = new GRBLinExpr(), edgeExists2= new GRBLinExpr(), edgeExists3= new GRBLinExpr();
		edgeExists.addConstant(1);
		edgeExists.addTerm(-1,grid.get(row/size).get(row%size));
		
		edgeExists2.addConstant(1);
		edgeExists2.addTerm(-1, grid.get(row/size).get(row%size-1));
		
		GRBModel.addConstr(currentVar, GRB.LESS_EQUAL, edgeExists, "duncan");
		GRBModel.addConstr(currentVar, GRB.LESS_EQUAL, edgeExists2, "duncan");
		edgeExists3.addConstant(1.0);
		edgeExists3.addTerms(new double[] {-1,-1}, new GRBVar[] {grid.get(row/size).get(row%size), grid.get(row/size).get(row%size-1)});
		GRBModel.addConstr(currentVar, GRB.GREATER_EQUAL, edgeExists3, "edgedown");
	}

	void addEdgeUp(GRBVar currentVar, int row) throws GRBException {		
		
		GRBLinExpr edgeExists = new GRBLinExpr(), edgeExists2= new GRBLinExpr(), edgeExists3= new GRBLinExpr();
		edgeExists.addConstant(1);
		edgeExists.addTerm(-1,grid.get(row/size).get(row%size));
		
		edgeExists2.addConstant(1);
		edgeExists2.addTerm(-1, grid.get(row/size-1).get(row%size));
		
		GRBModel.addConstr(currentVar, GRB.LESS_EQUAL, edgeExists, "duncan");
		GRBModel.addConstr(currentVar, GRB.LESS_EQUAL, edgeExists2, "duncan");
		edgeExists3.addConstant(1.0);
		edgeExists3.addTerms(new double[] {-1,-1}, new GRBVar[] {grid.get(row/size).get(row%size), grid.get(row/size-1).get(row%size)});
		GRBModel.addConstr(currentVar, GRB.GREATER_EQUAL, edgeExists3, "edgedown");
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
	
	public void basicMazeFeasibilityConstraints() {
		GRBLinExpr feasibilityConstraint = new GRBLinExpr();
		feasibilityConstraint.addTerm(1.0,  decisions.get(size-1).get(0).get((int) upperBound-1));
		
		try {
			GRBModel.addConstr(feasibilityConstraint, GRB.GREATER_EQUAL, 1, "");
		} catch (GRBException e) {
			e.printStackTrace();
		}
	}
	
	public void orderNSquaredConstraints() { 
		// first dimension is the row index, second is the column index and third is the timestep (not our usual timestep)
		GRBVar[][][] feasibilityVariables = new GRBVar[size][size][size*size];

		for (int i = 0; i < size; i++) {
			for (int j = 0; j < size; j++) {
				for (int k = 0; k < size*size; k++) {
					try {
						if (k!=0) {
							feasibilityVariables[i][j][k] = GRBModel.addVar(0.0, 1.0, 0.0, GRB.BINARY, "feasibilityVariables_row_" + row + "column"+ column + "Timestep"+ k+ Math.random());
						} else {
							if (i==0 && j == 0) {
								feasibilityVariables[i][j][k] = GRBModel.addVar(1.0, 1.0, 1.0, GRB.BINARY, "feasibilityVariables_row_" + row + "column"+ column + "Timestep"+ k+ Math.random());
							} else {
								feasibilityVariables[i][j][k] = GRBModel.addVar(0.0, 0.0, 0.0, GRB.BINARY,  "feasibilityVariables_row_" + row + "column"+ column + "Timestep"+ k + Math.random());
							}
						}
						GRBLinExpr obstacleConstraint = new GRBLinExpr();
						obstacleConstraint.addConstant(1.0);
						obstacleConstraint.addTerm(-1.0, grid.get(i).get(j));
						GRBModel.addConstr(feasibilityVariables[i][j][k], GRB.LESS_EQUAL, obstacleConstraint, "obsConstrForRow"+ row + "Column"+column + "Timestep" + k + Math.random());
					} catch (GRBException e) {
						e.printStackTrace();
					}
				}
			}
		}

		for (int i = 0; i < size; i++) {
			for (int j = 0; j < size; j++) {
				for (int k = 1; k < size*size; k++) {
					try {
						GRBVar[] orCondition = new GRBVar[5];
						GRBVar orVariable = GRBModel.addVar(0.0, 1.0, 0.0, GRB.BINARY, "milne"+Math.random());
						orCondition[0] = feasibilityVariables[i][j][k-1];
						if (i>0)
							orCondition[1] = feasibilityVariables[i-1][j][k-1];
						else
							orCondition[1] = GRBModel.addVar(0.0,0.0,0.0,GRB.BINARY,"milne"+Math.random());
						if (i != size-1)
							orCondition[2] = feasibilityVariables[i+1][j][k-1];
						else 
							orCondition[2] = GRBModel.addVar(0.0,0.0,0.0,GRB.BINARY,"milne"+Math.random());
						if (j > 0)
							orCondition[3] = feasibilityVariables[i][j-1][k-1];	
						else 
							orCondition[3] = GRBModel.addVar(0.0,0.0,0.0,GRB.BINARY,"milne" + Math.random());
						if (j != size-1)
							orCondition[4] = feasibilityVariables[i][j+1][k-1];
						else 
							orCondition[4] = GRBModel.addVar(0.0,0.0,0.0,GRB.BINARY,"milne" + Math.random());
						
						GRBModel.addGenConstrOr(orVariable, orCondition, "orI"+ i + "j" + j + "k"+ k);
						GRBModel.addConstr(feasibilityVariables[i][j][k], GRB.LESS_EQUAL, orVariable, "orConstrI"+ i + "j" + j + "k"+ k);
						
						GRBLinExpr kPlusOneConstraint = new GRBLinExpr();
						kPlusOneConstraint.addTerm(-1.0, grid.get(i).get(j));
						kPlusOneConstraint.addTerm(1.0, orVariable);
						GRBModel.addConstr(feasibilityVariables[i][j][k], GRB.GREATER_EQUAL, kPlusOneConstraint,"kPlusONeConstraintI"+ i + "j" + j + "k"+ k);
						
					} catch (GRBException e) {
						e.printStackTrace();
					}
				}
			}
		}
		
		try {
			GRBModel.addConstr(feasibilityVariables[size-1][0][(size*size)-1], GRB.EQUAL, 1, "NewFeasibilityConstraint");
		} catch (GRBException e) {
			e.printStackTrace();
		}
	}
	
	// At timestep k we sum up all of squeakyDecision variables at k and make sure they are leq than 1
	public void oneMoveAtATime() {
		GRBLinExpr oneMovePerK;
		
		for(timestep = 0; timestep < upperBound; timestep++) {
			oneMovePerK = new GRBLinExpr();
			for (row = 0; row < size; row++) {
				for (column = 0; column < size; column++) {
					oneMovePerK.addTerm(1.0, decisions.get(row).get(column).get(timestep));
				}
			}
			try {
				GRBModel.addConstr(oneMovePerK, GRB.EQUAL, 1, "onemoveatatimeI" + row + "j"+ column +"k" +timestep);
			} catch (GRBException e) {
				e.printStackTrace();
			}
		}
	}
	
	// Trying to maximise the number of timesteps squeaky is not at n,1
		public void setObjectiveFunction() {
			
			objective = new GRBLinExpr();
			
			objective.addTerm(-1.0, visits.get(size-1).get(0).get((int) upperBound-1));
			objective.addConstant(upperBound);
//			for (int timestep = 0; timestep < upperBound; timestep++) {
//				objective.addConstant(1.0);
//				objective.addTerm(-1.0, decisions.get(size-1).get(0).get(timestep));
//			}
			try {
				if (score>0)
					GRBModel.addConstr(objective, GRB.GREATER_EQUAL, score, "objectivefunction");
				GRBModel.setObjective(objective, GRB.MAXIMIZE);
			} catch (GRBException e) {
				e.printStackTrace();
			}
		}
}
