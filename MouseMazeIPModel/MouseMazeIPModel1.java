import java.util.ArrayList;
import java.util.Arrays;

import gurobi.GRB;
import gurobi.GRBConstr;
import gurobi.GRBEnv;
import gurobi.GRBException;
import gurobi.GRBLinExpr;
import gurobi.GRBModel;
import gurobi.GRBVar;


public class MouseMazeIPModel1 {

	private int size, score, timeInMinutes;
	double upperBound;

	private String fileToPrintTo;

	private ArrayList<ArrayList<GRBVar>> grid;

	public GRBEnv GRBEnv;

	public GRBModel gRBModel;
	GRBLinExpr T;
	private ArrayList<ArrayList<ArrayList<GRBVar>>> squeakysDecisions;
	private ArrayList<ArrayList<ArrayList<GRBVar>>> visits;

	GRBLinExpr lessThanOne = new GRBLinExpr();
	
	// Iterator variables
	int i,j,k;

	GRBConstr cons;
	
	GRBVar toStore1, toStore2, toStore3, toStore4,visitsiplus1jk, visitsijplus1k;
	
	GRBLinExpr storeExpr;
	
	public MouseMazeIPModel1(int size, int score, int timeInMinutes, String fileToPrintTo) {
		try {
			GRBEnv = new GRBEnv();
			gRBModel = new GRBModel(GRBEnv);
		} catch (GRBException e) {
			e.printStackTrace();
		}
		this.size = size;
		this.score = score;
		this.timeInMinutes = timeInMinutes;
		this.fileToPrintTo = fileToPrintTo;
		this.upperBound = 20;
		lessThanOne.addConstant(1.0);
	}

	public void createGrid() throws GRBException {

		grid = new ArrayList<ArrayList<GRBVar>>();

		ArrayList<GRBVar> row = new ArrayList<GRBVar>();

		// Will set edges to be infty when setting visits for all
		for (i = 0; i < size + 2; i++) { // For each row
			row = new ArrayList<GRBVar>();
			for (j = 0; j < size + 2; j++) { // For each column
				// Create var at start and end of each row and set visits to \infty
				GRBVar currentGRBVar;
				if (i!=2 || j!=1)
					currentGRBVar = gRBModel.addVar(0.0, 1.0, 0.0, GRB.BINARY, "Obstacle in row " + i + " and column " + j);
				else
					currentGRBVar = gRBModel.addVar(1.0, 1.0, 0.0, GRB.BINARY, "Obstacle in row " + i + " and column " + j);
				row.add(currentGRBVar); // create row of gurobi variables
			}
			grid.add(row);
		}
	}

	public void createVisits() throws GRBException {
		visits = new ArrayList<ArrayList<ArrayList<GRBVar>>>();

		ArrayList<GRBVar> column;

		ArrayList<ArrayList<GRBVar>> row;

		GRBVar currentGRBVar;

		for (i = 0; i < size + 2; i++) {
			row = new ArrayList<ArrayList<GRBVar>>();
			for (j = 0; j < size + 2; j++) {
				column = new ArrayList<GRBVar>();
				for (k = 0; k < upperBound; k++) {
					if (i == 0 || i == size + 1 || j == 0 || j == size + 1) {	// if cell is an edge, set visits to \infty
						currentGRBVar = gRBModel.addVar(upperBound + 1, upperBound + 1, 0.0, GRB.INTEGER,
								"Visits for row " + i + " and column " + j);
					} else if (k == 0 && i == 1 && j == 1) {	// set cell 1, 1 to be 1 and only one for first timestep
						currentGRBVar = gRBModel.addVar(1.0, 1.0, 0.0, GRB.BINARY, "");
					} else {	// otherwise it is a good cell whose visits can be between 0 and the upperbound
						currentGRBVar = gRBModel.addVar(0.0, upperBound, 0.0, GRB.INTEGER,
								"Visits for row " + i + " and column " + j);
					}
					column.add(currentGRBVar);
				}
				row.add(column);
			}
			visits.add(row);
		}
	}

	public void createSqueakysDecisionVariables() throws GRBException {

		squeakysDecisions = new ArrayList<ArrayList<ArrayList<GRBVar>>>();
		
		GRBVar currentGRBVar;

		ArrayList<ArrayList<GRBVar>> columns;
		ArrayList<GRBVar> timesteps;

		for (i = 0; i < size + 2; i++) { // For every cell
			columns = new ArrayList<ArrayList<GRBVar>>();
			for (j = 0; j < size + 2; j++) {
				timesteps = new ArrayList<GRBVar>();
				for (k = 0; k < upperBound; k++) { // for every timestep
					if (k == 0 && i == 1 && j == 1) {	// if its first timestep at cell 1,1
						currentGRBVar = gRBModel.addVar(1.0, 1.0, 0.0, GRB.BINARY, "");
						//	currentGRBVar.set(GRB.DoubleAttr.Start, 1.0);
					} else {
						currentGRBVar = gRBModel.addVar(0.0, 1.0, 0.0, GRB.BINARY,
								"Row " + i + " column " + j + " timestep " + k);
					}
					timesteps.add(currentGRBVar);
				}
				columns.add(timesteps);
			}
			squeakysDecisions.add(columns);
		}
	}

	public void addConstraints() {
		// This will iterate over all non edge cells of the grid
		for (i = 1; i < size+1; i++) {
			for (j = 1; j < size+1; j++) {
				for (k = 0; k < upperBound; k++) {
					visitsConstraints();
					if (k!=upperBound-1) { // we do not need to consider when k = upperbound-1 as we are adding constraint saying k+1 is geq than k.. so we already do it for upperbound-1
						movementConstraints();
					} 
					obstacleConstraints();
				}
			}
		}

		mazeFeasibilityConstraints();
		oneMoveAtATime();
	}
	
	// This method is just a way of counting the number of moves squeaky has made at cell i,j at timestep k by counting all times squeaky has moved to i,j up until this timestep
	public void visitsConstraints() {
		
		GRBLinExpr sumOfSqueakysDecisions = new GRBLinExpr();

		double[] ones;

		// do k+1 as squeakysDecisions.get(i).get(j).get(0) represents squeakys decision at time 0. e.g. at timestep 1 we want to add his position at timestep 0 and timestep 1.
		ones = new double[k+1];
		Arrays.fill(ones, 1.0);
		
		GRBVar[] squeakysDecisionsUntilTimeK = squeakysDecisions.get(i).get(j).subList(0, k+1).toArray(new GRBVar[k+1]); // get first k vars from this to count how many times he has moved to i,j

		try {
			sumOfSqueakysDecisions.addTerms(ones, squeakysDecisionsUntilTimeK);
			gRBModel.addConstr(visits.get(i).get(j).get(k), GRB.EQUAL, sumOfSqueakysDecisions, "d"); // timestep k of cell i,j added to lhs. sum of all mouse movements to i,j for all k' up to k thus far...
		} catch (GRBException e1) {
			e1.printStackTrace();
		}
	}
	
	public void obstacleConstraints() {
		// iterate over G for all i j and 
		GRBLinExpr neverVisitObstacle = new GRBLinExpr();
		
		neverVisitObstacle.addTerm(1.0, grid.get(i).get(j));
		
		try {
			neverVisitObstacle.addTerm(1.0, squeakysDecisions.get(i).get(j).get(k));
			gRBModel.addConstr(neverVisitObstacle, GRB.LESS_EQUAL, lessThanOne, "less than one");
		} catch (GRBException e) {
			e.printStackTrace();
			System.exit(1);
		}
	}

	// TODO - current prob is that we want squeaky to move to finish square if he is adjacent to it but we do not want squeaky to check whether he should move to the finish square when he is at the finish square
	public void movementConstraints() {
		// Do we instead have to say, dont do movedown if at n-1,1. dont do moveleft if at n,j+1 etc..
		if (i!=size || j!=1) {  // don't consider moving if we're at the finish square
			moveDown();
			moveRight();
			moveLeft();
			moveUp();
		} else {
			GRBLinExpr LHS = new GRBLinExpr();
			LHS.addTerm(1.0, squeakysDecisions.get(i).get(j).get(k+1));
			GRBLinExpr RHS = new GRBLinExpr();
			RHS.addTerm(1.0, squeakysDecisions.get(i).get(j).get(k));
			try {
				gRBModel.addConstr(LHS, GRB.GREATER_EQUAL, RHS, "");
			} catch (GRBException e) {
				e.printStackTrace();
			}
		}
	}
	
	public void moveDown() {
		try {

			if (i!=size-1 || j != 1) { // if we are one above finish square we just move down
				GRBVar DLETR = gRBModel.addVar(0.0, 1.0, 0.0, GRB.BINARY, ""), DLETL = gRBModel.addVar(0.0, 1.0, 0.0, GRB.BINARY, ""), DLETU = gRBModel.addVar(0.0, 1.0, 0.0, GRB.BINARY, "");
				
				GRBLinExpr DLTRExpr = new GRBLinExpr(), DLTLExpr = new GRBLinExpr(), DLTUExpr = new GRBLinExpr();
				
				DLTRExpr.addTerm(1.0, visits.get(i+1).get(j).get(k));
				DLTRExpr.addTerm(-1.0, visits.get(i).get(j+1).get(k));
				
				DLTLExpr.addTerm(1.0, visits.get(i+1).get(j).get(k));
				DLTLExpr.addTerm(-1.0, visits.get(i).get(j-1).get(k));
				
				DLTUExpr.addTerm(1.0, visits.get(i+1).get(j).get(k));
				DLTUExpr.addTerm(-1.0, visits.get(i-1).get(j).get(k));
				
				gRBModel.addGenConstrIndicator(DLETR, 1, DLTRExpr, GRB.LESS_EQUAL, 0.0, ""); // If DLETR is 1 then the following linear constraint must hold: v_{i+1,j,k} - v_{i,j+1,k} <= 0.
				gRBModel.addGenConstrIndicator(DLETR, 0, DLTRExpr, GRB.GREATER_EQUAL, 1, "");
		
				gRBModel.addGenConstrIndicator(DLETL, 1, DLTLExpr, GRB.LESS_EQUAL, 0.0, "");
				gRBModel.addGenConstrIndicator(DLETL, 0, DLTLExpr, GRB.GREATER_EQUAL, 1.0, "");
				
				gRBModel.addGenConstrIndicator(DLETU, 1, DLTUExpr, GRB.LESS_EQUAL, 0.0, "");
				gRBModel.addGenConstrIndicator(DLETU, 0, DLTUExpr, GRB.GREATER_EQUAL, 1.0, "");
	
				
				GRBVar[] toSatisfy = new GRBVar[] {DLETR, DLETL, DLETU, squeakysDecisions.get(i).get(j).get(k)};
					
				GRBVar temp = gRBModel.addVar(0.0, 1.0, 0.0, GRB.BINARY, "");
				gRBModel.addGenConstrAnd(temp, toSatisfy, "down less than up");
				
				// we need something to be 1 if temp is 1 and grid is 0 or 0 if grid otherwise
				GRBVar indicator = gRBModel.addVar(0.0, 1.0, 0.0, GRB.BINARY, "");
				GRBLinExpr tempExpr = new GRBLinExpr();
				tempExpr.addTerm(1.0, temp);
				tempExpr.addTerm(-1.0, grid.get(i+1).get(j));
				
				gRBModel.addGenConstrIndicator(indicator, 1, tempExpr, GRB.GREATER_EQUAL, 1, "");
				gRBModel.addGenConstrIndicator(indicator, 0, tempExpr, GRB.LESS_EQUAL, 0, "");
				
				gRBModel.addConstr(squeakysDecisions.get(i+1).get(j).get(k+1), GRB.GREATER_EQUAL, indicator, "");
			} else { // if we at n-1,1
				GRBLinExpr LHSS = new GRBLinExpr();
				LHSS.addTerm(1.0, squeakysDecisions.get(i+1).get(j).get(k+1));
				GRBLinExpr rightHandSide = new GRBLinExpr();
				rightHandSide.addTerm(1.0, squeakysDecisions.get(i).get(j).get(k));
				gRBModel.addConstr(LHSS, GRB.GREATER_EQUAL, rightHandSide, "");
			}
				
		} catch (GRBException e) {
			e.printStackTrace();
		}
	}
	
	public void moveRight() {
		
		try {
			if (i!=size || j!=0) { //we do not consider moving right when at the finish square, we simply stay still.
				GRBVar RLTD = gRBModel.addVar(0.0, 1.0, 0.0, GRB.BINARY, ""), RLETL = gRBModel.addVar(0.0, 1.0, 0.0, GRB.BINARY, ""), RLETU = gRBModel.addVar(0.0, 1.0, 0.0, GRB.BINARY, "");
				
				GRBLinExpr RLTDExpr = new GRBLinExpr(), RLETLExpr = new GRBLinExpr(), RLETUExpr = new GRBLinExpr();
				RLTDExpr.addTerm(1.0, visits.get(i).get(j+1).get(k));
				RLTDExpr.addTerm(-1.0, visits.get(i+1).get(j).get(k));
				RLTDExpr.addConstant(1.0); // adding one so we can use GRB.LESS_EQUAL as u cant do just less than
				
				RLETLExpr.addTerm(1.0, visits.get(i).get(j+1).get(k));
				RLETLExpr.addTerm(-1.0, visits.get(i).get(j-1).get(k));
				
				RLETUExpr.addTerm(1.0, visits.get(i).get(j+1).get(k));
				RLETUExpr.addTerm(-1.0, visits.get(i-1).get(j).get(k));
				
				gRBModel.addGenConstrIndicator(RLTD, 1, RLTDExpr, GRB.LESS_EQUAL, 0.0, ""); // If DLETR is 1 then the following linear constraint must hold: v_{i+1,j,k} - v_{i,j+1,k} <= 0.
				gRBModel.addGenConstrIndicator(RLTD, 0, RLTDExpr, GRB.GREATER_EQUAL, 1, "");
				
				gRBModel.addGenConstrIndicator(RLETL, 1, RLETLExpr, GRB.LESS_EQUAL, 0.0, ""); // If DLETR is 1 then the following linear constraint must hold: v_{i+1,j,k} - v_{i,j+1,k} <= 0.
				gRBModel.addGenConstrIndicator(RLETL, 0, RLETLExpr, GRB.GREATER_EQUAL, 1, "");
				
				gRBModel.addGenConstrIndicator(RLETU, 1, RLETUExpr, GRB.LESS_EQUAL, 0.0, ""); // If DLETR is 1 then the following linear constraint must hold: v_{i+1,j,k} - v_{i,j+1,k} <= 0.
				gRBModel.addGenConstrIndicator(RLETU, 0, RLETUExpr, GRB.GREATER_EQUAL, 1, "");
				
				
				GRBVar temp = gRBModel.addVar(0.0, 1.0, 0.0, GRB.BINARY, "");
				gRBModel.addGenConstrAnd(temp, new GRBVar[] {RLTD, RLETL, RLETU, squeakysDecisions.get(i).get(j).get(k)}, "down less than up");
				gRBModel.addConstr(squeakysDecisions.get(i).get(j+1).get(k+1), GRB.GREATER_EQUAL, temp, "");
			} else { //if we at n,0
				GRBLinExpr LHSS = new GRBLinExpr();
				LHSS.addTerm(1.0, squeakysDecisions.get(i).get(j+1).get(k+1));
				GRBLinExpr rightHandSide = new GRBLinExpr();
				rightHandSide.addTerm(1.0, squeakysDecisions.get(i).get(j).get(k));
				gRBModel.addConstr(LHSS, GRB.GREATER_EQUAL, rightHandSide, "");
			}
		} catch (GRBException e) {
			e.printStackTrace();
		}
	}
	
	public void moveLeft() { //return GRBVar determining whether or not squeaky moves this direction

		try {

			if (i!=size || j != 2) { // do not consider moving left when at finish square, just do it.
				GRBVar LLTD = gRBModel.addVar(0.0, 1.0, 0.0, GRB.BINARY, ""), LLTR = gRBModel.addVar(0.0, 1.0, 0.0, GRB.BINARY, ""), LLETU = gRBModel.addVar(0.0, 1.0, 0.0, GRB.BINARY, "");
				
				GRBLinExpr LLTDExpr = new GRBLinExpr(), LLTRExpr = new GRBLinExpr(), LLETUExpr = new GRBLinExpr();
				LLTDExpr.addTerm(1.0, visits.get(i).get(j-1).get(k));
				LLTDExpr.addTerm(-1.0, visits.get(i+1).get(j).get(k));
				LLTDExpr.addConstant(1.0);
				
				LLTRExpr.addTerm(1.0, visits.get(i).get(j-1).get(k));
				LLTRExpr.addTerm(-1.0, visits.get(i).get(j+1).get(k));
				LLTRExpr.addConstant(1.0);
						
				LLETUExpr.addTerm(1.0, visits.get(i).get(j-1).get(k));
				LLETUExpr.addTerm(-1.0, visits.get(i-1).get(j).get(k));
				
				gRBModel.addGenConstrIndicator(LLTD, 1, LLTDExpr, GRB.LESS_EQUAL, 0.0, ""); // If DLETR is 1 then the following linear constraint must hold: v_{i+1,j,k} - v_{i,j+1,k} <= 0.
				gRBModel.addGenConstrIndicator(LLTD, 0, LLTDExpr, GRB.GREATER_EQUAL, 1, "");
				
				gRBModel.addGenConstrIndicator(LLTR, 1, LLTRExpr, GRB.LESS_EQUAL, 0.0, ""); // If DLETR is 1 then the following linear constraint must hold: v_{i+1,j,k} - v_{i,j+1,k} <= 0.
				gRBModel.addGenConstrIndicator(LLTR, 0, LLTRExpr, GRB.GREATER_EQUAL, 1, "");
				
				gRBModel.addGenConstrIndicator(LLETU, 1, LLETUExpr, GRB.LESS_EQUAL, 0.0, ""); // If DLETR is 1 then the following linear constraint must hold: v_{i+1,j,k} - v_{i,j+1,k} <= 0.
				gRBModel.addGenConstrIndicator(LLETU, 0, LLETUExpr, GRB.GREATER_EQUAL, 1, "");
		
				// when squeaky is at 2,2, squeaky is constrained to not move to 1,2 but he is told to move to 1,2 by moveDown
				// potential solution is to store whether or not squeaky moves to 1,2 as a result of moving left and then OR it with all other possibilities of squeaky moving to 1,2
				// or say squeakys decision has to be greater than or equal to the AND
				GRBVar temp = gRBModel.addVar(0.0, 1.0, 0.0, GRB.BINARY, "");
				gRBModel.addGenConstrAnd(temp, new GRBVar[] {LLTD, LLTR, LLETU, squeakysDecisions.get(i).get(j).get(k)}, "down less than up");
				//this does not say anything.............................
				gRBModel.addConstr(squeakysDecisions.get(i).get(j-1).get(k+1), GRB.GREATER_EQUAL, temp, "");
			} else {
				GRBLinExpr LHSS = new GRBLinExpr();
				LHSS.addTerm(1.0, squeakysDecisions.get(i).get(j-1).get(k+1));
				GRBLinExpr rightHandSide = new GRBLinExpr();
				rightHandSide.addTerm(1.0, squeakysDecisions.get(i).get(j).get(k));
				gRBModel.addConstr(LHSS, GRB.GREATER_EQUAL, rightHandSide, "");
			}
		} catch (GRBException e) {
			e.printStackTrace();
		}
	}
	
	public void moveUp() {

		try {

			if (i!=size+1 || j != 1) { // dont consider moving up when at finish square.
				
				GRBVar ULTD = gRBModel.addVar(0.0, 1.0, 0.0, GRB.BINARY, ""), ULTR = gRBModel.addVar(0.0, 1.0, 0.0, GRB.BINARY, ""), ULTL = gRBModel.addVar(0.0, 1.0, 0.0, GRB.BINARY, "");
				
				GRBLinExpr ULTDExpr = new GRBLinExpr(), ULTRExpr = new GRBLinExpr(), ULTLExpr = new GRBLinExpr();
				ULTDExpr.addTerm(1.0, visits.get(i-1).get(j).get(k));
				ULTDExpr.addTerm(-1.0, visits.get(i+1).get(j).get(k));
				ULTDExpr.addConstant(1.0);
				
				ULTRExpr.addTerm(1.0, visits.get(i-1).get(j).get(k));
				ULTRExpr.addTerm(-1.0, visits.get(i).get(j+1).get(k));
				ULTRExpr.addConstant(1.0);
				
				ULTLExpr.addTerm(1.0, visits.get(i-1).get(j).get(k));
				ULTLExpr.addTerm(-1.0, visits.get(i).get(j-1).get(k));
				ULTLExpr.addConstant(1.0);
				
				gRBModel.addGenConstrIndicator(ULTD, 1, ULTDExpr, GRB.LESS_EQUAL, 0.0, ""); // If DLETR is 1 then the following linear constraint must hold: v_{i+1,j,k} - v_{i,j+1,k} <= 0.
				gRBModel.addGenConstrIndicator(ULTD, 0, ULTDExpr, GRB.GREATER_EQUAL, 1, "");
				
				gRBModel.addGenConstrIndicator(ULTR, 1, ULTRExpr, GRB.LESS_EQUAL, 0.0, ""); // If DLETR is 1 then the following linear constraint must hold: v_{i+1,j,k} - v_{i,j+1,k} <= 0.
				gRBModel.addGenConstrIndicator(ULTR, 0, ULTRExpr, GRB.GREATER_EQUAL, 1, "");
				
				gRBModel.addGenConstrIndicator(ULTL, 1, ULTLExpr, GRB.LESS_EQUAL, 0.0, ""); // If DLETR is 1 then the following linear constraint must hold: v_{i+1,j,k} - v_{i,j+1,k} <= 0.
				gRBModel.addGenConstrIndicator(ULTL, 0, ULTLExpr, GRB.GREATER_EQUAL, 1, "");
				
				GRBVar temp = gRBModel.addVar(0.0, 1.0, 0.0, GRB.BINARY, "");
				gRBModel.addGenConstrAnd(temp,  new GRBVar[] {ULTD, ULTR, ULTL, squeakysDecisions.get(i).get(j).get(k)}, "down less than up");
				gRBModel.addConstr(squeakysDecisions.get(i-1).get(j).get(k+1), GRB.GREATER_EQUAL, temp, "");
			} else {
				GRBLinExpr LHSS = new GRBLinExpr();
				LHSS.addTerm(1.0, squeakysDecisions.get(i).get(i-1).get(k+1));
				GRBLinExpr rightHandSide = new GRBLinExpr();
				rightHandSide.addTerm(1.0, squeakysDecisions.get(i).get(j).get(k));
				gRBModel.addConstr(LHSS, GRB.GREATER_EQUAL, rightHandSide, "");
			}
		} catch (GRBException e) {
			e.printStackTrace();
		}
	}
	
	// At timestep k we sum up all of squeakyDecision variables at k and make sure they are leq than 1
	public void oneMoveAtATime() {
		
		GRBLinExpr RHS = new GRBLinExpr();
		RHS.addConstant(1.0);
		GRBLinExpr oneMovePerK;
		
		for(int timestep = 0; timestep < upperBound; timestep++) {
			oneMovePerK = new GRBLinExpr();
			for (int row = 0; row < size + 2; row++) {
				for (int column = 0; column < size + 2; column++) {
					oneMovePerK.addTerm(1.0, squeakysDecisions.get(row).get(column).get(timestep));
				}
			}
			try {
				gRBModel.addConstr(oneMovePerK, GRB.LESS_EQUAL, RHS, "");
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
			T.addTerm(-1.0, squeakysDecisions.get(size).get(1).get(timestep));
		}
		T.addConstant(1.0);
		try {
			gRBModel.setObjective(T, GRB.MAXIMIZE);
		} catch (GRBException e) {
			e.printStackTrace();
		}
	}
	
	public void mazeFeasibilityConstraints() {
		GRBLinExpr feasibilityConstraint = new GRBLinExpr();
		for (int timestep = 0; timestep < upperBound; timestep++) {
			feasibilityConstraint.addTerm(1.0, squeakysDecisions.get(size).get(1).get(timestep));
		}
		GRBLinExpr rightHandSide = new GRBLinExpr();
		rightHandSide.addConstant(1.0);
//		System.out.println("squeakys decisions size " + squeakysDecisions.get(1).size() + " and size var is " + size);
		try {
			gRBModel.addConstr(feasibilityConstraint, GRB.GREATER_EQUAL, rightHandSide, "");	
		} catch (GRBException e) {
			e.printStackTrace();
		}
	}
	
	public void addAdditionalConstraint() {

	}

	public void printToFile() {
		// printing values of visits for now

		for (k = 0; k < upperBound; k++) {
			for (i = 0; i < size + 2; i++) {
				for (j = 0; j < size + 2; j++) {
						try {
							 System.out.print(visits.get(i).get(j).get(k).get(GRB.DoubleAttr.X) + " "); // up until timestep 9, squeaky is at all four non edge cells
						} catch (GRBException e) {
							e.printStackTrace();
						}
					
				}
				System.out.println();
			}
			System.out.println();
		}

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
	
	void printGrid() {
		for(int i = 0; i < grid.size(); i++) {
			for (int j = 0; j < grid.get(i).size(); j++) {
				System.out.print(visits.get(i).get(j).get(0));
			}
			System.out.println();
		}
	}
}
