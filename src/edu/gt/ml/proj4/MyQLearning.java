package edu.gt.ml.proj4;

import burlap.behavior.policy.GreedyQPolicy;
import burlap.behavior.policy.Policy;
import burlap.behavior.policy.PolicyUtils;
import burlap.behavior.singleagent.Episode;
import burlap.behavior.singleagent.auxiliary.EpisodeSequenceVisualizer;
import burlap.behavior.singleagent.auxiliary.StateReachability;
import burlap.behavior.singleagent.auxiliary.performance.LearningAlgorithmExperimenter;
import burlap.behavior.singleagent.auxiliary.performance.PerformanceMetric;
import burlap.behavior.singleagent.auxiliary.performance.TrialMode;
import burlap.behavior.singleagent.auxiliary.valuefunctionvis.ValueFunctionVisualizerGUI;
import burlap.behavior.singleagent.auxiliary.valuefunctionvis.common.ArrowActionGlyph;
import burlap.behavior.singleagent.auxiliary.valuefunctionvis.common.LandmarkColorBlendInterpolation;
import burlap.behavior.singleagent.auxiliary.valuefunctionvis.common.PolicyGlyphPainter2D;
import burlap.behavior.singleagent.auxiliary.valuefunctionvis.common.StateValuePainter2D;
import burlap.behavior.singleagent.learning.LearningAgent;
import burlap.behavior.singleagent.learning.LearningAgentFactory;
import burlap.behavior.singleagent.learning.tdmethods.QLearning;
import burlap.behavior.singleagent.learning.tdmethods.SarsaLam;
//import burlap.behavior.singleagent.learning.
import burlap.behavior.singleagent.planning.Planner;
import burlap.behavior.singleagent.planning.deterministic.DeterministicPlanner;
import burlap.behavior.singleagent.planning.deterministic.informed.Heuristic;
import burlap.behavior.singleagent.planning.deterministic.informed.astar.AStar;
import burlap.behavior.singleagent.planning.deterministic.uninformed.bfs.BFS;
import burlap.behavior.singleagent.planning.deterministic.uninformed.dfs.DFS;
import burlap.behavior.singleagent.planning.stochastic.valueiteration.ValueIteration;
import burlap.behavior.singleagent.planning.stochastic.policyiteration.PolicyIteration;
import burlap.behavior.valuefunction.QProvider;
import burlap.behavior.valuefunction.ValueFunction;
import burlap.domain.singleagent.gridworld.GridWorldDomain;
import burlap.domain.singleagent.gridworld.GridWorldTerminalFunction;
import burlap.domain.singleagent.gridworld.GridWorldRewardFunction;
import burlap.domain.singleagent.gridworld.GridWorldVisualizer;
import burlap.domain.singleagent.gridworld.state.GridAgent;
import burlap.domain.singleagent.gridworld.state.GridLocation;
import burlap.domain.singleagent.gridworld.state.GridWorldState;
import burlap.mdp.auxiliary.stateconditiontest.StateConditionTest;
import burlap.mdp.auxiliary.stateconditiontest.TFGoalCondition;
import burlap.mdp.core.TerminalFunction;
import burlap.mdp.core.state.State;
import burlap.mdp.core.state.vardomain.VariableDomain;
import burlap.mdp.singleagent.common.GoalBasedRF;
import burlap.mdp.singleagent.environment.SimulatedEnvironment;
import burlap.mdp.singleagent.model.FactoredModel;
import burlap.mdp.singleagent.oo.OOSADomain;
import burlap.statehashing.HashableStateFactory;
import burlap.statehashing.simple.SimpleHashableStateFactory;
import burlap.visualizer.Visualizer;
import burlap.mdp.singleagent.common.VisualActionObserver;
import java.util.Random;

import java.awt.*;
import java.util.List;

public class MyQLearning {
	
	static int gridWidth[] ={10, 20, 30, 40, 50, 60, 70, 80, 90, 100}; 
	static int gridlength[] ={10, 20, 30, 40, 50, 60, 70, 80, 90, 100};

	GridWorldDomain gwdg[] = new GridWorldDomain[gridWidth.length];
	OOSADomain domain[] = new OOSADomain[gridWidth.length];
	TerminalFunction tf;
	StateConditionTest goalCondition[] = new StateConditionTest[gridWidth.length];
	GridWorldState initialState;
	HashableStateFactory hashingFactory;
	SimulatedEnvironment env[] = new SimulatedEnvironment[gridWidth.length];

	static int groupSize = gridWidth.length;
	static double trainingTimes[] = new double[groupSize];
	static int totalPolicyIt[] = new int[groupSize];
	static int totalValueIt[] = new int[groupSize];
	static LearningAgentFactory qLearningFactory_1; 
	static LearningAgentFactory qLearningFactory_2; 
	static LearningAgentFactory qLearningFactory_3; 
	
	void initMap(int randomMap[][]) {
		Random rand = new Random();
		for (int k = 0; k < 10; k++) {
		int i = rand.nextInt(gridWidth[k]);
		int j = rand.nextInt(gridlength[k]);
		randomMap[i][j] = 1;
		}
	}
	
	public MyQLearning(){
		//initMap();
		
		for (int k = 0; k < gridWidth.length; k++) {
		gwdg[k] = new GridWorldDomain(gridWidth[k], gridlength[k]);
		//gwdg.setMapToFourRooms();
		//gwdg.setMap(myMap);
		int randomMap[][] = new int [gridWidth[k]][gridlength[k]];

		//gwdg.setMap(randomMap);
		int locX = gridWidth[k] - 1;
		int locY = gridlength[k] - 1;

		tf = new GridWorldTerminalFunction(locX, locY);
		gwdg[k].setTf(tf);
		goalCondition[k] = new TFGoalCondition(tf);
		
		
	//	GridWorldRewardFunction rf = new GridWorldRewardFunction(locX, locY);
		//rf.setReward(locX - 1,  locY - 1, 100);
		//rf.setReward(2,  2, -100);
		//gwdg.setRf(rf);
		
//		goalCondition = new TFGoalCondition(rf);
		
		domain[k] = gwdg[k].generateDomain();
		

		initialState = new GridWorldState(new GridAgent(0, 0), new GridLocation(locX, locY, "loc0"));
				//new GridLocation(15, 15, "loc1"));
		hashingFactory = new SimpleHashableStateFactory();

		env[k] = new SimulatedEnvironment(domain[k], initialState);
		


		//VisualActionObserver observer = new VisualActionObserver(domain, GridWorldVisualizer.getVisualizer(gwdg.getMap()));
		//observer.initGUI();
		//env.addObservers(observer);
		}
	}


	public void visualize(String outputpath, int k){
		Visualizer v = GridWorldVisualizer.getVisualizer(gwdg[k].getMap());
		new EpisodeSequenceVisualizer(v, domain[k], outputpath);
	}

	public void valueIterationExample(String outputPath, int k){

		ValueIteration planner = new ValueIteration(domain[k], 0.99, hashingFactory, 0.001, 1000);
		long time1= System.currentTimeMillis();
		Policy p = planner.planFromState(initialState);
		long time2= System.currentTimeMillis();
		long trainingTime = time2 - time1;
		//trainingTime /= Math.pow(10,9);
		trainingTimes[k] = trainingTime/1000.0;
		System.out.println("VI Time: " + trainingTimes[k]);

		PolicyUtils.rollout(p, initialState, domain[k].getModel()).write(outputPath + "vi");
		
		//PolicyUtils.
		//simpleValueFunctionVis((ValueFunction)planner, p, k);
		//manualValueFunctionVis((ValueFunction)planner, p);

	}

	public void policyIterationExample(String outputPath, int k) {
		PolicyIteration planner = new PolicyIteration(domain[k], 0.99,  hashingFactory, 0.001, 10000, 1000);
		long time1= System.currentTimeMillis();
		Policy p = planner.planFromState(initialState);
		long time2= System.currentTimeMillis();
		long trainingTime = time2 - time1;
		trainingTimes[k] = trainingTime/1000.0;
		//trainingTime /= Math.pow(10,9);
		System.out.println("PI Time: " + trainingTimes[k]);
		PolicyUtils.rollout(p, initialState, domain[k].getModel()).write(outputPath + "pi");

		//simpleValueFunctionVis((ValueFunction)planner, p, k);
		totalPolicyIt[k] = planner.getTotalPolicyIterations();
		totalValueIt[k] = planner.getTotalValueIterations();
	}

	public void qLearningExample(String outputPath, int k){

		QLearning agent = new QLearning(domain[k], 0.99, hashingFactory, 0., 1.);
		//Policy greedyPolicy = new burlap.behavior.policy.GreedyQPolicy((QProvider) agent);
		//Policy randomPolicy = new burlap.behavior.policy.RandomPolicy(domain[k]);
		//Policy greedyDQPolicy = new burlap.behavior.policy.GreedyDeterministicQPolicy((QProvider) agent);
		burlap.behavior.policy.EpsilonGreedy epsilonPolicy = new burlap.behavior.policy.EpsilonGreedy((QProvider) agent, 0.5);
		
		//agent.setLearningPolicy(greedyPolicy);
		//agent.setLearningPolicy(greedyDQPolicy);
		//agent.setLearningPolicy(randomPolicy);
		agent.setLearningPolicy(epsilonPolicy);
		
		long time1= System.currentTimeMillis();

		//run learning for 50 episodes
		for(int i = 0; i < 50; i++){
			Episode e = agent.runLearningEpisode(env[k]);
			//agent.set

			e.write(outputPath + "ql_" + k + "_" + i);
			//System.out.println(i + ": " + e.maxTimeStep());

			//reset environment for next learning episode
			env[k].resetEnvironment();
		}
		long time2= System.currentTimeMillis();
		long trainingTime = time2 - time1;
		trainingTimes[k] = trainingTime/1000.0;
		System.out.println("QLearning Time: " + trainingTimes[k]);

		
		//agent.
		//simpleValueFunctionVis((ValueFunction)agent, new GreedyQPolicy((QProvider) agent), k);

	}


	public void simpleValueFunctionVis(ValueFunction valueFunction, Policy p, int k){

		List<State> allStates = StateReachability.getReachableStates(initialState, domain[k], hashingFactory);
		ValueFunctionVisualizerGUI gui = GridWorldDomain.getGridWorldValueFunctionVisualization(allStates, gridWidth[k], gridlength[k], valueFunction, p);
		gui.initGUI();

	}

	public void manualValueFunctionVis(ValueFunction valueFunction, Policy p, int k){

		List<State> allStates = StateReachability.getReachableStates(initialState, domain[k], hashingFactory);

		//define color function
		LandmarkColorBlendInterpolation rb = new LandmarkColorBlendInterpolation();
		rb.addNextLandMark(0., Color.RED);
		rb.addNextLandMark(1., Color.BLUE);

		//define a 2D painter of state values, specifying which attributes correspond to the x and y coordinates of the canvas
		StateValuePainter2D svp = new StateValuePainter2D(rb);
		svp.setXYKeys("agent:x", "agent:y", new VariableDomain(0, gridlength[k]), new VariableDomain(0, gridlength[k]), 1, 1);

		//create our ValueFunctionVisualizer that paints for all states
		//using the ValueFunction source and the state value painter we defined
		ValueFunctionVisualizerGUI gui = new ValueFunctionVisualizerGUI(allStates, svp, valueFunction);

		//define a policy painter that uses arrow glyphs for each of the grid world actions
		PolicyGlyphPainter2D spp = new PolicyGlyphPainter2D();
		spp.setXYKeys("agent:x", "agent:y", new VariableDomain(0, gridlength[k]), new VariableDomain(0, gridlength[k]), 1, 1);

		spp.setActionNameGlyphPainter(GridWorldDomain.ACTION_NORTH, new ArrowActionGlyph(0));
		spp.setActionNameGlyphPainter(GridWorldDomain.ACTION_SOUTH, new ArrowActionGlyph(1));
		spp.setActionNameGlyphPainter(GridWorldDomain.ACTION_EAST, new ArrowActionGlyph(2));
		spp.setActionNameGlyphPainter(GridWorldDomain.ACTION_WEST, new ArrowActionGlyph(3));
		spp.setRenderStyle(PolicyGlyphPainter2D.PolicyGlyphRenderStyle.DISTSCALED);


		//add our policy renderer to it
		gui.setSpp(spp);
		gui.setPolicy(p);

		//set the background color for places where states are not rendered to grey
		gui.setBgColor(Color.GRAY);

		//start it
		gui.initGUI();



	}


	public void experimentAndPlotter(int k){

		//different reward function for more structured performance plots
		((FactoredModel)domain[k].getModel()).setRf(new GoalBasedRF(this.goalCondition[k], 100.0, -0.1));

		/**
		 * Create factories for Q-learning agent and SARSA agent to compare
		 */
		qLearningFactory_1 = new LearningAgentFactory() {

			public String getAgentName() {
				return "Q-Learning_1_" + k;
			}


			public LearningAgent generateAgent() {
				QLearning agent = new QLearning(domain[k], 0.99, hashingFactory, 0.3, 0.1);
				Policy greedyDQPolicy = new burlap.behavior.policy.GreedyDeterministicQPolicy((QProvider) agent);
				agent.setLearningPolicy(greedyDQPolicy);
				return agent;
			}
		};

		qLearningFactory_2 = new LearningAgentFactory() {

			public String getAgentName() {
				return "Q-Learning_2_" + k;
			}


			public LearningAgent generateAgent() {
				QLearning agent = new QLearning(domain[k], 0.99, hashingFactory, 0.3, 0.1);
				Policy greedyPolicy = new burlap.behavior.policy.GreedyQPolicy((QProvider) agent);				
				agent.setLearningPolicy(greedyPolicy);
				return agent;
			}
		};
	
		qLearningFactory_3 = new LearningAgentFactory() {

			public String getAgentName() {
				return "Q-Learning_3_" + k;
			}


			public LearningAgent generateAgent() {
				QLearning agent = new QLearning(domain[k], 0.99, hashingFactory, 0.3, 0.1);
				return agent;
			}
		};
	}
	public void experimentAndPlotter2(int k){
	LearningAlgorithmExperimenter exp = new LearningAlgorithmExperimenter(env[k], 10, 100, 
			qLearningFactory_1, qLearningFactory_2, qLearningFactory_3);//, sarsaLearningFactory);
	exp.setUpPlottingConfiguration(500, 250, 2, 1000,
			TrialMode.MOST_RECENT_AND_AVERAGE,
			PerformanceMetric.CUMULATIVE_STEPS_PER_EPISODE,
			PerformanceMetric.AVERAGE_EPISODE_REWARD);

	exp.startExperiment();
	exp.writeStepAndEpisodeDataToCSV("expData_"+ k);
	}

	public static void main(String[] args) {
		MyQLearning example = new MyQLearning();

		for (int k = 0; k < groupSize; k++) {
			String outputPath = "output_" + k + "/";

		//example.valueIterationExample(outputPath, k);
		//example.policyIterationExample(outputPath, k);
		 example.qLearningExample(outputPath, k);

		//example.experimentAndPlotter(k);
		//example.experimentAndPlotter2(k);
		//example.visualize(outputPath, k);
		}
		System.out.println("Grid Size\tPlaning_T\tPolicy#\tValue#");
		
		for (int k = 0; k < groupSize; k++) {
			
			System.out.println(gridWidth[k] + "\t" + trainingTimes[k] +"\t" + totalPolicyIt[k] + "\t" + totalValueIt[k] );
		}

	}

}
