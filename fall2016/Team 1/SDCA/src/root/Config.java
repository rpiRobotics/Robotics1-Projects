package root;

import MachineLearningComponents.AlgorithmResult;
import MachineLearningComponents.HumanResult;

/**
 * This class essentially serves as the "command center" for configuring most everything interesting about
 * how the machine learning algorithm does its job. 
 * @author Ryan
 *
 */
public class Config {
	public static final int GENERATION_SIZE = 1000;
	public static final double GENERATION_KILL_PERCENT = 0.50;
	
	public static final int RANDOM_NUMBER_GEN_MEDIAN = 0;
	public static final int RANDOM_NUMBER_GEN_STANDARD_DEVIATION = 10;
	
	public static double grade(AlgorithmResult algorithmResult, HumanResult humanResult){
		
		return 0;
	}
}
