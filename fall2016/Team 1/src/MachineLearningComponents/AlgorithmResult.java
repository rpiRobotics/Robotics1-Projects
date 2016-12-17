package MachineLearningComponents;

import java.util.ArrayList;

/**
 * Defines the result/metrics of the machine learning algorithm. These results are "graded" and then the
 * worst performers die. 
 * @author Ryan
 *
 */
public class AlgorithmResult {
	// TODO what units? Should measure using System.nanoTime()?
	private final int runtime;
	private final int size;
	private ArrayList<ArrayList<Integer>> resultList = null;
	
	public AlgorithmResult(ArrayList<ArrayList<Integer>> resultList, int runtime, int size){
		this.resultList = resultList;
		this.runtime = runtime;
		this.size = size;
	}

	public int getRuntime() {
		return runtime;
	}

	public int getSize() {
		return size;
	}

	public ArrayList<ArrayList<Integer>> getResultList() {
		return resultList;
	}
}
