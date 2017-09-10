package MachineLearningComponents;

import java.util.ArrayList;

/**
 * Defines the human result that is compared to the output from the machine learning algorithm. 
 * @author Ryan
 *
 */
public class HumanResult {
	private ArrayList<ArrayList<Integer>> resultList = null;
	
	public HumanResult(ArrayList<ArrayList<Integer>> resultList){
		this.resultList = resultList;
	}

	public ArrayList<ArrayList<Integer>> getResultList() {
		return resultList;
	}
}
