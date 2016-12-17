package MachineLearningFramework;

/**
 * NodeGeneratorTask represents the data needed characterize an generated node.
 * 
 * @author feldh
 *
 */
public class NodeGeneratorTask {
	public int methodId;
	public int taskId;
	public Object[] functionArgs;

	public NodeGeneratorTask(int methodId, int taskId, int parameterCount, Object[] functionArgs) {
		this.methodId = methodId;
		this.taskId = taskId;
		this.functionArgs = new Object[parameterCount];
		for (int x = 0; x < parameterCount; x++) {
			this.functionArgs[x] = functionArgs[x];
		}
	}

	public String makeJava() {

		return null;
	}

	public String makePython() {
		return null;
	}
}
