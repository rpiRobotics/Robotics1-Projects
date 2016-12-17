package MachineLearningFramework;
import java.lang.reflect.Method;

public class AbstractMethod {
	public int parameterCount;
	public Class<?>[] parameterTypes;
	public Class<?> returnType;
	public Method method;
	public Object owner;
	
	public AbstractMethod(Method method, Object owner){
		this.owner = owner;
		this.method = method;
		this.returnType = method.getReturnType();
		this.parameterCount = method.getParameterCount();
		this.parameterTypes = method.getParameterTypes();
	}
}
