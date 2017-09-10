package MachineLearningFramework;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;

/**
 * 
 * @author Ryan Feldhausen
 *
 */
public class ReflectionHelper {
	private int foundMethodCount;
	private ArrayList<Method> foundMethods;
	private Object origin;
	
	public ReflectionHelper(Object object, String prefix){
		// Initialize vars
		int foundMethodCount = 0;
		ArrayList<Method> foundMethods = new ArrayList<Method>();
		
		// Grab all methods from the given object
		Method[] allMethods = object.getClass().getMethods();
		// Iterate through the given object and pull out all methods that 
		// A: Start with the given class
		// B: Are prefixed with the given prefix.
		// ex. NodeGenerator.constructRectangle()
		for (Method method : allMethods){
			if (method.getDeclaringClass().equals(object.getClass()) && method.getName().startsWith(prefix)){
				foundMethods.add(method);
				foundMethodCount += 1;
			}
		}
		
		// Methods are sorted by name to ensure that they an integer always 
		// references the same method assuming none are added or removed.
		class MethodCompartor implements Comparator<Method> {
		    @Override
		    public int compare(Method a, Method b) {
		        return a.getName().compareTo(b.getName());
		    }
		}
		
		Collections.sort(foundMethods, new MethodCompartor());
		this.foundMethodCount = foundMethodCount;
		this.foundMethods = foundMethods;
		this.origin = object;
	}
	
	public Object executeMethodX(int x, Object...args){
		Method methodX = foundMethods.get(x);
		try {
			Object returnValue = methodX.invoke(origin, args);
			return returnValue;
		} catch (IllegalAccessException | IllegalArgumentException | InvocationTargetException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
			return -1;
		}
	}
	
	public Object executeMethodX(int x){
		Method methodX = foundMethods.get(x);
		Object[] args = null;
		try {
			Object returnValue = methodX.invoke(origin, args);
			return returnValue;
		} catch (IllegalAccessException | IllegalArgumentException | InvocationTargetException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
			return -1;
		}
	}
	
	public AbstractMethod characterizeMethodX(int x){
		Method methodX = foundMethods.get(x);
		AbstractMethod characterizedMethod = new AbstractMethod(methodX, origin);
		return characterizedMethod;
	}
	
	public int getMethodCount(){
		return foundMethodCount;
	}
	
}
