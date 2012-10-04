import java.lang.reflect.Array;


public class Util {
	public static <T> T[] addNull(T[] array){
		@SuppressWarnings("unchecked")
		T[] newArray = (T[]) Array.newInstance(array.getClass().getComponentType(), array.length+1);
		for(int i = 0; i<array.length; ++i)
			newArray[i+1] = array[i];
		return newArray;
	}
}
