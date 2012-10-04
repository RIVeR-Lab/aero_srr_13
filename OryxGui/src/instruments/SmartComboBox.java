package instruments;

import java.util.Vector;

import javax.swing.ComboBoxModel;
import javax.swing.JComboBox;

/**
 * An extension to the JCombobox that allows for easy switching to the prior/next item
 * in the combobox
 * @author joe
 *
 */
public class SmartComboBox extends JComboBox {

	public SmartComboBox() {
		super();
	}

	public SmartComboBox(ComboBoxModel aModel) {
		super(aModel);
	}

	public SmartComboBox(Object[] items) {
		super(items);
	}

	public SmartComboBox(Vector<?> items) {
		super(items);
	}
	
	/**
	 * Moves to the next item in the combobox. If the item is the last, it will move to
	 * the first item.
	 */
	public void nextSelection(){
		if(getSelectedIndex()==getItemCount()-1){
			setSelectedIndex(0);
		}
		else setSelectedIndex(getSelectedIndex()+1);
	}
	
	/**
	 * Moves to the previous item in the combobox. If it is the first item, it moves to the last one.
	 */
	public void previousSelection(){
		if(getSelectedIndex()==0){
			setSelectedIndex(getItemCount()-1);
		}
		else setSelectedIndex(getSelectedIndex()-1);
	}

}
