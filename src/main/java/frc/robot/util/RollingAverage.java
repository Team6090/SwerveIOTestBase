package frc.robot.util;

/**
 * A simple class that stores a rolling buffer of data and allows
 * a moving average to be calculated.
 *
 * @author Jordan Bancino
 */
public class RollingAverage {
	private double[] data;
	private int cursor;
	private int windowSize;
	
	/**
	 * Create a new RollingAverage.
	 *
	 * @param windowSize The size of the data buffer to create.
	 */
	public RollingAverage(int windowSize) {
		this.windowSize = windowSize;
		reset();
	}
	
	/**
	 * Erase the buffer and set the cursor back to zero.
	 */
	public void reset() {
		this.data = new double[windowSize];
		this.cursor = 0;
	}
	
	/**
	 * Add a data point to the end of the buffer. If the buffer is
	 * full, the first point in the buffer will be dropped and all
	 * the elements will be shifted over to make room at the end.
	 */
	public void add(double point) {
		/* Check to see if the buffer is full. */
		if (cursor == windowSize - 1) {
			/* Shift all the data over one, overwriting the 0th index. */
			for (int i = 1; i < data.length; i++) {
				data[i - 1] = data[i];
			}
		}
		/* Add the point to the cursor index. */
		data[cursor] = point;
		/* Iterate the cursor if the buffer is not full. */
		if (cursor < windowSize - 1) {
			cursor++;
		}
	}
	
	/**
	 * Get the average of all the points in the buffer.
	 *
	 * @return An average of all the points in the buffer.
	 */
	public double get() {
		double sum = 0;
		for (int i = 0; i < cursor; i++) {
			sum += data[i];
		}
		return sum / cursor;
	}

	/** 
	 * Get the size of the cursor, how many spaces are populated with values.
	 * 
	 * @return Cursor size.
	 */
	public int getCursor() {
		return cursor;
	}
}