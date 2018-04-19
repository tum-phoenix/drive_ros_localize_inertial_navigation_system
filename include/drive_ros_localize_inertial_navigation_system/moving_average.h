#ifndef MOVING_AVERAGE_H
#define MOVING_AVERAGE_H

/*
 * Original code from: https://github.com/alphaville/MovingAverage
 * Author: Pantelis Sopasakis
 *
 */

class MovingAverage {

private:

  const static unsigned short default_filter_length = 5;  // Default Length
  unsigned short filter_length;                           // Length of the filter
  double * data;                                          // Vector with raw data
  double sum, average;
  unsigned short index;
  bool filter_complete;
  void init()
  {
    filter_complete = false;
    index = -1;
    sum = 0;
    average = 0;
    data = new double[filter_length];
    clear();
  }

public:
  /**
   * Creates a new instance of MovingAverage with
   * given filter length.
   */
  MovingAverage(unsigned short filterLength)
  {
    filter_length = filterLength;
    init();
  }

  /**
   * Creates a new instance of MovingAverage with
   * the default filter length value 5.
   */
  MovingAverage()
  {
    filter_length = default_filter_length;
    init();
  }

  /**
   * Releases the memory objects associated with the
   * current MovingAverage instance.
   */
  ~MovingAverage()
  {
    delete[] data;
  }

  /**
   * Adds a new element in the Moving Average vector.
   * Updates the current average.
   */
  void add(double x)
  {
    index = (index + 1) % filter_length;
    sum -= data[index];
    data[index] = x;
    sum += x;
    if (!filter_complete && index == filter_length - 1) {
      filter_complete = true;
    }
    if (filter_complete) {
      average = sum / filter_length;
    } else {
      average = sum / (index+1);
    }
  }

  /**
   * Returns the Filter's Length.
   */
  unsigned short getFilterLength()
  {
    return filter_length;
  }

  /**
   * Sets the Filter's Length.
   */
  void setFilterLength(unsigned short filterLength)
  {
      filter_length = filterLength;
      init();
  }

  /**
   * Returns the current average as update after the invocation
   * of MovingAverage::add(double).
   */
  double getCurrentAverage()
  {
    return average;
  }

  /**
   * First adds a new element and then returns the current average.
   */
  double addAndGetCrrtAvg(double x)
  {
    add(x);
    return average;
  }

  /**
   * Clears the vector of data by setting it to zero.
   */
  void clear()
  {
    for (unsigned short i = 0; i < filter_length; i++) {
      data[i] = 0;
    }
  }

  /**
   * Returns the raw data that are currently stored
   * in an internal vector.
   */
  double* getData()
  {
    return data;
  }
};


#endif // MOVING_AVERAGE_H
