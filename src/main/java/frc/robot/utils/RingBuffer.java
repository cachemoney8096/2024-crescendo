package frc.robot.utils;

import java.util.NoSuchElementException;

public class RingBuffer<T> {

  private Object[] backingArray;
  private int firstIdx = 0; // Index of the first elements
  private int nextIdx = 0; // Index of the slot into which we would insert an element on addLast
  // If nextIdx == (firstIdx - 1) % array_size, then insertion requires resizing the array.
  // If nextIdx == firstIdx, then the buffer is empty.

  public RingBuffer() {
    backingArray = new Object[2];
  }

  private synchronized void doubleBackingArraySize() {
    int newSize = backingArray.length * 2;
    Object[] newArray = new Object[newSize];
    int oldSize;

    if (nextIdx < firstIdx) {
      int numElementsToEndOfArray = backingArray.length - firstIdx;
      System.arraycopy(backingArray, firstIdx, newArray, 0, numElementsToEndOfArray);
      System.arraycopy(backingArray, 0, newArray, numElementsToEndOfArray, nextIdx);
      oldSize = numElementsToEndOfArray + nextIdx;
    } else {
      // This will happen if firstIdx == 0
      System.arraycopy(backingArray, firstIdx, newArray, 0, nextIdx - firstIdx);
      oldSize = nextIdx - firstIdx;
    }

    backingArray = newArray;
    // Update our indices into that array.
    firstIdx = 0;
    nextIdx = oldSize;
  }

  /**
   * Returns the number of elements that the array backing this can hold. This is NOT necessarily
   * the number of elements presently in the buffer. Useful for testing that the implementation
   * works correctly, and for figuring out if an add{First, Last} will cause a re-allocation.
   */
  public int capacity() {
    return backingArray.length - 1;
  }

  /** The number of elements currently held in the buffer. */
  public int size() {
    if (firstIdx <= nextIdx) {
      return nextIdx - firstIdx;
    } else {
      return (backingArray.length - firstIdx + nextIdx);
    }
  }

  public void addLast(T x) {
    if ((nextIdx + 1) % backingArray.length == firstIdx) {
      doubleBackingArraySize();
    }

    backingArray[nextIdx] = x;
    nextIdx += 1;
    nextIdx %= backingArray.length;
  }

  public void addFirst(T x) {
    if ((nextIdx + 1) % backingArray.length == firstIdx) {
      doubleBackingArraySize();
    }

    firstIdx -= 1;
    // Note: in Java -1 % n == -1
    if (firstIdx < 0) {
      firstIdx += backingArray.length;
    }
    backingArray[firstIdx] = x;
  }

  public void removeFirst() {
    if (size() == 0) {
      throw new NoSuchElementException();
    } else {
      firstIdx += 1;
      firstIdx %= backingArray.length;
    }
  }

  public void removeLast() {
    if (size() == 0) {
      throw new NoSuchElementException();
    } else {
      nextIdx -= 1;
      if (nextIdx < 0) {
        nextIdx += backingArray.length;
      }
    }
  }

  public T getFromFirst(int i) {
    if (i < 0 || i >= size()) {
      throw new NoSuchElementException();
    } else {
      @SuppressWarnings("unchecked")
      T returnValue = (T) backingArray[(firstIdx + i) % backingArray.length];
      return returnValue;
    }
  }

  public T getFromLast(int i) {
    if (i < 0 || i >= size()) {
      throw new NoSuchElementException();
    } else {
      int idx = nextIdx - 1 - i;
      if (idx < 0) {
        idx += backingArray.length;
      }
      @SuppressWarnings("unchecked")
      T returnValue = (T) backingArray[idx];
      return returnValue;
    }
  }
}
