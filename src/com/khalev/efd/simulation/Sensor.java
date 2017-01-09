package com.khalev.efd.simulation;

/**
 * Interface that any collision sensor must implement.
 */
//TODO: consider storing Object
public class Sensor<T> {

    private T input;

    void receiveInput(Object input) {
        //TODO: remove this check & Exception
        if (this.input == null || (input.getClass().isAssignableFrom(this.input.getClass()))) {
            @SuppressWarnings("unchecked")
            T inp = (T) input;
            this.input = inp;
        } else {
            throw new RuntimeException();
        }
    }

    boolean inputReceived(Object input) {
        if (this.input == null && input == null) {
            return true;
        }
        if (this.input == null || input == null || !(input.getClass().isAssignableFrom(this.input.getClass()))) {
            return false;
        }
        return this.input.equals(input);
    }

    T getInput() {
        return this.input;
    }
}
