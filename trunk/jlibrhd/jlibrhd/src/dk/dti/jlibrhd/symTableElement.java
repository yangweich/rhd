/** \file symTableElement.java
 *  \brief RHD Symbol table element container
 *
 *  Socket interface to the Robot Hardware Daemon HAL in Java
 *
 *  \author Anders Billesø Beck
 *  $Rev: 798 $
 *  $Date: 2009-12-04 14:21:56 +0100 (Fri, 04 Dec 2009) $
 */

package dk.dti.jlibrhd;

import java.lang.String;

/**
 *
 * @author andersbeck
 */
public class symTableElement {
    //Length of RHD variable name for transfer.
    public final static int RHD_NAMELEN = 33;

    int[]    data;
    int      length;
    String   name;
    boolean  updated;
    long     time_sec;
    long     time_usec;
    double  timestamp;

    public double getTime() {
        return (double)time_sec + ((double)time_usec)/1000000;
        //return timestamp;
    }

    public int getData(int id) {
            return this.data[id];
    }

    public int[] getData() {
        return this.data;
    }

    public String getName() {
        return this.name;
    }

    public int getLength() {
        return this.length;
    }

    public boolean isUpdated() {
        return updated;
    }

    public void setData(int id, int data) {
        this.data[id] = data;
    }

    public void setLength(int len) {
        this.data = new int[len]; //Allocate the data container
        this.length = len;
    }

    public void setName(String nam) {
        this.name = nam;
    }

    public void setUpdated(boolean upd) {
        this.updated = upd;
    }

    public void setTime(double tim) {
        this.timestamp = tim;
    }


}
