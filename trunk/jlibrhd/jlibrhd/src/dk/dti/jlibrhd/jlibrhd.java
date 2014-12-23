/** \file jlibrhd.java
 *  \brief Robot Hardware Daemon Library
 *  Socket interface to the Robot Hardware Daemon HAL in Java
 *
 *  \author Anders Billesø Beck
 *  $Rev: 798 $
 *  $Date: 2009-12-04 14:21:56 +0100 (Fri, 04 Dec 2009) $
 */
package dk.dti.jlibrhd;

import java.util.Vector;
import java.io.*;
import java.net.*;

/**
 *
 * @author andersbeck
 */
public class jlibrhd {

    public static final String DEFAULT_HOST = "localhost";
    public static final int    DEFAULT_PORT = 24902;

    private final boolean DEBUG = false;

    private Vector<symTableElement> rTable = new Vector<symTableElement>();
    private Vector<symTableElement> wTable = new Vector<symTableElement>();

    Socket rhdSocket;
    DataInputStream inStream;
    DataOutputStream outStream;

    char connected;

/*
  char              rhdDisconnect(void);
  char              rhdConnect(char, char *, int);
  char              rhdSync(void);
  symTableElement*  getSymbolTable(char);
  int               getSymbolTableSize(char);
  int               readValue(int, int);
  int               readValueNamed(char *, int);
  int *             readArray(int);
  int *             readArrayNamed(char *);
  int               writeValue(int, int, int);
  int               writeValueNamed(char *, int, int);
  int               writeArray(int, int, int*);
  int               writeArrayNamed(char *,int, int*);
  int               isRead(int);
  int               isReadNamed(char *);
 */

    public char rhdConnect(char dir, String host, int port) {

        //Connect to RHD socket
        try {
            rhdSocket = new Socket(host,port);

            rhdSocket.setTcpNoDelay(true);
            inStream  = new DataInputStream(rhdSocket.getInputStream());
            outStream = new DataOutputStream(rhdSocket.getOutputStream());

            //Send a request acceess to RHD
            byte[] req = new byte[2];
            req[0] = 'a';
            req[1] = (byte)dir;

            outStream.write(req, 0, 2); //Write the request to RHD

            //Run the package parser
            if (!this.rhdPackageParser()) {
                return 0;
            }

            //Clear updated flags

        } catch (UnknownHostException ex) {
            return 0;
        } catch (IOException ex) {
            return 0;
        }

        if (DEBUG) System.out.println("Conneced to RHD with " + connected + " access");

        return connected;

    }

    public boolean rhdDisconnect() {
        try {
            connected = 0;
            if (rhdSocket.isConnected()) rhdSocket.close();
            rTable.clear();
            wTable.clear();
        }
        catch (IOException ex) {
            return false;
        }

        return true;
    }

    public boolean rhdSync() {

        byte[] readyPkgType = {(byte)'r',(byte)'r'};
        byte[] endPkgType = {(byte)'e',(byte)'e'};

        //Set the direction in the ready package
        readyPkgType[1] = (byte)connected;

        try {
            outStream.write(readyPkgType, 0, 2);

            //TODO: Implement TX of write package

            outStream.write(endPkgType, 0, 2);

            //Reset the updated flags
            for (int i = 0; i < rTable.size(); i++) rTable.elementAt(i).setUpdated(false);
            //Only clear w-status if read-client
            if (connected == 'r') {
                for (int i = 0; i < wTable.size(); i++) wTable.elementAt(i).setUpdated(false);
            }

            if (!rhdPackageParser()) {
                rhdDisconnect();
                return false;
            }
        }
        catch (IOException ex) {
            System.out.println("RHD Sync error!");
            rhdDisconnect();
        }

        return true;
    }

    private boolean rhdPackageParser() {

        boolean end = false;
        byte[] pkgType = new byte[2];

        while (!end) {
            try {
                //Read pkg-type from socket
                inStream.readFully(pkgType,0,2);

                if (DEBUG) System.out.println("RHD Package: " + (char)pkgType[0] + " " + (char)pkgType[1]);

                //Parse package type
                switch (pkgType[0]) {
                    case 'a': //Parse access permission package
                        if (pkgType[1] == 'r' || pkgType[1] == 'w'){
                            connected = (char)pkgType[1];
                        } else {
                            return false;
                        }
                        break;

                    case 't': //Parse symbol table package
                        if (!recieveSymboltable((char)pkgType[1])) {
                            return false;
                        }
                        break;

                    case 'd': //Parse data package
                        if (!recieveDatapackage((char)pkgType[1])) {
                            return false;
                        }
                        break;

                    case 'e': //Parse end of period package
                    default:
                        if (DEBUG) System.out.println("End of package session...");
                        end = true;
                        break;

                }
                
                
            }
            catch (IOException ex) {
                System.out.println("IO Error when reading socket");
                rhdDisconnect();
                return false;
            }
            
            
        }
        return true;
    }

    private boolean recieveSymboltable (char dir) {

        Vector<symTableElement> tmpTable;
        symTableElement tmpElement;
        int nTableElements, dLength;
        byte[] name = new byte[symTableElement.RHD_NAMELEN];

        if (DEBUG) System.out.println("Parsing symbol table for direction: " + dir);
        
        try {
            //Recieve the size of symbol table
            nTableElements = inStream.readInt();
            if (DEBUG) System.out.println("   Reciving " + nTableElements + " Elements");

            for (int nRemaining = nTableElements; nRemaining > 0; nRemaining--) {
                tmpElement = new symTableElement();

                //Read the length
                tmpElement.setLength(inStream.readInt());
                //Read the name (Using the fixed length, defined in symTableElement and rhd.h)
                inStream.readFully(name, 0,tmpElement.RHD_NAMELEN);
                tmpElement.setName(new String(name).trim());
                tmpElement.setUpdated(false);

                if (DEBUG) System.out.println("   Added: " + tmpElement.getName() + "["+ tmpElement.getLength() + "]");

                //Add the table element to table vectors.
                if (dir == 'r') {
                    rTable.add(tmpElement);
                } else if (dir == 'w') {
                    wTable.add(tmpElement);
                }
            }
        }
        catch (IOException ex) {
                System.out.println("IO Error when symboltable");
                rhdDisconnect();
                return false;
        }
        
        return true;
    }

    //Recieve a data package
    private boolean recieveDatapackage(char dir) {
        Vector<symTableElement> tmpTable;
        int nDataElements, varId, value;
        double timestamp;

        if (DEBUG) System.out.println("Recieving datapackage for " + dir);

        //Select the read or write table
        if (dir == 'r') {
            tmpTable = rTable;
        } else {
            tmpTable = wTable;
        }

        //Recieve the data package
        try {
            //Recieve the size of symbol table
            nDataElements = inStream.readInt();

            if (DEBUG)System.out.println("   Contains " + nDataElements + " elements");

            for (int nRemaining = nDataElements; nRemaining > 0; nRemaining--) {
                //Read the variable id
                varId = inStream.readInt();

                if (DEBUG) System.out.print("    " + tmpTable.elementAt(varId).getName() + ": [");

                //Read timestamp (and convert to double)
                timestamp =     (double)inStream.readInt();
                timestamp +=    ((double)inStream.readInt()) / 1000000;
                tmpTable.elementAt(varId).setTime(timestamp);

                //Then read data elements
                for (int nData = 0; nData < tmpTable.elementAt(varId).getLength(); nData++){
                    value = inStream.readInt();
                    tmpTable.elementAt(varId).setData(nData, value);
                    if (DEBUG) System.out.print(value + ",");
                }
                if (DEBUG) System.out.print("] ");

                if (DEBUG) System.out.println(" (" + timestamp + ")");
                //Set the variable as updated
                tmpTable.elementAt(varId).setUpdated(true);
            }
        }
        catch (IOException ex) {
                System.out.println("IO Error when symboltable");
                rhdDisconnect();
                return false;
        }

        return true;
    }

    /*
     *   symTableElement*  getSymbolTable(char);
  int               getSymbolTableSize(char);
  int               readValue(int, int);
  int               readValueNamed(char *, int);
  int *             readArray(int);
  int *             readArrayNamed(char *);
  int               writeValue(int, int, int);
  int               writeValueNamed(char *, int, int);
  int               writeArray(int, int, int*);
  int               writeArrayNamed(char *,int, int*);
  int               isRead(int);
  int               isReadNamed(char *);
     */

    public Vector<symTableElement> getSymbolTable(char dir) {

        if (dir == 'w') {
            return wTable;
        } else {
            return rTable;
        }
    }

    public int getSymbolTableSize(char dir) {

        if (dir == 'w') {
            return wTable.size();
        } else {
            return rTable.size();
        }
    }

    public int[] getReadData(String name) {

        for (int i = 0; i < rTable.size(); i++) {
            if (rTable.elementAt(i).getName().equals(name)) {
                return rTable.elementAt(i).getData();
            }
        }
        return null;
    }

    public int[] getWriteData(String name) {
        for (int i = 0; i < wTable.size(); i++) {
            if (wTable.elementAt(i).getName().equals(name)) {
                return wTable.elementAt(i).getData();
            }
        }
        return null;
    }



    //Main method for testing...
    public static void main(String[] args) {
        jlibrhd lrhd =  new jlibrhd();

        lrhd.rhdConnect('r', lrhd.DEFAULT_HOST, lrhd.DEFAULT_PORT);

        for (int i = 0; i < 100; i++) {
            if (!lrhd.rhdSync()) {
                System.out.println("Sync error!");
                break;
            }
            int[] tick = lrhd.getReadData("tick");
            System.out.println("Recieved tick " + tick[0]);
        }

        lrhd.rhdDisconnect();
    }

}
