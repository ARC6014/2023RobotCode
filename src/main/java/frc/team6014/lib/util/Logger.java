package frc.team6014.lib.util;

/*import com.opencsv.CSVWriter;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;

/**
 * A class that logs data to a CSV file.
 * @author Carabelli / 6014
 * @version 1.1
 * @since 1.0
 * @see CSVWriter
 *
 * MIT License
 * Copyright (c) 2022 Carabelli, All Rights Reserved #6014
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 */
public class Logger {
/*     private boolean wasConnected;
    private long startTime;

    //TODO: change pathway relative to your project
    public static final String loggerFilesFolder = "src\\main\\java\\frc\\team6014\\lib\\util";
    private static final String logFront = "Log_";
    private static int teamNumber = 6014;
    private String[] keys = {"Key", "Value", "Time"};
    private static final DateTimeFormatter dtf = DateTimeFormatter.ofPattern("yyyy-MM-dd_HH-mm-ss");
    private CSVWriter csvWriter;

    private static Logger logger;

    public static Logger getLoggerInstance() {
        if (logger == null) {
            logger = new Logger();
        }
        return logger;
    }

    Logger() {
        connectToLogger();
    }

    private void initializeLogging() {

        startTime = System.currentTimeMillis();
        wasConnected = false;

    }

    private String getCSVFileName() {
        return loggerFilesFolder + "\\" + logFront + dtf.format(LocalDateTime.now()) + ".csv";
    }*/

    /*
     * Opens a new log file with the current date and time
     * @see CSVWriter
     */
/*     private void openNewLogFile() throws IOException {
        if (wasConnected == false){
            try {
                String fileName = getCSVFileName();

                System.out.print(fileName);

                File csv = new File("ghgg.csv");
                csv.createNewFile();

                System.out.println("00000000000000000000000000000000000000000");
                FileWriter fileWriter = new FileWriter(csv);

                System.out.println("****************************");
                csvWriter = new CSVWriter(fileWriter);
                csvWriter.writeNext(keys);
            } catch (IOException e) {
                System.err.println("Error opening csvWriter");
                e.printStackTrace();
            }
            startTime = System.currentTimeMillis();
        }
        else {
            throw new IOException("Couldn't connect to logger");
        }

    }



    private void connectToLogger() {
        try {
            System.out.println("Initializing logging (opening new log file)");
            initializeLogging();
            openNewLogFile();
            wasConnected = true;
            System.out.println("Logging initialized");
        }
        catch (IOException e) {
            System.out.println("Failed to initialize logging");
            e.printStackTrace();
        }

    }*/

    /**
     * Closes the log file
     * @throws IOException
     *
     * @see IOException
     *
     */
/*     public void closeLogger() {
        System.out.println("Disconnected, closing log file");
        closeLogFile();
        wasConnected = false;
    }

    private void closeLogFile() {
        try {
            if (csvWriter != null) {
                csvWriter.close();
                System.out.println("CSV file closed");
                csvWriter = null;
            }
        } catch (IOException e) {
            System.err.println("*********Error closing csvWriter************");
            e.printStackTrace();
        }
    }

    private long getElapsedTime() {
        return (System.currentTimeMillis() - startTime)/1000;
    }


    public <T> void writeToLogFile(String key,T value) {
        long elapsedTime = getElapsedTime();
        String[] data = {key, value.toString(), String.valueOf(elapsedTime)};
        csvWriter.writeNext(data);
    }



    private boolean wasConnected() {
        return wasConnected;
    }

    public void addShutdownHook() {
        Runtime.getRuntime().addShutdownHook(new Thread() {
            @Override
            public void run() {
                System.out.println("Application terminating");
                logger.closeLogFile();
            }
        });
    }
*/
}