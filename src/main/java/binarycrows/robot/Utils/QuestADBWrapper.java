package binarycrows.robot.Utils;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.util.Arrays;
import java.util.concurrent.TimeUnit;

public class QuestADBWrapper {

    public enum QuestNavADBCommandResult {
        SUCCESS,
        PREVIOUS_COMMAND_NOT_DONE,
        IO_EXCEPTION,
        COMMAND_FAILED
    }

    // TODO: Quest ADB reset stuff!
    // ./adb tcpip 5555
    // ./adb connect 10.105.89.21:5555
    // ./adb shell am start -n gg.QuestNav.QuestNav/com.unity3d.player.UnityPlayerGameActivity
    public static Process currentADBCommand;

    public static final String adbExecPath = "~/adb";
    public static final String questAddress = "10.105.89.200:5555";

    public static boolean isConnected = false;

    public static <T> T[] add2BeginningOfArray(T[] elements, T element1, T element2, T element3)
    {
        T[] newArray = Arrays.copyOf(elements, elements.length + 3);
        newArray[0] = element1;
        newArray[1] = element2;
        newArray[2] = element3;
        System.arraycopy(elements, 0, newArray, 3, elements.length);

        return newArray;
    }


    public static QuestNavADBCommandResult sendQuestNavADBCommand(String command) {
        if (currentADBCommand != null) {
            if (currentADBCommand.isAlive()) return QuestNavADBCommandResult.PREVIOUS_COMMAND_NOT_DONE;
        }
        try {
            currentADBCommand = Runtime.getRuntime().exec(new String[]{"bash", "-c", adbExecPath + " " + command}, new String[] {"HOME=/home/lvuser/"});
            return QuestNavADBCommandResult.SUCCESS;
        } catch (IOException e) {
            e.printStackTrace();
            return QuestNavADBCommandResult.IO_EXCEPTION;
        }
    }

    public static void waitForCurrentADBCommand() {
        try {
            if (!currentADBCommand.waitFor(500, TimeUnit.MILLISECONDS)) {
                System.out.println("Timed out");
                currentADBCommand.destroyForcibly();
            }
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    public static String getCurrentADBCommandOutput() {
        BufferedReader outputStream = new BufferedReader(new InputStreamReader(currentADBCommand.getInputStream()));
        String line = null;
        String lastLine = "[no output]";
        try {
            while ((line = outputStream.readLine()) != null) {lastLine = line;}
            printOutput();
        } catch (IOException e) {
            e.printStackTrace();
        }
        
        return lastLine;
    }

    public static void printOutput() throws IOException {
        BufferedReader stdInput = new BufferedReader(new 
            InputStreamReader(currentADBCommand.getInputStream()));

        BufferedReader stdError = new BufferedReader(new 
            InputStreamReader(currentADBCommand.getErrorStream()));

        // Read the output from the command
        System.out.println("Here is the standard output of the command:\n");
        String s = null;
        while ((s = stdInput.readLine()) != null) {
            System.out.println(s);
        }

        // Read any errors from the attempted command
        System.out.println("Here is the standard error of the command (if any):\n");
        while ((s = stdError.readLine()) != null) {
            System.out.println(s);
        }
    }

    public static boolean updateIsConnected() {
        if (isConnected) return true;
        QuestNavADBCommandResult result = sendQuestNavADBCommand("get-state");
        System.out.println(result.name());
        if (result == QuestNavADBCommandResult.SUCCESS) {
            waitForCurrentADBCommand();
            String output = getCurrentADBCommandOutput();
            System.out.println("hmm:");
            System.out.println(output);
            System.out.println(output.equals("device"));
            QuestADBWrapper.isConnected = output.equals("device");
        }
        return QuestADBWrapper.isConnected;
    }

    public static QuestNavADBCommandResult tryConnect() {
        QuestNavADBCommandResult result = sendQuestNavADBCommand("connect " + questAddress);
        waitForCurrentADBCommand();
        if (result == QuestNavADBCommandResult.SUCCESS && currentADBCommand.exitValue() != 0) return QuestNavADBCommandResult.COMMAND_FAILED; 
        return result;
    }

    public static void lazyTryConnect() {
        sendQuestNavADBCommand("connect " + questAddress);
    }

    public static void tryRestartQuestNav() {
        // For some reason we have to run it twice...
        for (int i = 0; i < 1; i++) {
            waitForCurrentADBCommand();
            sendQuestNavADBCommand("shell am start -n gg.QuestNav.QuestNav/com.unity3d.player.UnityPlayerGameActivity");
        }
    }
    
}
