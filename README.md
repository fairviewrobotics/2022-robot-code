# 2022 Robot Code

Code for FRC 2036 Black Knights' 2022 Rapid React robot.

## Environment
This project is most compatible with Java 11, which is
not the most recent version of Java. 

It is 
recommended to use [jEnv](https://github.com/jenv/jenv)
to switch between Java versions on Mac OS and Linux.
I followed [these](https://chamikakasun.medium.com/how-to-manage-multiple-java-version-in-macos-e5421345f6d0)
instructions to set up jEnv on macOS Big Sur.

## Installing the JVM (Ubuntu/Debian based systems)
-	`sudo apt install -y openjdk-11-jdk` - install the JVM
-	`echo export JAVA_HOME=/usr/lib/jvm/java-1.11.0-openjdk-amd64 >> ~/.bashrc` - Make sure Gradle knows where your Java installation is
-	`source ~/.bashrc` - Update your terminal

## Installing the JVM (Windows 10/11)

Note: You must create an Oracle account in order to download Java.

-	Go to the Java 11 download page [here](https://www.oracle.com/java/technologies/javase/jdk11-archive-downloads.html)
-	Install the file once it has finished downloading and restart your computer
-	Download the file marked as the Windows x64 Installer for the JAVA SE Development Kit 10.11.11

## Building

This project uses gradle as its build system. The GradleRIO plugin provides a wide number of FRC specific gradle commands (see [here](https://github.com/wpilibsuite/GradleRIO) for details).

The major commands are:
-	`./gradlew build` - build the project
-	`./gradlew deploy` - deploy code to the robot (or `./gradlew build deploy` to do both)
-	`./gradlew riolog` - display the rio log output

Deploying and displaying the log require your computer to be connected to the robot (tethered over ethernet or connected to the robot's radio).

Passing the `--offline` flag to gradle will prevent it from trying to update and/or download dependencies, which can be useful at competition.

It is **highly recommended** to use intellij when working on the code. VSCode, though it is the officially supported editor, provides *very* limited code completion. Intellij provides excellent code completion, especially for kotlin.

## FRC Documentation
The documentation for the FRC Control System can be found [here](https://docs.wpilib.org/en/latest/). These are very useful to refer to.

