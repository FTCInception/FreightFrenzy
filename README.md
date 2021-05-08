# UltimateGoal
The code for the 2020-2021 season


Instructions to create repo with separate TeamCode repo:

1) open git bash shell (download from https://git-scm.com/downloads) 
  
2) cd to any directory of your choice to place your code.  For example, I ran the following commands.  These commands are optional:

        $ cd /c/awork/ftc/

        $ mkdir UltimateGoal

        $ cd UltimateGoal



 
3) $ git clone https://github.com/FIRST-Tech-Challenge/FtcRobotController
4) $ cd FtcRobotController/TeamCode/src/main/java/
5) $ mkdir Inception
6) $ cd Inception
7) $ git clone https://github.com/FTCInception/UltimateGoal
8) $ cd ../../../../..
9) $ git filter-branch --prune-empty --subdirectory-filter TeamCode/src/main/java/Inception master


After the above is run, your directory is setup and can be imported into Android Studio.  Import the top-level "FtcRobotController" directory. Once in Andriod Studio, you will be able to commit changes to either the FTC FtcRobotController repo or to the Inception UltimateGoal (this) repo.


######## The following instructions are obsolete with V6.2 RobotController #######
########   Refer to this: https://www.learnroadrunner.com/installing.html  #######
######## Skip step 7 (Roadrunner teamcode is already committed in our repo #######
To be able to build with RoadRunner, you will need to add the following to your gradle files:

    TeamCode/build.release.gradle:

        implementation 'org.apache.commons:commons-math3:3.6.1'

        implementation 'com.acmerobotics.roadrunner:core:0.5.3'

        implementation 'com.acmerobotics.dashboard:dashboard:0.4.0'

    FtcRobotController/build.release.gradle:

        implementation 'com.acmerobotics.dashboard:dashboard:0.4.0'

    build.common.gradle (Change VERSION_1_7 to VERSION_1_8:

         sourceCompatibility JavaVersion.VERSION_1_8

         targetCompatibility JavaVersion.VERSION_1_8

## References:

https://help.github.com/en/articles/splitting-a-subfolder-out-into-a-new-repository

https://www.firstinspires.org/sites/default/files/uploads/resource_library/ftc/android-studio-tutorial.pdf

https://www.learnroadrunner.com/installing.html

https://acme-robotics.gitbook.io/road-runner/#installation

