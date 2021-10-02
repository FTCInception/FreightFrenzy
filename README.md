# FreightFrenzy
The code for the 2020-2021 season


Instructions to create repo with separate TeamCode repo:

1) open git bash shell (download from https://git-scm.com/downloads) 
  
2) cd to any directory of your choice to place your code.  For example, I ran the following commands.  These commands are optional:

        $ cd /c/awork/ftc/

        $ mkdir FreightFrenzy

        $ cd FreightFrenzy



3) $ git clone https://github.com/FIRST-Tech-Challenge/FtcRobotController
4) $ cd FtcRobotController/TeamCode/src/main/java/
5) $ mkdir Inception
6) $ cd Inception
7) $ git clone https://github.com/FTCInception/FreightFrenzy
8) $ cd ../../../../..
9) $ git filter-branch --prune-empty --subdirectory-filter TeamCode/src/main/java/Inception master


After the above is run, your directory is setup and can be imported into Android Studio.  Import the top-level "FtcRobotController" directory. Once in Andriod Studio, you will be able to commit changes to either the FTC FtcRobotController repo or to the Inception UltimateGoal (this) repo.


########   Refer to this: https://www.learnroadrunner.com/installing.html  #######

########   Method 2: https://learnroadrunner.com/installing.html#method-2-installing-rr-on-your-project #######

######## Skip step 7 (Roadrunner teamcode is already committed in our repo #######



####### Follow this for enabling FTCDashbaord:                             #######

####### https://acmerobotics.github.io/ftc-dashboard/gettingstarted        #######

####### Note that I think there are now hooks in the RobotController code  #######

####### for FTCDashboard but the instructions at the link don't use them.  #######

####### I think the old way at the link shodl still work.                  #######

####### I also found that I needed to add an import for FTCDashboard.      #######


I also update Gradle

And I found that Gradle Sync was always failing

This was resolved by opening Tools->SDK Manager and then installing API Level-29 in SDK Platforms

## References:

https://help.github.com/en/articles/splitting-a-subfolder-out-into-a-new-repository

https://www.firstinspires.org/sites/default/files/uploads/resource_library/ftc/android-studio-tutorial.pdf

https://www.learnroadrunner.com/installing.html

https://acme-robotics.gitbook.io/road-runner/#installation



######## The following instructions are obsolete with V6.2 RobotController #######
#NA To be able to build with RoadRunner, you will need to add the following to your gradle files:

#NA 

#NA     TeamCode/build.release.gradle:

#NA 

#NA         implementation 'org.apache.commons:commons-math3:3.6.1'

#NA 

#NA         implementation 'com.acmerobotics.roadrunner:core:0.5.3'

#NA 

#NA         implementation 'com.acmerobotics.dashboard:dashboard:0.4.0'

#NA 

#NA     FtcRobotController/build.release.gradle:

#NA 

#NA         implementation 'com.acmerobotics.dashboard:dashboard:0.4.0'

#NA 

#NA     build.common.gradle (Change VERSION_1_7 to VERSION_1_8:

#NA 

#NA          sourceCompatibility JavaVersion.VERSION_1_8

#NA 

#NA          targetCompatibility JavaVersion.VERSION_1_8

