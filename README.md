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

####### FTCDashbaord should 'just work'.                                   #######


####### I did not update my Gradle this time.

####### If your gradle fails see note below

####### You may need to download API 29: Tools->SDK Manager install API Level-29 in SDK Platforms

## References:

https://help.github.com/en/articles/splitting-a-subfolder-out-into-a-new-repository

https://www.firstinspires.org/sites/default/files/uploads/resource_library/ftc/android-studio-tutorial.pdf

https://www.learnroadrunner.com/installing.html

https://acme-robotics.gitbook.io/road-runner/#installation

https://acmerobotics.github.io/ftc-dashboard/gettingstarted

