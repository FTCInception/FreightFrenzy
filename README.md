# Skystone
The code for the 2019-2020 season


Instructions to create repo with separate TeamCode repo:

1) open git bash shell (download from https://git-scm.com/downloads) 
  
2) cd to any directory of your choice to place your code.  For example, I ran the following commands.  These commands are optional:

        $ cd /c/awork/ftc/

        $ mkdir skystone2

        $ cd skystone2



 
3) $ git clone https://github.com/FIRST-Tech-Challenge/SkyStone
4) $ cd SkyStone/TeamCode/src/main/java/
5) $ mkdir Inception
6) $ cd Inception
7) $ git clone https://github.com/FTCInception/Skystone
8) $ cd ../../../../..
9) $ git filter-branch --prune-empty --subdirectory-filter TeamCode/src/main/java/Inception master


After the above is run, your directory is setup and can be imported into Android Studio.  Import the top-level "SkyStone" directory. 



## References:

https://help.github.com/en/articles/splitting-a-subfolder-out-into-a-new-repository

https://www.firstinspires.org/sites/default/files/uploads/resource_library/ftc/android-studio-tutorial.pdf

