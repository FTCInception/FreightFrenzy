# Skystone
The code for the 2019-2020 season



Instructions to create repo with separate TeamCode repo:
  open git bash shell

  cd to any directory of your choice to place your code.  For example, I ran

$ cd /c/awork/ftc/
$ mkdir skystone2
$ cd skystone2
 
$ git clone https://github.com/FIRST-Tech-Challenge/SkyStone

$ cd SkyStone/TeamCode/src/main/java/
$ mkdir Inception
$ cd Inception
$ git clone https://github.com/FTCInception/Skystone
$ cd ../../../../..

$  git filter-branch --prune-empty --subdirectory-filter TeamCode/src/main/java/Inception master



References:
https://help.github.com/en/articles/splitting-a-subfolder-out-into-a-new-repository
https://www.firstinspires.org/sites/default/files/uploads/resource_library/ftc/android-studio-tutorial.pdf

