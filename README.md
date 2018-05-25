# MCAG_Base_Project
TwinCAT project prepared to be used with EPICS

EPICS model 3 motor driver for a TwinCAT 3 system

http://www.aps.anl.gov/epics

A general overview can be found here:

https://indico.fnal.gov/getFile.py/access?contribId=63&sessionId=12&resId=1&materialId=slides&confId=9718

This Git is today somewhat obsolete. To increases the maintainability,
this Git is split into different Gits:

- The TwinCAT part is moved into an own Git with submodules.

git clone --recursive https://bitbucket.org/europeanspallationsource/tc_generic_structure.git

- The EPICS model 3 motor driver is found here:

git clone https://bitbucket.org/europeanspallationsource/m-epics-ethercatmc.git
- You may want to use an improved version of the motorRecord:

git clone -b ess-6.9.x https://bitbucket.org/europeanspallationsource/m-epics-motor.git

There will be some more information, a kind of wiki.
This work is ongoing, so please contact us.

If you want to see an old version, just use

git checkout HEAD^

  