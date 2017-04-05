#!/bin/sh
#Shell script to load configuration binaries to TwinCAT MCU controller, make the project to autoboot and finally reboot MCU.
#The script need the TwinCAT boot folder of the MCU to be shared.
# Argument 1 = MCU IP ("192.168.88.60")
# Argument 2 = TwinCAT source prject binaries folder ("source/ChopperDemo/ChopperDemo/_Boot/TwinCAT RT (x86)")
# (AMSNetID will be derived from the IP-address by extending it with ".1.1" )
# or see the help below


ADSStateRelativePath="getADSState/getADSState.bin"
PlcBootMountFolderBase="plcbootfolder"
REFSYSTEMIP=192.168.88.60

#check number of arguments atleast two args are needed

if test $# -lt 2; then
  echo >&2 $0 "<IP> [<AMSNetID>] <TwinCatBinaryFolder>"
  echo >&2 example:
  echo >&2 $0 ${REFSYSTEMIP} '../../TwinCAT/MCAG_Base_Project/_Boot/TwinCAT\ RT\ \(x64\)/'
  echo >&2 $0 ${REFSYSTEMIP} ${REFSYSTEMIP}.1.1 '../../TwinCAT/MCAG_Base_Project/_Boot/TwinCAT\ RT\ \(x64\)/'
  exit 1
fi

if test $# -gt 3; then
  echo "Too Many arguments!"
  exit 2
fi

PLCIP=$1
if test $# -eq 2 ; then
  AMSAdr="$PLCIP.1.1"
  TwinCATBootFolder=$2
fi

if test $# -eq 3 ; then
  AMSAdr=$2
  TwinCATBootFolder=$3
fi

#Determine path to getADSState.bin (needs to be located ADSStateRelativePath from this script location)
SCRIPT=$(readlink -f "$0")
# Absolute path this script is in, thus /home/user/bin
SCRIPTPATH=$(dirname "$SCRIPT")
ADSStatePath=$SCRIPTPATH/$ADSStateRelativePath
ADSStateDir=$(dirname "$ADSStatePath")
ADSLibPath=$SCRIPTPATH/ADS

echo "Running the script with the following arguments: "
echo "MCU IP: $PLCIP"
echo "MCU AMSNetId: $AMSAdr"
echo "TwinCAT binaries folder: $TwinCATBootFolder"

#First check if controller is connected (ping)
echo "Check connection to MCU at $PLCIP."
if ping -c 1 $PLCIP >/dev/null; then
  echo "Connection to MCU at $PLCIP succeeded."
else
  echo "Connection to MCU at $PLCIP failed."
  exit 3
fi

#Generate temporary folder to mount MCU boot folder to
echo "Creating temporary folder for mounting to MCU boot folder"
count=0
found=0
# Maximum 100 tries to find name for new folder
while [ $count -lt 100 ]; do
  PlcBootMountFolder=$PlcBootMountFolderBase$count
  if ! test -d "$PlcBootMountFolder"; then
    count=99
    found=1
    mkdir $PlcBootMountFolder || {
      echo >&2 "failed1 mkdir $PlcBootMountFolder"
      exit 8
    }
  fi
  count=$(( $count + 1))
done
if test $found -ne 1; then
  echo >&2 "failed2 mkdir $PlcBootMountFolder"
  exit 8
fi
echo "Temporary folder created: $PlcBootMountFolder"

if mount | grep "$PlcBootMountFolder" >/dev/null; then
  echo "Unmount MCU boot folder of $PLCIP at $PlcBootMountFolder"
  sudo umount $PlcBootMountFolder/ >/dev/null || {
    echo >&2 "failed sudo umount $PlcBootMountFolder"
    exit 4
  }
fi

echo "Remounting MCU boot folder of $PLCIP at $PlcBootMountFolder"
id=$(id -u) &&
sudo mount.cifs //$PLCIP/C$/TwinCAT/3.1/Boot $PlcBootMountFolder/ -o user=Administrator,uid=$id || {
  echo >&2 can not mount
  exit 1
}

#echo "Clear MCU boot directory: $PlcBootMountFolder/*"
#NOT NEEDEDrm -rf $PlcBootMountFolder/*

echo "Changing access rights of binaries: $TwinCATBootFolder/*"
chmod -R 755 "$TwinCATBootFolder/"

echo "Copying binaries to MCU boot directory: $TwinCATBootFolder/*"
sudo cp -rv "$TwinCATBootFolder/"* $PlcBootMountFolder/  || {
  echo >&2 can not copy
  exit 1
}

echo "Adding autostart files for ADS port 851 and 852."
sudo touch "$PlcBootMountFolder/Plc/Port_851.autostart"
sudo touch "$PlcBootMountFolder/Plc/Port_852.autostart"

echo Unmount plcbootfolder of $PLCIP at $PlcBootMountFolder
sudo umount $PlcBootMountFolder  || {
  echo >&2 can not unmount
  exit 1
}

echo "Deleting temporary folder: $PlcBootMountFolder"
rm -r $PlcBootMountFolder

#Restart MCU
echo -n "Rebooting MCU:"
net rpc -S $PLCIP -U Administrator%1 shutdown -t 1 -f -r >/dev/null || {
  echo "Restart command failed."
  exit 4
}

#Wait for ping FAILURE
echo -n "Wait for shutdown of MCU $PLCIP "
count=100
echo -n 's'
while test $count -ne 0; do
  if ! ping -c 1 -i 0.2 -W 1 $PLCIP  >/dev/null; then
    count=0
  else
    count=$(($count - 1))
    sleep 0.7;
    echo -n 's'
    if test $count -eq 0; then
      echo "Timeout: MCU not rebooting."
      exit 5
    fi
  fi
done
echo "MCU $PLCIP has shut down, wait for reboot"

#Wait for ping SUCCESS
echo -n "Wait for boot of MCU $PLCIP "
count=100                            # Maximum number to try.
while test $count -ne 0; do
  if ping -c 1 -i 0.2 -W 1 $PLCIP  >/dev/null; then
    count=0                     # If okay, flag to exit loop.
  else
    count=$((count - 1))                  # So we don't go forever.
  fi
  echo -n "b"
done
echo "MCU $PLCIP has rebooted"

if test "$rc" -ne 0 ; then                  # Make final determination.
  echo "Timeout: MCU not started."
  exit 6
fi

if ! test -d $ADSLibPath; then
  (
    cd $SCRIPTPATH &&
    git clone https://github.com/Beckhoff/ADS.git &&
    (cd ADS && 
      git checkout d1e8b0d9fc7829a691fe87718e0e84bb8dbbff94 && 
      make ) &&
    (cd getADSState &&  make)
  )
fi  || {
  echo >&2 could not clone ADS
  exit 11
}

#Check TwinCAT
echo -n "Reconnecting to TwinCAT:"
count=100
echo -n 'R'
while test $count -ne 0 ; do
  $ADSStatePath $PLCIP $AMSAdr
  rc=$?
  echo ADSstate=$rc
  if test $rc -eq 5; then                #ADS state 5 = Running
    count=1                           # If okay, flag to exit loop.
  fi
  count=$((count - 1))                  # So we don't go forever.
  sleep 1;
  echo -n 'R'
done
echo ""
if test $rc -ne 5 ; then                  # Make final determination.
  echo "Timeout: TwinCAT not started."
  exit 7
fi
echo "TwinCAT started"
exit 0 # Success
