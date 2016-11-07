#!/bin/sh
APPXX=eemcu
export APPXX

../checkws.sh || exit 1
uname_s=$(uname -s 2>/dev/null || echo unknown)
uname_m=$(uname -m 2>/dev/null || echo unknown)

INSTALLED_EPICS=../../../.epics.$(hostname).$uname_s.$uname_m

if test -r $INSTALLED_EPICS; then
  echo INSTALLED_EPICS=$INSTALLED_EPICS
. $INSTALLED_EPICS
else
  echo not found: INSTALLED_EPICS=$INSTALLED_EPICS
fi


if test -z "$EPICS_BASE";then
  echo >&2 "EPICS_BASE" is not set
  exit 1
fi

makeCleanClean() {
  echo makeCleanClean
  test -d builddir && rm -rf builddir/
  make -f  Makefile.epics clean
  make -f  Makefile.epics uninstall
  make -f  Makefile.epics clean || :
}

../checkws.sh || {
  echo >&2 whitespace damage
  exit 2
}
MOTORIP=127.0.0.1
MOTORPORT=5000


cd startup &&
if ! test -f st.${1}.cmd; then
  CMDS=$(echo st.*.cmd | sed -e "s/st\.//g" -e "s/\.cmd//g")
  #echo CMDS=$CMDS
  test -n "$1" && echo >&2 "not found st.${1}.cmd"
  echo >&2 "try one of these:"
  for cmd in $CMDS; do
    echo >&2 $0 " $cmd" " <ip>[:port]"
  done
  exit 1
fi &&
cd ..

#Need a add a dot, needs to be improved later
MOTORCFG=".$1"
export MOTORCFG
echo MOTORCFG=$MOTORCFG
shift

if test -n "$1"; then
  # allow doit.sh host:port
  PORT=${1##*:}
  HOST=${1%:*}
  echo HOST=$HOST PORT=$PORT
  if test "$PORT" != "$HOST"; then
    MOTORPORT=$PORT
  fi
  echo HOST=$HOST MOTORPORT=$MOTORPORT
  MOTORIP=$HOST
  echo MOTORIP=$MOTORIP
fi
export MOTORIP MOTORPORT
setttings_file=./.set_${uname_s}_${uname_m}.txt
oldsetttings_file=./.set_${uname_s}_${uname_m}.old.txt
export setttings_file oldsetttings_file

set | grep EPICS_ | sort >"$setttings_file"
if ! test -f "$oldsetttings_file"; then
  make_clean_uninstall=y
  (
    makeCleanClean
  )
  cp "$setttings_file" "$oldsetttings_file"
else
 if ! diff "$oldsetttings_file" "$setttings_file" ; then
   make_clean_uninstall=y
   (
     . "$oldsetttings_file"
     makeCleanClean
   )
   rm -f "$oldsetttings_file"
 fi
fi

if ! test -d ${APPXX}App; then
  makeBaseApp.pl -t ioc $APPXX
fi &&
if ! test -d iocBoot; then
  makeBaseApp.pl -i -t ioc $APPXX
fi &&

if test -z "$EPICS_HOST_ARCH"; then
  echo >&2 EPICS_HOST_ARCH is not set
  exit 1
fi &&
TOP=$PWD &&
if test -d $EPICS_BASE/../modules/motor/Db; then
  EPICS_MOTOR_DB=$EPICS_BASE/../modules/motor/Db
elif test -d $EPICS_BASE/../modules/motor/db; then
  EPICS_MOTOR_DB=$EPICS_BASE/../modules/motor/db
elif test -d $EPICS_BASE/../modules/motor/dbd; then
  EPICS_MOTOR_DB=$EPICS_BASE/../modules/motor/dbd
elif test -n "$EPICS_BASES_PATH"; then
   echo >&2 found: EPICS_BASES_PATH=$EPICS_BASES_PATH
   echo >&2        EPICS_BASE=$EPICS_BASE
   mybasever=$(echo $EPICS_BASE | sed -e "s!^$EPICS_BASES_PATH/base-!!")
   echo >&2 mybasever=$mybasever
   EPICS_MOTOR_DB=$EPICS_MODULES_PATH/motor/6.10.6-ESS/$mybasever/dbd
   echo >&2 EPICS_MOTOR_DB=$EPICS_MOTOR_DB
   if ! test -d "$EPICS_MOTOR_DB"; then
     echo >&2 Not found EPICS_MOTOR_DB=$EPICS_MOTOR_DB
     exit 1
   fi
   EPICS_EEE=y
   #export EPICS_EEE make_clean_uninstall
   export EPICS_EEE
else
   echo >&2 Not found: $EPICS_BASE/../modules/motor/[dD]b
   echo >&2 Unsupported EPICS_BASE:$EPICS_BASE
  exit 1
fi &&
if ! test -d "$EPICS_MOTOR_DB"; then
  echo >&2 $EPICS_MOTOR_DB does not exist
  exit 1
fi
(
  cd configure &&
  if ! test -f RELEASE_usr_local; then
    git mv RELEASE RELEASE_usr_local
  fi &&
  sed <RELEASE_usr_local >RELEASE \
  -e "s%^EPICS_BASE=.*$%EPICS_BASE=$EPICS_BASE%" &&
  if  test -f MASTER_RELEASE; then
    if ! test -f MASTER_RELEASE_usr_local; then
      git mv MASTER_RELEASE MASTER_RELEASE_usr_local
    fi &&
    sed <MASTER_RELEASE_usr_local >MASTER_RELEASE \
      -e "s%^EPICS_BASE=.*$%EPICS_BASE=$EPICS_BASE%"
  fi
) &&
if test "x$make_clean_uninstall" = xy; then
  makeCleanClean
fi &&

if test "x$EPICS_EEE" = "xy"; then
  make install || {
    echo >&2 EEE
    exit 1
  }
else
  make -f Makefile.epics || {
    rm -f "$oldsetttings_file"
    make -f  Makefile.epics clean && make -f Makefile.epics  ||  exit 1
  }
fi
(
  stcmddst=./st.cmd${MOTORCFG}.$EPICS_HOST_ARCH &&
  cd ./iocBoot/ioc${APPXX}/ &&
  if test "x$EPICS_EEE" = "xy"; then
    #EEE
    cd ../../startup &&
    stcmddst=./st.cmd${MOTORCFG}.EEE.tmp &&
    rm -f $stcmddst &&
    sed  <st${MOTORCFG}.cmd  \
      -e "s/require eemcu/require eemcu,$USER/" \
      -e "s/^cd /#cd /" \
      -e "s/127.0.0.1/$MOTORIP/" \
      -e "s/5000/$MOTORPORT/" |
    grep -v '^  *#' >$stcmddst || {
      echo >&2 can not create stcmddst $stcmddst
      exit 1
    }
    chmod -w $stcmddst &&
    chmod +x $stcmddst &&
    cmd=$(echo iocsh $stcmddst) &&
    echo PWD=$PWD cmd=$cmd &&
    eval $cmd
  else
    # classic EPICS, non EEE
    # We need to patch the cmd files to adjust dbLoadRecords
    for src in  ../../startup/*; do
      dst=${src##*/}
      echo PWD=$PWD src=$src dst=$dst
      sed <"$src" >"$dst" \
          -e "s%dbLoadRecords(\"%dbLoadRecords(\"./${APPXX}App/Db/%"
    done
    rm -f $stcmddst &&
    cat >$stcmddst <<EOF &&
#!../../bin/$EPICS_HOST_ARCH/${APPXX}
#This file is autogenerated by doit.sh - do not edit
epicsEnvSet("ARCH","$EPICS_HOST_ARCH")
epicsEnvSet("IOC","ioc${APPXX}")
epicsEnvSet("TOP","$TOP")
epicsEnvSet("EPICS_BASE","$EPICS_BASE")

cd ${TOP}
dbLoadDatabase "dbd/${APPXX}.dbd"
eemcu_registerRecordDeviceDriver pdbbase
EOF
      sed <../../startup/st${MOTORCFG}.cmd  \
      -e "s/__EPICS_HOST_ARCH/$EPICS_HOST_ARCH/" \
      -e "s/127.0.0.1/$MOTORIP/" \
      -e "s/5000/$MOTORPORT/" \
      -e "s%cfgFile=../iocBoot/ioceemcu/%cfgFile=./%"    \
      -e "s%< *%< $TOP/iocBoot/ioceemcu/%" \
      -e "s%require%#require%" \
        | grep -v '^  *#' >>$stcmddst &&

    cat >>$stcmddst <<-EOF &&
  cd ${TOP}/iocBoot/ioc${APPXX}
  iocInit
EOF
    chmod -w $stcmddst &&
    chmod +x $stcmddst &&
    egrep -v "^ *#" st.gdb.$EPICS_HOST_ARCH >xx
    echo PWD=$PWD $stcmddst
    $stcmddst
  fi
)

