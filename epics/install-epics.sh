#!/bin/sh

EPICS_DOWNLOAD=$PWD/installed-epics

uname_s=$(uname -s 2>/dev/null || echo unknown)
uname_m=$(uname -m 2>/dev/null || echo unknown)

INSTALLED_EPICS=./.epics.$(hostname).$uname_s.$uname_m

#Version of base
EPICS_BASE_VER=3.15.2
BASE_VER=GIT
#EPICS_BASE_GIT_VER=R3.14.12.5
EPICS_BASE_GIT_VER=R3.15.2


#Version for synApps
#SYNAPPSVER=5_8

#Version for ASYN
#ASYNVER=4-21
ASYNVER=GIT
ASYN_GIT_VER=R4-26

MOTORVER=GIT
#MOTOR_GIT_VER=204515c143c6b7ad55054
MOTOR_GIT_VER=R6-8-1
#MOTOR_GIT_VER=d5be8003ba
#http://www.aps.anl.gov/bcda/synApps/motor/tar/motorR6-8-1.tar.gz
#http://www.aps.anl.gov/bcda/synApps/motor/tar/motorR6-9.tar.gz

#msi
EPICS_MSI_VER=msi1-5

if test -n "$SYNAPPSVER"; then
  SYNAPPS_VER_X_Y=synApps_$SYNAPPSVER
fi

# Debug version for e.g. kdbg
if test "$EPICS_DEBUG" = ""; then
  if type kdbg; then
    EPICS_DEBUG=y
  fi
fi
#Where are the binaries of EPICS
if ! test "$EPICS_DOWNLOAD"; then
  EPICS_DOWNLOAD=/usr/local/epics
fi

EPICS_ROOT=$EPICS_DOWNLOAD/EPICS_BASE_${EPICS_BASE_VER}
if test -n "$BASE_VER"; then
  EPICS_ROOT=${EPICS_ROOT}_GIT
fi
if test -n "$ASYNVER"; then
  ASYN_VER_X_Y=asyn$ASYNVER
  EPICS_ROOT=${EPICS_ROOT}_ASYN_${ASYNVER}
fi
if test -n "$SYNAPPSVER"; then
  EPICS_ROOT=${EPICS_ROOT}_SYNAPPS_${SYNAPPSVER}
fi
if test -n "$MOTORVER"; then
  MOTOR_VER_X_Y=motorR$MOTORVER$MOTOR_GIT_VER
  EPICS_ROOT=${EPICS_ROOT}_MOTOR_${MOTORVER}_${MOTOR_GIT_VER}
fi

if test "$EPICS_DEBUG" = y; then
  EPICS_ROOT=${EPICS_ROOT}_DBG
fi
EPICS_BASE=$EPICS_ROOT/base
EPICS_MODULES=$EPICS_ROOT/modules

echo EPICS_ROOT=$EPICS_ROOT
EPICS_ROOT=$(echo $EPICS_ROOT | sed -e "s%/[^/][^/]*/\.\./%/%")



echo EPICS_ROOT=$EPICS_ROOT
echo Do you want to install EPICS in $EPICS_ROOT ? [y/N]
read yesno
case $yesno in
  y|Y)
  ;;
  *)
  exit 0
esac

if $(echo "$EPICS_ROOT" | grep -q /usr/local); then
	echo EPICS_ROOT=$EPICS_ROOT
	echo EPICS_DOWNLOAD=$EPICS_DOWNLOAD
	if ! test -w "$EPICS_DOWNLOAD"; then
		FSUDO=sudo
	fi
fi

CP="$FSUDO cp"
LN="$FSUDO ln"
MKDIR="$FSUDO mkdir"
MV="$FSUDO mv"
RM="$FSUDO rm"

export CP FSUDO LN MKDIR MV RM SUDO

case "$SYNAPPS_VER_X_Y" in
  synApps_5_6)
  MODSTOBEREMOVED="ALLEN_BRADLEY DAC128V IP330 IPUNIDIG LOVE IP VAC SOFTGLUE QUADEM DELAYGEN CAMAC VME AREA_DETECTOR DXP"
  MODSFROMMAKEFILE='ADSupport NDPlugin simDetector netCDF dxp "xxx_Common_LIBS += ip"'
  ;;
  synApps_5_7)
  MODSTOBEREMOVED="ALLEN_BRADLEY AREA_DETECTOR AUTOSAVE CAMAC DAC128V DXP DELAYGEN IP IP330 IPUNIDIG LOVE MCA MEASCOMP OPTICS QUADEM SOFTGLUE STD SNCSEQ VAC VME"
  MODSFROMMAKEFILE='ADSupport NDPlugin simDetector netCDF dxp "xxx_Common_LIBS += ip"'
  ;;
  synApps_5_8)
  MODSTOBEREMOVED="ALLEN_BRADLEY AREA_DETECTOR AUTOSAVE CAMAC DAC128V DXP DELAYGEN IP IP330 IPUNIDIG LOVE MCA MEASCOMP OPTICS QUADEM SOFTGLUE STD SNCSEQ VAC VME"
  MODSFROMMAKEFILE='ADSupport NDPlugin simDetector dxp'
  ;;
  '')
  ;;
  *)
  echo >&2 "Invalid version for synapps $SYNAPPS_VER_X_Y"
  exit 1
  ;;
esac

#extensions top
EPICS_EXTENSIONS_TOP_VER=extensionsTop_20120904


#StreamDevice, later version than synApps
#STREAMDEVICEVER=StreamDevice-2-6

export EPICS_ROOT EPICS_BASE EPICS_MODULES EPICS_BASE_VER EPICS_ROOT EPICS_DEBUG
export EPICS_EXT=${EPICS_ROOT}/extensions
#########################
#apt or yum or port
if uname -a | egrep "CYGWIN|MING" >/dev/null; then
  SUDO=
else
  SUDO=sudo
fi
APTGET=/bin/false
if type apt-get >/dev/null 2>/dev/null; then
  APTGET="$SUDO apt-get install"
fi
if type yum >/dev/null 2>/dev/null; then
  APTGET="$SUDO /usr/bin/yum install"
fi
# port (Mac Ports)
if test -x /opt/local/bin/port; then
  APTGET="$SUDO port install"
fi
export APTGET
#########################

create_soft_x_y() {
  dir=$1
  src=$2
  dst=$3
  echo dir=$dir create_soft_x_y "$@"
  export dir src dst
  (
    cd "$dir" &&
    linkdst=$(readlink $dst) || linkdst=""
    if ! test "$linkdst" || test "$linkdst" != "$src"; then
      # unlink, first as user, then as SUDO
      if test "$linkdst" != "$src"; then
        echo "$linkdst" != "$dst" &&
        echo PWD=$PWD $RM $dst &&
        $RM -f $dst &&
        echo PWD=$PWD $LN -s $src $dst &&
        $LN -s $src $dst || {
          echo >&2 can not link $src $dst
          exit 1
        }
      fi
    fi
  )
}


########################
if ! test -d $EPICS_ROOT; then
  echo $MKDIR -p $EPICS_ROOT &&
  $MKDIR -p $EPICS_ROOT || {
    echo >&2 can not $MKDIR $EPICS_ROOT
    exit 1
  }
fi

if ! test -w $EPICS_ROOT; then
  echo FSUDO=$FSUDO
  echo $FSUDO chown "$USER" $EPICS_ROOT &&
  $FSUDO chown "$USER" $EPICS_ROOT || {
    echo >&2 can not chown $EPICS_ROOT
    exit 1
  }
else
  echo FSUDO=
  export FSUDO
fi
echo FSUDO=$FSUDO

if ! test -d /usr/local; then
  sudo $MKDIR /usr/local
fi &&
create_soft_x_y $EPICS_ROOT base-${EPICS_BASE_VER} base &&


wget_or_curl()
{
  url=$1
  file=$2
  if test -e $file; then
    return;
  fi
  (
    echo cd $EPICS_DOWNLOAD &&
    cd $EPICS_DOWNLOAD &&
    if ! test -e $file; then
        if type curl >/dev/null 2>/dev/null; then
            curl "$url" >/tmp/"$file.$$.tmp" &&
              $MV "/tmp/$file.$$.tmp" "$file" || {
                echo >&2 curl can not get $url
                exit 1
              }
        else
          # We need wget
          if ! type wget >/dev/null 2>/dev/null; then
              echo $APTGET wget
              $APTGET wget
          fi &&
            wget "$url" -O "$file.$$.tmp" &&
            $MV "$file.$$.tmp" "$file" || {
              echo >&2 wget can not get $url
              exit 1
            }
        fi
      fi
  ) &&
  $LN -s $EPICS_DOWNLOAD/$file $file
}

#add package x when y is not there
addpacketifneeded() {
  needed=$1
  tobeinstalled=$2
  if test -z "$tobeinstalled"; then
    tobeinstalled=$needed
  fi
  if ! which $needed ; then
    echo $APTGET $tobeinstalled
    $APTGET $tobeinstalled
  fi
}

install_re2c()
{
  cd $EPICS_ROOT &&
  if ! test -d re2c-code-git; then
    git clone git://git.code.sf.net/p/re2c/code-git re2c-code-git.$$.tmp &&
    $MV re2c-code-git.$$.tmp  re2c-code-git
  fi &&
  (
    cd re2c-code-git/re2c &&
    addpacketifneeded automake &&
    ./autogen.sh &&
    ./configure &&
    make &&
    echo PWD=$PWD $FSUDO make install &&
    $FSUDO make install
  )
}
run_make_in_dir()
{
  dir=$1 &&
  echo cd $dir &&
  (
    cd $dir &&
    $FSUDO make || {
    echo >&2 PWD=$PWD Can not make
    exit 1
  }
  )
}

install_asyn_ver()
{
  echo install_motor_from_synapps
  asyndir="$1"/
  cd $EPICS_ROOT/modules &&
  if test -L asyn; then
    echo $RM asyn &&
    $RM asyn
  fi &&
  test -d $asyndir || {
    echo >&2 PWD=$PWD Can not $LN -sv $asyndir asyn
    exit 1
  }
  $LN -sv $asyndir asyn || {
    echo >&2 Can not $LN -sv $asyndir asyn
    exit 1
  }
}

patch_motor_h()
{
  (
    cd "$1" &&
    if grep "epicsEndian.h" motor.h >/dev/null; then
      echo PWD=$PWD patch motor.h not needed &&
      return
    fi &&
    echo PWD=$PWD patch motor.h &&
    if ! test -e motor.h.original; then
      $CP motor.h motor.h.original
    fi &&
    $CP motor.h.original motor.h &&
    case $PWD in
      *motor-6-7*)
      cat <<EOF >motor.patch
diff --git a/motorApp/MotorSrc/motor.h b/motorApp/MotorSrc/motor.h
--- a/motorApp/MotorSrc/motor.h
+++ b/motorApp/MotorSrc/motor.h
63a64
> #include <epicsEndian.h>
140c141
< #elif ($CPU == PPC604) || ($CPU == PPC603) || ($CPU==PPC85XX) || ($CPU == MC68040) || ($CPU == PPC32)
---
> #elif defined($CPU) && (($CPU == PPC604) || ($CPU == PPC603) || ($CPU == PPC85XX) || ($CPU == MC68040) || ($CPU == PPC32))
141a143,148
> #elif defined(__GNUC__)
>     #if (EPICS_BYTE_ORDER == EPICS_ENDIAN_LITTLE)
>         #define LSB_First (TRUE)
>     #else
>         #define MSB_First (TRUE)
>     #endif
EOF
      ;;
      *motor-6-8*)
      cat <<EOF >motor.patch
diff --git a/motorApp/MotorSrc/motor.h b/motorApp/MotorSrc/motor.h
--- a/motorApp/MotorSrc/motor.h
+++ b/motorApp/MotorSrc/motor.h
63a64
> #include <epicsEndian.h>
140c141
< #elif ($CPU == PPC604) || ($CPU == PPC603) || ($CPU == PPC85XX) || ($CPU == MC68040) || ($CPU == PPC32)
---
> #elif defined($CPU) && (($CPU == PPC604) || ($CPU == PPC603) || ($CPU == PPC85XX) || ($CPU == MC68040) || ($CPU == PPC32))
141a143,148
> #elif defined(__GNUC__)
>     #if (EPICS_BYTE_ORDER == EPICS_ENDIAN_LITTLE)
>         #define LSB_First (TRUE)
>     #else
>         #define MSB_First (TRUE)
>     #endif
EOF
      ;;
      *)
      echo >&2 "Can not patch motor.h, only motor 6.7 or 6.8 is supported"
      exit 1
    esac &&
    patch <motor.patch
  )
}


install_motor_X_Y ()
{
  echo install_motor_X_Y
  create_soft_x_y $EPICS_ROOT/modules ../$MOTOR_VER_X_Y/ motor
  (
    cd $EPICS_ROOT &&
      if test "$MOTORVER" = GIT && test -n "$MOTOR_GIT_VER"; then
        if ! test -d $MOTOR_VER_X_Y; then
						(
              $FSUDO git clone https://github.com/epics-modules/motor.git $MOTOR_VER_X_Y  &&
              cd $MOTOR_VER_X_Y &&
							$FSUDO git checkout $MOTOR_GIT_VER
					)||
            ( $RM -rf $MOTOR_VER_X_Y; false )
        fi
      else
        if ! test -f $MOTOR_VER_X_Y.tar.gz; then
          wget_or_curl http://www.aps.anl.gov/bcda/synApps/motor/tar/$MOTOR_VER_X_Y.tar.gz $MOTOR_VER_X_Y.tar.gz
        fi
        if ! test -d $MOTOR_VER_X_Y; then
          tar xzvf $MOTOR_VER_X_Y.tar.gz
        fi
      fi
  ) &&
  (
    # Need to fix epics base for synapss already here,
    # (if the dir already exists)
    path=$EPICS_ROOT/$SYNAPPS_VER_X_Y/support/configure &&
      if test -d $path; then
          echo cd $path &&
            cd $path &&
            (
              fix_epics_base EPICS_BASE.$EPICS_HOST_ARCH &&
              fix_epics_base SUPPORT.$EPICS_HOST_ARCH
            )
      fi
  ) &&
  if test -n "$MOTORVER"; then
    (
      cd $EPICS_ROOT/$MOTOR_VER_X_Y/configure && {
        for f in $(find . -name "RELEASE*" ); do
          echo f=$f
          fix_epics_base $f
        done
      }
    ) &&
    if ! test "$MOTOR_GIT_VER" = master; then
      (
        cd $EPICS_ROOT/$MOTOR_VER_X_Y/motorApp &&
        comment_out_in_file Makefile HytecSrc AerotechSrc
      ) 
    fi &&
    (
      echo run_make_in_dir $EPICS_ROOT/$MOTOR_VER_X_Y &&
      run_make_in_dir $EPICS_ROOT/$MOTOR_VER_X_Y &&
      echo done run_make_in_dir $EPICS_ROOT/$MOTOR_VER_X_Y
    ) || {
      echo >&2 failed $MOTOR_VER_X_Y
      exit 1
    }
  fi
}


install_motor_from_synapps()
{
  echo install_motor_from_synapps
  cd $EPICS_ROOT/modules &&
  if test -e motor; then
    echo $RM -rf motor &&
    $RM -rf motor
  fi &&
  $MKDIR -p motor &&
  cd motor &&
  motordevver=$(echo ../../$SYNAPPS_VER_X_Y/support/motor-*) &&
  echo motordevver=$motordevver &&
  for f in src dbd Db lib include; do
    if test -e $f; then
      echo $RM -rf $f &&
      $RM -rf $f
    fi
  done &&
  (
    $MKDIR dbd &&
    cd dbd &&
    $RM -rf * &&
    for mdbd in $(find ../../../$SYNAPPS_VER_X_Y/support/motor-* -name '*.dbd'); do
      dbdbasename="${mdbd##*/}" &&
      #echo mdbd=$mdbd dbdbasename=$dbdbasename &&
      if ! test -f $dbdbasename; then
        $CP -fv $mdbd $dbdbasename
      fi
    done
  ) &&
  (
    $MKDIR Db &&
    cd Db &&
    $RM -rf * &&
    for mdbd in $(find ../../../$SYNAPPS_VER_X_Y/support/motor-* -name '*.db'); do
      dbdbasename="${mdbd##*/}" &&
      #echo mdbd=$mdbd dbdbasename=$dbdbasename &&
      if ! test -f $dbdbasename; then
        $CP -fv $mdbd $dbdbasename
      fi
    done
  ) &&
  (
    motorlib=$(find ../../$SYNAPPS_VER_X_Y/support/motor-*/ -name lib | sed -e "s!//!/!g");
    echo motorlib=$motorlib
    $LN -s "$motorlib" lib
  ) &&
  (
    motorinclude=$(find ../../$SYNAPPS_VER_X_Y/support/motor-*/ -name include | sed -e "s!//!/!g");
    echo motorinclude=$motorinclude
    $LN -s "$motorinclude" include
  )
}

install_streamdevice()
{
  cd $EPICS_ROOT/modules &&
  if ! test -d streamdevice; then
    $MKDIR -p streamdevice
  fi &&
  cd streamdevice &&
  streamdevver=$(echo ../../$SYNAPPS_VER_X_Y/support/stream-*) &&
  echo streamdevver=$streamdevver &&
  if test -L src; then
    echo $RM src &&
    $RM src
  fi &&
  echo $LN -s ../../$SYNAPPS_VER_X_Y/support/$streamdevver/streamDevice/src/ src &&
  $LN -s ../../$SYNAPPS_VER_X_Y/support/$streamdevver/streamDevice/src/ src || exit 1
  for f in dbd lib include; do
    if test -L $f; then
      echo $RM $f &&
      $RM $f
    fi &&
    $LN -s ../../$SYNAPPS_VER_X_Y/support/$streamdevver/$f/ $f || exit 1
  done
}

patch_CONFIG_gnuCommon()
{
  (
    file=CONFIG.gnuCommon
    export file
    cd "$1" &&
    if grep "OPT_CXXFLAGS_NO *= *-g *-O0" $file >/dev/null; then
      echo PWD=$PWD patch $file not needed &&
      return
    fi &&
    echo PWD=$PWD patch $file &&
    if ! test -e $file.original; then
      $CP $file $file.original
    fi &&
    $CP $file.original $file &&
    case $PWD in
      *3.14.12.3*|*3.14.12.5*|*3.15.1*|*3.15.2*)
      cat <<\EOF > "$file.patch"
diff --git a/CONFIG.gnuCommon b/CONFIG.gnuCommon
index f054802..d59a420 100644
--- a/CONFIG.gnuCommon
+++ b/CONFIG.gnuCommon
@@ -27,16 +27,16 @@ GPROF_CFLAGS_YES = -pg
 CODE_CFLAGS = $(PROF_CFLAGS_$(PROFILE)) $(GPROF_CFLAGS_$(GPROF))
 WARN_CFLAGS_YES = -Wall
 WARN_CFLAGS_NO = -w
-OPT_CFLAGS_YES = -O3
-OPT_CFLAGS_NO = -g
+OPT_CFLAGS_YES = -O0 -g
+OPT_CFLAGS_NO = -g -O0
 
 PROF_CXXFLAGS_YES = -p
 GPROF_CXXFLAGS_YES = -pg
 CODE_CXXFLAGS = $(PROF_CXXFLAGS_$(PROFILE)) $(GPROF_CXXFLAGS_$(GPROF))
 WARN_CXXFLAGS_YES = -Wall
 WARN_CXXFLAGS_NO = -w
-OPT_CXXFLAGS_YES = -O3
-OPT_CXXFLAGS_NO = -g
+OPT_CXXFLAGS_YES = -O0 -g
+OPT_CXXFLAGS_NO = -g -O0
 
 CODE_LDFLAGS = $(PROF_CXXFLAGS_$(PROFILE)) $(GPROF_CXXFLAGS_$(GPROF))
 
EOF
      ;;
      *)
      echo >&2 "PWD=$PWD Can not patch $file, not supported"
      exit 1
    esac &&
    patch < "$file.patch"
  )
}



fix_epics_base()
{
  file="$1" &&
  if test -e "$file"; then
    filebasename="${file%*.original}" &&
    echo fix_epics_base PWD=$PWD file=$file filebasename=$filebasename &&
    if ! test -f "$filebasename.original"; then
      $CP "$file" "$filebasename.original" || {
        echo >&2 failed $CP -v $file $filebasename.original in $PWD
        exit 1
      }
    fi &&
    sed <"$filebasename.original" >/tmp/$$.tmp \
      -e "s!^SUPPORT=.*!SUPPORT=$EPICS_ROOT/$SYNAPPS_VER_X_Y/support!" \
      -e "s!^EPICS_BASE=.*!EPICS_BASE=$EPICS_ROOT/base!" \
      -e "s!^\(IPAC=.*\)!## rem by install-epics \1!" \
      -e "s!^BUSY=.*!BUSY=\$(SUPPORT)/busy-1-6!" \
      -e "s!^\(SNCSEQ=.*\)!## rem by install-epics \1!"
      $MV -fv /tmp/$$.tmp "$file" &&
      if test "$ASYN_VER_X_Y"; then
        sed <"$file" >/tmp/$$.tmp \
          -e "s!^ASYN=.*!ASYN=$EPICS_MODULES/asyn!" &&
        $MV -fv /tmp/$$.tmp "$file"
    fi
  else
    echo fix_epics_base PWD=$PWD file=$file does not exist, doing nothing
  fi
}


remove_modules_from_RELEASE()
{
  file="$1" &&
  for mod in $MODSTOBEREMOVED; do
    echo removing $mod in $PWD/$file &&
    if grep $mod $file >/dev/null; then
      sed -e "s/\($mod=.*\$\)/## xx \1/g" <$file >$file.$$.tmp &&
      ! diff $file $file.$$.tmp >/dev/null &&
      $MV -f $file.$$.tmp $file || {
        echo >&2 failed removing $mod in $PWD
        exit 1
      }
    fi
  done
}

remove_modules_from_Makefile()
{
  file="$1" &&
  for mod in $MODSTOBEREMOVED; do
    echo removing $mod in $PWD/$file &&
    sed -e "s/ $mod / /g" -e "s/ $mod\$/ /g" <$file >$file.$$.tmp &&
    ! diff $file $file.$$.tmp >/dev/null &&
    $MV -f $file.$$.tmp $file || {
      echo >&2 failed removing $mod in $PWD
      exit 1
    }
  done
}

comment_out_in_file()
{
  suffix=$$
  file=$1 &&
  shift &&
  if ! test -f "$filebasename.original"; then
    $CP "$file" "$filebasename.original" || {
      echo >&2 failed $CP -v $file $filebasename.original in $PWD
      exit 1
    }
  fi &&
  $CP "$filebasename.original" "$file" &&
  for mod in "$@"; do
    if grep "^#.*$mod" $file >/dev/null; then
      echo already commented out $mod in $PWD/$file
    else
      echo commenting out $mod in $PWD/$file &&
      filebasename="${file%*.original}" &&
      echo file=$file filebasename=$filebasename &&
      sed -e "s/\(.*$mod.*\)/# rem by install-epics \1/g" <$file >/tmp/xx.$$ &&
      ! diff  $file $file.suffix >/dev/null &&
      $MV -f /tmp/xx.$$ $file
    fi
  done
}


(
  case "$EPICS_BASE_VER" in
    3.15.1|3.15.2)
    SEP="-"
    ;;
    3.14.12.*)
    SEP=R
    ;;
    *)
    exit 0
  esac

  cd $EPICS_ROOT &&

  if ! test -d base-$EPICS_BASE_VER; then
    #if test "$EPICS_BASE_VER" = GIT; then
    if /bin/true; then
      git clone https://github.com/epics-base/epics-base.git base-$EPICS_BASE_VER &&
      (
        cd base-$EPICS_BASE_VER && git checkout $EPICS_BASE_GIT_VER
      )
    else
      if ! test -f base${SEP}${EPICS_BASE_VER}.tar.gz; then
        wget_or_curl http://www.aps.anl.gov/epics/download/base/base${SEP}${EPICS_BASE_VER}.tar.gz base${SEP}${EPICS_BASE_VER}.tar.gz
      fi
      if ! test -d base-$EPICS_BASE_VER; then
        tar xzf base${SEP}${EPICS_BASE_VER}.tar.gz || {
          echo >&2 can not tar xzf base${SEP}${EPICS_BASE_VER}.tar.gz
          $RM -rf base-$EPICS_BASE_VER
          exit 1
        }
      fi
    fi
  fi
) || exit 1

#Need to set the softlink now

if test -n "$EPICS_HOST_ARCH"; then
EPICS_HOST_ARCH=$($EPICS_ROOT/base-${EPICS_BASE_VER}/startup/EpicsHostArch) || {
  echo >&2 EPICS_HOST_ARCH failed
  exit 1
}
fi
# here we know the EPICS_HOST_ARCH
export EPICS_HOST_ARCH
EPICS_BASE_BIN=${EPICS_BASE}/bin/$EPICS_HOST_ARCH
EPICS_EXT_BIN=${EPICS_EXT}/bin/$EPICS_HOST_ARCH
PATH=$PATH:$EPICS_BASE_BIN:$EPICS_EXT_BIN
EPICS_EXT_LIB=${EPICS_EXT}/lib/$EPICS_HOST_ARCH
if test "${LD_LIBRARY_PATH}"; then
  export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$EPICS_BASE_LIB
else
  export LD_LIBRARY_PATH=$EPICS_EXT_LIB
fi
echo PATH=$PATH
export EPICS_BASE_BIN EPICS_EXT EPICS_EXT_LIB EPICS_EXT_BIN PATH LD_LIBRARY_PATH
#update .epics
cat >${INSTALLED_EPICS} <<EOF &&
export EPICS_DEBUG=$EPICS_DEBUG
export EPICS_DOWNLOAD=$EPICS_DOWNLOAD
export EPICS_ROOT=$EPICS_ROOT
export EPICS_BASE=\$EPICS_ROOT/base
export EPICS_EXT=\${EPICS_ROOT}/extensions
export EPICS_HOST_ARCH=$($EPICS_BASE/startup/EpicsHostArch)
export EPICS_EXT_BIN=${EPICS_EXT}/bin/\$EPICS_HOST_ARCH
export EPICS_EXT_LIB=${EPICS_EXT}/lib/\$EPICS_HOST_ARCH
export EPICS_MODULES=\$EPICS_ROOT/modules
export EPICS_BASE_BIN=\${EPICS_BASE}/bin/\$EPICS_HOST_ARCH
export EPICS_BASE_LIB=\${EPICS_BASE}/lib/\$EPICS_HOST_ARCH
export LD_LIBRARY_PATH=\${EPICS_BASE_LIB}:\$LD_LIBRARY_PATH
if test "\$LD_LIBRARY_PATH"; then
  export LD_LIBRARY_PATH=\$LD_LIBRARY_PATH:\$EPICS_BASE_LIB
else
  export LD_LIBRARY_PATH=\$EPICS_EXT_LIB
fi
export PATH=\$PATH:\$EPICS_BASE_BIN:\$EPICS_EXT_BIN
EOF

$CP $INSTALLED_EPICS $EPICS_ROOT/.epics.$EPICS_HOST_ARCH &&
################

(
  # We need gcc and g++: gcc-g++ under Scientifc Linux
  if ! type g++ >/dev/null 2>/dev/null; then
    echo $APTGET gcc-c++
    $APTGET gcc-c++
  fi
  # We need g++
  if ! type g++ >/dev/null 2>/dev/null; then
    echo $APTGET g++
    $APTGET g++
  fi &&
  #We need readline
  # Mac OS: /usr/include/readline/readline.h
  # Linux: /usr/include/readline.h
  if ! test -r /usr/include/readline/readline.h; then
    test -r /usr/include/readline.h ||
    $APTGET readline-devel ||
    $APTGET libreadline-dev ||
    {
      echo >&2 can not install readline-devel
      exit 1
    }
  fi &&
  if test "$EPICS_DEBUG" = y; then
    patch_CONFIG_gnuCommon $EPICS_ROOT/base/configure
  fi &&
  run_make_in_dir $EPICS_ROOT/base || {
    echo >&2 failed in $PWD
    exit 1
  }
) &&

#Modules
(
  cd $EPICS_ROOT/ &&
  if ! test -d modules; then
    $MKDIR modules
  fi
) || exit 1



#Streamdevice from PSI
if test -n "$STREAMDEVICEVER"; then
  (
    cd $EPICS_ROOT &&
    if ! test -f $STREAMDEVICEVER.tgz; then
      wget_or_curl http://epics.web.psi.ch/software/streamdevice/$STREAMDEVICEVER.tgz $STREAMDEVICEVER.tgz.$$
    fi
    if ! test -d $STREAMDEVICEVER; then
      tar xzvf $STREAMDEVICEVER.tgz
    fi
    if ! test -d $EPICS_ROOT/$STREAMDEVICEVER/streamdevice-2.6/configure; then
      $MKDIR -p $EPICS_ROOT/$STREAMDEVICEVER/streamdevice-2.6/configure
    fi
    (
      # Create the files (Obs: \EOF != EOF)
      cd $EPICS_ROOT/$STREAMDEVICEVER/streamdevice-2.6/configure &&
      cat >CONFIG <<\EOF &&
#Generated by install-epics.sh
# CONFIG - Load build configuration data
#
# Do not make changes to this file!

# Allow user to override where the build rules come from
RULES = $(EPICS_BASE)

# RELEASE files point to other application tops
include $(TOP)/configure/RELEASE
-include $(TOP)/configure/RELEASE.$(EPICS_HOST_ARCH).Common
ifdef T_A
-include $(TOP)/configure/RELEASE.Common.$(T_A)
-include $(TOP)/configure/RELEASE.$(EPICS_HOST_ARCH).$(T_A)
endif
CONFIG = $(RULES)/configure
include $(CONFIG)/CONFIG
# Override the Base definition:
INSTALL_LOCATION = $(TOP)
# CONFIG_SITE files contain other build configuration settings
include $(TOP)/configure/CONFIG_SITE
-include $(TOP)/configure/CONFIG_SITE.$(EPICS_HOST_ARCH).Common
ifdef T_A
 -include $(TOP)/configure/CONFIG_SITE.Common.$(T_A)
 -include $(TOP)/configure/CONFIG_SITE.$(EPICS_HOST_ARCH).$(T_A)
endif
EOF

      cat >CONFIG_SITE <<\EOF &&
#Generated by install-epics.sh
CHECK_RELEASE = YES
EOF

      cat >Makefile <<\EOF &&
#Generated by install-epics.sh
TOP=..
include $(TOP)/configure/CONFIG
TARGETS = $(CONFIG_TARGETS)
CONFIGS += $(subst ../,,$(wildcard $(CONFIG_INSTALLS)))
include $(TOP)/configure/RULES
EOF

      cat >RELEASE <<\EOF &&
#Generated by install-epics.sh
TEMPLATE_TOP=$(EPICS_BASE)/templates/makeBaseApp/top
ASYN=${EPICS_ROOT}/modules/asyn
EPICS_BASE=${EPICS_ROOT}/base
EOF

      cat >RULES <<\EOF &&
#Generated by install-epics.sh
# RULES
include $(CONFIG)/RULES
# Library should be rebuilt because LIBOBJS may have changed.
$(LIBNAME): ../Makefile
EOF

      cat >RULES.ioc <<\EOF &&
#Generated by install-epics.sh
#RULES.ioc
include $(CONFIG)/RULES.ioc
EOF

      cat >RULES_DIRS <<\EOF &&
#Generated by install-epics.sh
#RULES_DIRS
include $(CONFIG)/RULES_DIRS
EOF

      cat >RULES_DIRS <<\EOF
#Generated by install-epics.sh
#RULES_TOP
include $(CONFIG)/RULES_TOP
EOF
    )
  )
fi


if test -n "$ASYN_VER_X_Y"; then
(
  create_soft_x_y $EPICS_ROOT/modules ../$ASYN_VER_X_Y/ asyn
    (
      #Note1: asyn should be under modules/
      #Note2: if "ASYNVER" = GIT
      cd $EPICS_ROOT &&
      if test "$ASYNVER" = GIT; then
        if ! test -d $ASYN_VER_X_Y; then
					(
						$FSUDO git clone https://github.com/epics-modules/asyn.git $ASYN_VER_X_Y
						cd $ASYN_VER_X_Y &&
						$FSUDO git checkout $ASYN_GIT_VER
					) ||
             ( $RM -rf $ASYN_VER_X_Y; false )
        fi
      else
        if ! test -f $ASYN_VER_X_Y.tar.gz; then
          wget_or_curl http://www.aps.anl.gov/epics/download/modules/$ASYN_VER_X_Y.tar.gz $ASYN_VER_X_Y.tar.gz
        fi
        if ! test -d $ASYN_VER_X_Y; then
          tar xzvf $ASYN_VER_X_Y.tar.gz
        fi
      fi
    ) &&
    (
      # Need to fix epics base for synapss already here,
      # (if the dir already exists)
      path=$EPICS_ROOT/$SYNAPPS_VER_X_Y/support/configure &&
      if test -d $path; then
        echo cd $path &&
        cd $path &&
        (
          fix_epics_base EPICS_BASE.$EPICS_HOST_ARCH &&
          fix_epics_base SUPPORT.$EPICS_HOST_ARCH
        )
      fi
    ) &&
    (
      cd $EPICS_ROOT/$ASYN_VER_X_Y/configure && {
        for f in $(find . -name "RELEASE*" ); do
          echo f=$f
          fix_epics_base $f
        done
      }
    ) &&
    (
      run_make_in_dir $EPICS_ROOT/$ASYN_VER_X_Y
    ) || {
      echo >&2 failed $ASYN_VER_X_Y
      exit 1
    }
)
else
  echo no special ASYN_VER_X_Y defined
fi


#synApps
if test -n "$SYNAPPS_VER_X_Y"; then
  (
    echo XXX SYNAPPS_VER_X_Y defined
    cd $EPICS_ROOT &&
    if ! test -f $SYNAPPS_VER_X_Y.tar.gz; then
      wget_or_curl http://www.aps.anl.gov/bcda/synApps/tar/$SYNAPPS_VER_X_Y.tar.gz $SYNAPPS_VER_X_Y.tar.gz
    fi &&
    if ! test -d $SYNAPPS_VER_X_Y; then
      tar xzvf $SYNAPPS_VER_X_Y.tar.gz
    fi
  ) || {
    echo >&2 failed tar xzvf $SYNAPPS_VER_X_Y.tar.gz in $PWD
    exit 1
  } &&
  (
    if test -n "$STREAMDEVICEVER"; then
      cd $EPICS_ROOT/$SYNAPPS_VER_X_Y/support/stream* &&
      (
        #Move the directory out of its way
        if ! test -d streamdevice.original; then
          $MV streamdevice streamdevice.original || {
            echo >&2 can not $MV streamdevice streamdevice.original PWD=$PWD
            exit 1
          }
          #copy the later streamdevice
          $MKDIR streamdevice &&
          $CP -R ../../../StreamDevice-2-6/ streamdevice/  || {
            echo >&2 $CP -R ../../../StreamDevice-2-6/ streamdevice/PWD=$PWD
            exit 1
          }
        fi
      )
    fi
  ) &&
  (
    path=$EPICS_ROOT/$SYNAPPS_VER_X_Y/support &&
    echo cd $path &&
    cd $path &&
    for f in $(find . -name RELEASE); do
      fix_epics_base $f
    done &&
    (
      cd configure &&
      $MKDIR -p orig || {
      echo >&2 failed $MKDIR -p orig in $PWD
        exit 1
      }
      for f in EPICS_BASE.cygwin* EPICS_BASE.linux-* EPICS_BASE.win32-* EPICS_BASE.windows-* SUPPORT.cygwin-*  SUPPORT.linux-*  SUPPORT.win32-*  SUPPORT.windows-*; do
        if test -f "$f"; then
          $MV -v "$f" $PWD/orig/ || {
            echo >&2 failed $MV "$f" orig/ in $PWD
            exit 1
          }
        fi
      done
    ) &&
    (
      echo cd $EPICS_ROOT/$SYNAPPS_VER_X_Y/support/configure &&
      cd $EPICS_ROOT/$SYNAPPS_VER_X_Y/support/configure &&
      if test "$SYNAPPS_VER_X_Y" = synApps_5_6; then
      (
        path=$EPICS_ROOT/$SYNAPPS_VER_X_Y/support
        echo cd $path &&
        cd $path &&
        if ! test -f makereleaseok; then
          make release && touch makereleaseok || {
            echo >&2 failed make release in $PWD
            exit 1
          }
        else
          echo The file $PWD/makereleaseok exist, skipping make release
        fi
      )
      fi &&
      fix_epics_base $PWD/RELEASE &&
      remove_modules_from_RELEASE RELEASE &&
      (
        echo cd $EPICS_ROOT/$SYNAPPS_VER_X_Y/support &&
        cd $EPICS_ROOT/$SYNAPPS_VER_X_Y/support &&
        file=Makefile &&
        if ! test -f $file.original; then
          $CP -v $file $file.original || exit 1
        fi &&
        $CP $file.original $file &&
        remove_modules_from_Makefile $file
      ) &&
      file=Makefile &&
      (
        # Remove AREA_DETECTOR and IP from RELEASE
        cd $EPICS_ROOT/$SYNAPPS_VER_X_Y/support/xxx-5*/configure &&
        if ! test -f RELEASE.original; then
          $CP -v RELEASE RELEASE.original || exit 1
        fi &&
        sed <RELEASE.original >RELEASE.$$.tmp \
          -e "s!^AREA_DETECTOR!#AREA_DETECTOR!" \
          -e "s!^IP=!#IP=!" \
          -e "s!^SNCSEQ!#SNCSEQ!" &&
        $MV -fv RELEASE.$$.tmp RELEASE
      ) &&
      (
        # Remove AREA_DETECTOR and IP from dbd
        cd $EPICS_ROOT/$SYNAPPS_VER_X_Y/support/xxx-5*/xxxApp/src &&
        if test -f xxxCommonInclude.dbd; then
          if ! test -f xxxCommonInclude.dbd.original; then
            $CP -v xxxCommonInclude.dbd xxxCommonInclude.dbd.original || exit 1
          fi &&
          sed <xxxCommonInclude.dbd.original >xxxCommonInclude.dbd.$$.tmp \
            -e "s!\(include.*ipSupport.dbd\)!#\1!" &&
          $MV -fv xxxCommonInclude.dbd.$$.tmp xxxCommonInclude.dbd
        fi
      ) &&
      (
        # Remove AREA_DETECTOR related modules from $file
        cd $EPICS_ROOT/$SYNAPPS_VER_X_Y/support/xxx-5*/xxxApp/src &&
        if ! test -f $file.original; then
          $CP -v $file $file.original || exit 1
        fi &&
        $CP $file.original $file &&
        for mod in $MODSFROMMAKEFILE; do
          echo removing $mod in $PWD/$file &&
          sed -e "s/\(.*$mod.*\)/#XXX Removed by install-epics.sh XXX  \1/g" <$file >$file.$$.tmp &&
          ! diff $file $file.$$.tmp >/dev/null &&
          $MV -f $PWD/$file.$$.tmp $PWD/$file ||{
            echo >&2 failed removing $mod in $PWD
            exit 1
          }
        done
      )
    ) || {
      echo >&2 failed in $PWD
      exit 1
    }
  ) || {
    echo >&2 failed RELEASE in $PWD
    exit 1
  } &&
  if test -z "$ASYN_VER_X_Y"; then
    (
      echo ASYN_VER_X_Y not defined, use asyn from synapps
      install_asyn_ver ../$SYNAPPS_VER_X_Y/support/asyn-4-*
    )
  fi
else
  echo XXX SYNAPPS_VER_X_Y not defined, skipping synApps
fi

if test "$EPICS_EXTENSIONS_TOP_VER"; then
  (
    cd $EPICS_ROOT &&
    if ! test -f $EPICS_EXTENSIONS_TOP_VER.tar.gz; then
      echo installing $EPICS_EXTENSIONS_TOP_VER &&
      wget_or_curl http://www.aps.anl.gov/epics/download/extensions/$EPICS_EXTENSIONS_TOP_VER.tar.gz  $EPICS_EXTENSIONS_TOP_VER.tar.gz
    fi
    if ! test -d ${EPICS_EXTENSIONS_TOP_VER}; then
      tar xzf $EPICS_EXTENSIONS_TOP_VER.tar.gz
    fi
  )
fi &&
if ! type re2c >/dev/null 2>/dev/null; then
  echo $APTGET re2c
  $APTGET re2c || install_re2c
fi &&
if test "$EPICS_MSI_VER"; then
  (
      cd $EPICS_ROOT &&
      if ! test -f $EPICS_MSI_VER.tar.gz; then
        echo installing $EPICS_MSI_VER &&
        wget_or_curl http://www.aps.anl.gov/epics/download/extensions/$EPICS_MSI_VER.tar.gz $EPICS_MSI_VER.tar.gz
      fi &&
      (
        $MKDIR -p extensions/src &&
        cd extensions/src &&
        tar xzf ../../$EPICS_MSI_VER.tar.gz &&
        cd $EPICS_MSI_VER &&
        run_make_in_dir . || {
          echo >&2 make failed in $PWD
          exit 1
        }
      ) || {
        echo >&2 msi failed in $PWD
        exit 1
      }
  )
fi &&
if test -z "$ASYN_VER_X_Y"; then
  #Need to compile asyn from synapps
  run_make_in_dir $EPICS_ROOT/$SYNAPPS_VER_X_Y/support/asyn-*/asyn
fi &&
if test -n "$SYNAPPS_VER_X_Y"; then
  run_make_in_dir $EPICS_ROOT/$SYNAPPS_VER_X_Y/support/sscan* &&
  run_make_in_dir $EPICS_ROOT/$SYNAPPS_VER_X_Y/support/calc-* &&
  run_make_in_dir $EPICS_ROOT/$SYNAPPS_VER_X_Y/support/stream-* || {
    echo >&2 failed $SYNAPPS_VER_X_Y PWD=$PWD PATH=$PATH
    exit 1
  }
fi

if test -z "$MOTORVER"; then
  patch_motor_h $EPICS_ROOT/$SYNAPPS_VER_X_Y/support/motor-*/motorApp/MotorSrc &&
  comment_out_in_file $EPICS_ROOT/$SYNAPPS_VER_X_Y/support/motor-*/motorApp/Makefile HytecSrc AerotechSrc &&
  run_make_in_dir $EPICS_ROOT/$SYNAPPS_VER_X_Y/support/motor-*/motorApp || {
    echo >&2 failed $SYNAPPS_VER_X_Y PWD=$PWD PATH=$PATH
    exit 1
  }
  install_motor_from_synapps
else
  install_motor_X_Y
fi &&
install_streamdevice &&
echo install $EPICS_ROOT OK || {
  echo >&2 failed install_streamdevice PWD=$PWD
  exit 1
}

