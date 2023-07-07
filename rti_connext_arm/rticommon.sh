#!/bin/sh
#########################################################################
# (c) 2005-2014 Copyright, Real-Time Innovations. All rights reserved.  #
# No duplications, whole or partial, manual or electronic, may be made  #
# without express written permission.  Any such copies, or              #
# revisions thereof, must display this notice unaltered.                #
# This code contains trade secrets of Real-Time Innovations, Inc.       #
#########################################################################
#
#  NAME: rticommon.sh
#  ------------------------------------------------------------------------
#  rticommon.sh is a common interface for RTI Connext DDS
#  scripts. It sets up the environment to achieve transparent execution
#  of applications.
#
#  USAGE:
#  ------------------------------------------------------------------------
#  To call the script use the following commands in your batch script:
#
#   filename=$0
#   script_dir=`dirname "$filename"` [Required]
#   executable_name="rtiddsspy" [Optional]
#
#   and then "source" rticommon.sh like this:
#   . ../resource/scripts/rticommon.sh [Required]
#
#  Parameters:
#
#    Note that you before calling the script you will need to set the
#    following parameters first:
#       - script_dir              [Required]
#         Directory where the script lives, i.e., path/to/installation/bin.
#         It can be set by using the following command:
#         script_dir=`dirname "$filename"`
#
#       - script_version          [Optional]
#         Version of the application to be run. This parameter is used to
#         load the right version of the Core and API dynamic libraries by
#         setting the path to the right folder.
#         For instance:
#         LD_LIBRARY_PATH=$lib_dir/$platform/$script_version:$lib_dir/$platform
#
#       - needs_platform_name     [Optional]
#         Indicates wether the script requires platform_name to be set
#         to run the application. For instance, rtiddsspy requires
#         $platform_name to be set to complete the path to the executable,
#         whereas rtiddsgen does not need to find any path based on that
#         $platform_name.
#
#       - needs_shared_libraries  [Optional]
#         Indicates wether the script requires DDS's shared libraries to
#         run. For instance, rtiroutingservice is built dynamically against
#         libnddscore and libnddsc and therefore it needs these libraries
#         to be present in the system in order to run. Setting
#         needs_shared_libraries to true is only required in scripts that don't
#         define an executable_name, such as Java tools that load DDS libraries
#         dynamically.
#
#       - executable_name [Required only for C/C++ Applications]
#         For example: executable_name="rtirecorder"
#
#  Return variables:
#
#    The script will set in the environment of your script the following
#    variables if it runs successfully:
#
#       - bin_dir -- Directory where binary applications are installed
#         bin_dir ==
#           path/to/installation/resource/app/bin
#
#       - lib_dir -- Directory where RTI Connext DDS libraries are
#         installed.
#           path/to/installation/lib
#
#       - eclipse_dir -- Directory where libraries required by
#         eclipse are installed.
#         eclipse_dir ==
#           path/to/installation/resource/app/eclipse
#
#       - jre_dir -- Directory containing all the different JREs in
#         the installation.
#         jre_dir == path/to/installation/resource/app/jre
#
#       - doc_dir -- Documentation Directory. Documentation is sorted
#         by application name within this directory.
#         doc_dir ==
#           path/to/installation/doc
#
#       - app_lib_dir -- Directory where libraries required by
#         applications and services are installed.
#         app_lib_dir ==
#           path/to/installation/resource/app/lib
#
#       - lib_java_dir -- Directory where RTI Java libraries are
#         installed.
#         lib_java_dir ==
#           path/to/installation/lib/java
#
#       - app_lib_java_dir -- Directory where Java libraries required
#         by applications and services are installed.
#         app_lib_java_dir ==
#           path/to/installation/resource/app/lib/java
#
#       - resource_example_dir -- Directory containing all the
#         examples this directory will be copied to the user's home
#         directory the fisrt time any RTI tool or utility is run.
#         resource_example_dir ==
#           path/to/installation/resource/template/rti_workspace/example
#
#       - resource_user_config_dir -- Directory containing all the
#         default user's configuration files. This directory will be
#         copied to the user's home directory the first time any
#         RTI tool or utility is run.
#         resource_user_config_dir ==
#           path/to/installation/resource/template/rti_workspace/user_config_dir
#
#       - app_support_dir -- Directory where applications keep different
#         application-specific files (e.g., configuration files).
#         app_support_dir == path/to/installation/resource/app/app_support
#
#       - workspace_dir -- User directory to place RTI user-specific
#         configurations and documents. For instance:
#         /home/fgarcia/rti_workspace.
#
#       - home_example_dir -- Directory within $workspace_dir containing
#         examples. This directory is created by this script when run if
#         it does not previously exist. It is a copy of
#         $resource_example_dir.
#         resource_example_dir == $HOME/rti_workspace/example
#
#       - home_user_config_dir -- Directory within $workspace_dir containing
#         all the default user's configuration files. This directory is
#         created by this script when run if it does not previously exist.
#         It is a copy of $resource_user_config_dir.
#         home_user_config = $HOME/rti_workspace/user_config
#
#       - installation_path -- Top level directory of the installation (Might
#         be a duplicate of NDDSHOME for some cases)
#
#       - NDDSHOME -- Top level directory of the installation.
#
#       - RTI_LICENSE_FILE -- Points to the default location of the products
#         license file. This script will not override the RTI_LICENSE_FILE
#         environment variable if it is already set.
#
#       - JREHOME -- Directory where Java Runtime Environment for your
#         platform is installed. To run Java use $JREHOME\lib\java.
#         JREHOME == path/to/installation/resource/app/
#
#       - platform_name -- Name of the RTI Connext DDS architecture
#         installed in your system for your environment (e.g.,
#         i86Linux3.xgcc4.6.3).
#
#       - jre_platform -- Name of the RTI Connext DDS host architecture
#         for your environment (e.g., i86Linux).

if [ ! -x "$script_dir" ]
then
    echo "Warning: Could not find script_dir. Please contact support@rti.com."
    exit 1
fi

# Installation Information
##########################
host_version=6.1.1

# Common directories
####################
doc_dir=$script_dir/../doc
app_lib_dir=$script_dir/../resource/app/lib
app_lib_java_dir=$app_lib_dir/javacd
app_support_dir=$script_dir/../resource/app/app_support
eclipse_dir=$script_dir/../resource/app/eclipse
resource_example_dir=$script_dir/../resource/template/rti_workspace/examples
resource_user_config_dir=$script_dir/../resource/template/rti_workspace/user_config
shared_lib_extension=so

# Do not redefine lib_dir if it is already defined
if [ "x$lib_dir" = "x" ]
then
    if [ "x$run_within_module" = "xtrue" ]
    then
		# Running within the module
		lib_dir=$script_dir/../lib
		app_lib_dir=$lib_dir
    elif [ -d "$script_dir"/../../ndds.4.1 ]
    then
		# Running from ndds.4.1
		lib_dir=$script_dir/../lib
		app_lib_dir=$lib_dir
    else
		# Running from a shipped location
		lib_dir=$script_dir/../lib
		copy_workspace=true
    fi
fi

# Do not redefine bin_dir if it is already defined
if [ "x$bin_dir" = "x" ]
then
    bin_dir=$script_dir/../resource/app/bin
    if [ "x$executable_name" = "x" ] || [ ! -d "$bin_dir" ]
    then
        bin_dir=$app_lib_dir
    fi
fi

# Do not redefine jre_dir if it is already defined
if [ "x$jre_dir" = "x" ]
then
    jre_dir=$script_dir/../resource/app/jre
fi

# RTI Environment variables
###########################
NDDSHOME=`cd "$rticommon_script_dir/../.."; pwd`; export NDDSHOME
PATH="$NDDSHOME/bin":$PATH; export PATH

# Default workspace directory. It can be overriden in the following
# configuration files:
#
# - $HOME/.rti/rticommon_config.sh (for the current user)
# - $NDDSHOME/resource/scripts/rticommon_config.sh (for all users)
workspace_dir=$HOME/rti_workspace/$host_version

# By default we copy examples. Users can override that by editing the files
# listed above.
copy_examples=true

if [ -f "$HOME/.rti/rticommon_config.sh" ]
then
    # Override defaults if $HOME/.rti/rticommon_config.sh exists
    . "$HOME/.rti/rticommon_config.sh"
elif [ -f "$NDDSHOME/resource/scripts/rticommon_config.sh" ]
then
    # Override defaults if resource/scripts/rticommon_config.sh exists
    . "$NDDSHOME/resource/scripts/rticommon_config.sh"
fi

home_user_config_dir=$workspace_dir/user_config
home_example_dir=$workspace_dir/examples

# Set RTI_LICENSE_FILE to the license file in NDDSHOME if
# it was not previously set
if [ "x$RTI_LICENSE_FILE" = "x" ]
then
    # Workspace directory for this specific version
    if [ -f "$workspace_dir/rti_license.dat" ]
    then
	RTI_LICENSE_FILE="$workspace_dir/rti_license.dat"; export RTI_LICENSE_FILE
    # Workspace top level directory
    elif [ -f "$workspace_dir/../rti_license.dat" ]
    then
	RTI_LICENSE_FILE="$workspace_dir/../rti_license.dat"; export RTI_LICENSE_FILE
    # Top level installation directory
    else
	RTI_LICENSE_FILE="$NDDSHOME/rti_license.dat"; export RTI_LICENSE_FILE
    fi
fi

# Create Workspace Dir
######################
# Copy examples to workspace dir if they are not there already
if [ ! -x "$home_example_dir" ] && [ "x$copy_workspace" = "xtrue" ] && [ -d "$resource_example_dir" ] && [ "x$copy_examples" = "xtrue" ]
then
    echo "First time running RTI Connext DDS... "
    echo "Copying examples into $home_example_dir..."
    mkdir -p "$home_example_dir"
    cp -Rpf "$resource_example_dir" "$workspace_dir/"
fi


# Copy user configuration files to workspace dir if they do not exist. If the home_user_config_dir exists
# check that all the user_config directories have been copied and copy the ones that have not otherwise.
if [ ! -x "$home_user_config_dir" ] && [ "x$copy_workspace" = "xtrue" ] && [ -d "$resource_user_config_dir" ]
then
    echo "Copying user configuration files into $home_user_config_dir..."
    mkdir -p "$home_user_config_dir"
    cp -Rpf "$resource_user_config_dir" "$workspace_dir/"

elif [ "x$copy_workspace" = "xtrue" ] && [ -d "$resource_user_config_dir" ]
then
    user_config_dirnames=`ls "$resource_user_config_dir/"`
    for user_config_dirname in $user_config_dirnames
    do
	if [ ! -x "$home_user_config_dir/$user_config_dirname" ]
	then
	    echo "Copying missing $user_config_dirname user configuration files into $home_user_config_dir..."
	    cp -Rpf "$resource_user_config_dir/$user_config_dirname" "$home_user_config_dir/"
	fi
    done
fi


# Define Platforms to try
#########################

# Darwin Platforms
x64DarwinPlatforms=`ls -1 "$bin_dir/" | grep x64Darwin`
arm64DarwinPlatforms=`ls -1 "$bin_dir/" | grep arm64Darwin`
x64DarwinGenericPlatforms=`ls -1 "$bin_dir/" | grep x86_64Darwin`

# Linux Platforms
i86LinuxPlatforms=`ls -1 "$bin_dir/" | grep i86Linux`
x64LinuxPlatforms=`ls -1 "$bin_dir/" | grep x64Linux`
ppcLinux26Platforms=`ls -1 "$bin_dir/" | grep ppc | grep Linux2.6`
ppc32LinuxPlatforms=`ls -l "$bin_dir/" | grep ppc32 | grep Linux`
ppc64Linux26Platforms=`ls -1 "$bin_dir/" | grep ppc64Linux2.6`
ppcLinux3Platforms=`ls -1 "$bin_dir/" | grep ppc | grep Linux3`
power7Linux26Platforms=`ls -1 "$bin_dir/" | grep power7Linux2.6`
cell64Linux26Platforms=`ls -1 "$bin_dir/" | grep cell64Linux2.6`
armv7Linux26Platforms=`ls -1 "$bin_dir/" | grep armv7leLinux2.6`
armv7Linux3Platforms=`ls -1 "$bin_dir/" | grep armv7.*Linux3`
armv7LinuxPlatforms=`ls -1 "$bin_dir/" | grep armv7Linux`
armv6LinuxPlatforms=`ls -1 "$bin_dir/" | grep armv6.*Linux`
i86LinuxGenericPlatforms=`ls -1 "${bin_dir}/" | grep i686Linux`
x64LinuxGenericPlatforms=`ls -1 "${bin_dir}/" | grep x86_64Linux`
armv8LinuxPlatforms=`ls -1 "$bin_dir/" | grep armv8.*Linux`
armv8LinuxGenericPlatforms=`ls -1 "$bin_dir/" | grep aarch64Linux`
x64RedHawkPlatforms=`ls -1 "$bin_dir/" | grep x64RedHawk`

x64WRLinuxPlatforms=`ls -1 "$bin_dir/" | grep x64WRLinux`
armv7aWRLinuxPlatforms=`ls -1 "$bin_dir/" | grep armv7aWRLinux`

ppce6500LinuxPlatforms=`ls -1 "$bin_dir/" | grep ppce6500Linux`

# QNX Platforms
ppcQNX6Platforms=`ls -1 "$bin_dir/" | grep "ppc.*QNX6"`
i86QNX6Platforms=`ls -1 "$bin_dir/" | grep i86QNX6`
armv7aQNX6Platforms=`ls -1 "$bin_dir/" | grep armv7aQNX6`
armv7QNX7Platforms=`ls -1 "$bin_dir/" | grep armv7QNX7`
armv8QNX7Platforms=`ls -1 "$bin_dir/" | grep armv8QNX7`
x64QNX7Platforms=`ls -1 "$bin_dir/" | grep x64QNX7`

# AIX Platforms
p964AIXPlatforms=`ls -1 "$bin_dir/" | grep "^64p9AIX"`

platform_name="<arch>"
jre_platform="unknown"
jvm_lib="unknown"

os=`uname -s`
osver=`uname -r`

case $os in
    Darwin*)
	processor=`uname -p`
	jre_platform="darwin"
	jvm_lib="lib/server"
	shared_lib_extension=dylib
	executable_extension=app
	case $osver in
	    *)
        if test "processor" = "x86_64"; then
    		platforms_to_try="$x64DarwinPlatforms $x64DarwinGenericPlatforms"
        else
            platforms_to_try="$arm64DarwinPlatforms $x64DarwinPlatforms $x64DarwinGenericPlatforms"
        fi
		;;
	esac
	;;

    Linux*)
    processor=`uname -m`
    case $osver in
        *)
        if test "$processor" = "x86_64"; then
            platforms_to_try="$x64LinuxPlatforms $x64WRLinuxPlatforms $x64RedHawkPlatforms $x64LinuxGenericPlatforms"
            jre_platform="x64Linux"
            jvm_lib="lib/server"
        elif test "$processor" = "i686"; then
            platforms_to_try="$i86LinuxPlatforms i86LinuxGenericPlatforms"
            jre_platform="i86Linux"
            jvm_lib="lib/client"
        elif test "$processor" = "armv6l"; then
            platforms_to_try="$armv6LinuxPlatforms"
            jre_platform="armvfphLinux"
            jvm_lib="lib/client"
        elif test "$processor" = "armv7l"; then
            platforms_to_try="$armv7Linux3Platforms $armv7Linux26Platforms $armv6LinuxPlatforms $armv7aWRLinuxPlatforms $armv7LinuxPlatforms"
            jre_platform="armvfphLinux"
            jvm_lib="lib/client"
        elif test "$processor" = "aarch64"; then
            platforms_to_try="$armv8LinuxPlatforms $armv8LinuxGenericPlatforms"
            jre_platform="arm64vfphLinux"
            jvm_lib="lib/server"
        fi
        ;;
    esac
	;;

    QNX*)
	processor=`uname -p`
	case $osver in
	    6.*)
		if [ "$processor" = "ppcbe" ]; then
		    platforms_to_try=$ppcQNX6Platforms
		fi
		if [ "$processor" = "x86" ]; then
		    platforms_to_try=$i86QNX6Platforms
		fi
		if [ "$processor" = "armle" ]; then
		    platforms_to_try=$armv7aQNX6Platforms
		fi
		;;
	    7.*)
			if [ "$processor" = "armle" ]; then
				platforms_to_try=$armv7QNX7Platforms
			fi
			if [ "$processor" = "aarch64le" ]; then
				platforms_to_try=$armv8QNX7Platforms
			fi
			if [ "$processor" = "x86_64" ]; then
				platforms_to_try=$x64QNX7Platforms
			fi
		;;
	esac
	;;

    AIX*)
    jre_platform="aix"
    platforms_to_try=$p964AIXPlatforms
    ;;

    *)
	echo "Warning: OS $os may not be supported."
	;;
esac

if [ "x$CONNEXTDDS_ARCH" != "x" ]
then
    platforms_to_try="$CONNEXTDDS_ARCH $platforms_to_try"
	custom_architecture=$CONNEXTDDS_ARCH
elif [ "x$connextdds_architecture" != "x" ]
then
    platforms_to_try="$connextdds_architecture $platforms_to_try"
    custom_architecture=$connextdds_architecture
fi
for platform in $platforms_to_try
do
    if [ "x$executable_name" != "x" ]
    then
        # C/C++ Applications
        if [ -x "$bin_dir/$platform/$executable_name" ] || [ -x "$bin_dir/$platform/$executable_name.$executable_extension" ]
        then
            # By default all executables require shared libraries, unless
            # explicitly disabled.
            if [ "x$needs_shared_libraries" != "xfalse" ]
            then
                if [ -f "$app_lib_dir/$platform/libnddscore.$shared_lib_extension" ]
                then
                    # Executables that need to load shared libs
                    platform_name=$platform
                    break
                # Knowing that the target architectures won't include the directory
                # resource/app/lib/<arch>, we must also check if the core library
                # is in the lib directory. We know that the binary is in place,
                # so we won't add any architecture that doesn't include them.
                elif [ -f "$lib_dir/$platform/libnddscore.$shared_lib_extension" ]
                then
                    # Executables that need to load shared libs
                    platform_name=$platform
                    # In this case app_lib_dir becomes lib_dir
                    app_lib_dir=$lib_dir
                    break
                fi
            else
                platform_name=$platform
                break
            fi
		fi
	else
		# Java Applications
		if [ "x$needs_shared_libraries" = "xtrue" ]
		then
			if [ -f "$app_lib_dir/$platform/libnddscore.$shared_lib_extension" ]
			then
				# Java applications that need to load shared libraries
				platform_name=$platform
				break
			fi
		else
			# If we get here we can choose any platform_name--the application
			# is not likely to require it to run.
			platform_name=$platform
			break
		fi
    fi
done

# Check that platform_name is not empty for all the applications except
# for those platforms that do not have dependencies.
if [ "$platform_name" = "<arch>" ] && [ "x$needs_platform_name" != "xfalse" ]
then
    if [ "x$needs_shared_libraries" = "x" ] || [ "x$needs_shared_libraries" = "xfalse" ]
    then
		echo "Warning: Could not find the executable for $executable_name."
		echo "         Please examine $bin_dir/$platform_name to find the executable for $executable_name"
		echo "         and run this executable directly. If $executable_name is not in your bin directory"
		echo "         please contact support@rti.com."
    else
		echo "Warning: Could not find libraries for your platform"
		echo "         Please examine $app_lib_dir/<arch> and make sure you have the right"
		echo "         libraries and binaries installed. If they are not, please contact"
		echo "         support@rti.com."
    fi
    exit 1
fi

# Set JRE home
if [ "$jre_platform" != "unknown" ] && [ "x$JREHOME" = "x" ]
then
    JREHOME=$jre_dir/$jre_platform
    # In MacOSX we probably have JREHOME under $JREHOME/Contents/Home
    if [ -d "$JREHOME/Contents/Home" ]
    then
		JREHOME=$JREHOME/Contents/Home
    fi
	export JREHOME
fi

# Set jvm_lib pointing to the lib directory of the JRE
if [ "$jvm_lib" != "unknown" ]
then
    jvm_lib=$JREHOME/$jvm_lib
fi

# Set Library Path
LD_LIBRARY_PATH="$app_lib_dir/$platform_name/$script_version":"$app_lib_dir/$platform_name":"$jvm_lib":$RTI_LD_LIBRARY_PATH:$LD_LIBRARY_PATH; export LD_LIBRARY_PATH
DYLD_LIBRARY_PATH=$LD_LIBRARY_PATH:$DYLD_LIBRARY_PATH; export DYLD_LIBRARY_PATH

# Export other environment variables
RTI_SHARED_LIB_PREFIX=lib; export RTI_SHARED_LIB_PREFIX
RTI_SHARED_LIB_SUFFIX=.$shared_lib_extension; export RTI_SHARED_LIB_SUFFIX
RTI_EXAMPLES_DIR=$home_example_dir; export RTI_EXAMPLES_DIR
RTI_WORKSPACE_DIR=$workspace_dir; export RTI_WORKSPACE_DIR
