#!/bin/bash

#  This file belongs to the RoQME toolchain.
#  Copyright (C) 2019  University of Extremadura, University of Málaga, Biometric Vox.
#
#  RoQME is free software; you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation; either version 3 of the License, or
#  any later version.
#
#  RoQME is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#
#  GNU GPLv3 link: http://www.gnu.org/licenses/gpl-3.0.html


function printHelp {
  echo "\n------------------------------------------------------------------------"
  echo "RoQME Reasoner launcher"
  echo "It starts the module that estimates QoS metrics from observations"
  echo "\nOptions:"
  echo "   -period : [Optional] reasoner period in milliseconds (1s by default)"
  echo "   -gui    : [Optional] it indicates whether or not to use GUI"
  echo "   -nodds  : [Optional] it disables DDS communications (e.g., for testing purposes)"
  echo "   -model  : [Optional] Path to an alternative intermediate model description"
  echo "------------------------------------------------------------------------\n"
}


if [ "$1" == "help" ]
then
  printHelp
  exit 0
fi

ARGS=""
while [ "$1" != "" ]; do
    ARGS="${ARGS} ${1}"
    shift
done

#
# Checking everything is ok
#

if [ -z $OSPL_HOME ]
then
  echo "\nUndefined environment variable OSPL_HOME\n" 1>&2
  exit 1
fi

if [ ! -d $OSPL_HOME/lib ]
then
  echo "\nIt could not find OpenSplice lib directory\n" 1>&2
  exit 1
fi

if [ ! -f $OSPL_HOME/jar/dcpssaj5.jar ]
then
  echo "\nIt could not find dcpssaj5.jar\n" 1>&2
  exit 1
fi

#
# Finds the jar file
#
JAR=$(find . -type f -iname "roqme.reasoner-*.jar" | head -n 1)

if [ -z $JAR ]
then
  echo "\nIt could not find roqme.reasoner-*.jar\n" 1>&2
  exit 1
fi

#
# Jar execution
#

#IFS=";"

echo "\nExecuting " $JAR ${ARGS} "\n"



java -cp $JAR:$OSPL_HOME/jar/dcpssaj5.jar -Djava.library.path=$OSPL_HOME/lib roqme.reasoner.ReasonerLauncher ${ARGS}
