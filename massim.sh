#!/bin/bash


EXECUTION_DIRECTORY="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

cd "$EXECUTION_DIRECTORY/third-party/massim/massim-2018-1.1/server"

#IT IS NECESSARY TO RUN ONCE BEFORE "mvn install" IN THE ROOT PROJECT OF MASSIM

java -jar server-2018-1.1-jar-with-dependencies.jar --monitor

#Monitor available at http://localhost:8000/
