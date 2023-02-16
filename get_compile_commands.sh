#!/bin/bash

# Some constants
TASK=generateCompileCommands
BUILD_FILE=./build/compile_commands/linuxx86-64/compile_commands.json
FILE=./compile_commands.json

# Log some info
script_log(){
    echo "[get_compile_commands INFO]: $1"
}

# Log an error
script_log_err(){
    echo "[get_compile_commands ERROR]: $1"
}

# if no $JAVA_HOME, gradle can't run
if test -z "$JAVA_HOME"; then
    script_log_err "JAVA_HOME is empty (wpi environment variables probably aren't in env)"
    return 1
fi

# Check to see if gradle is executable, if not, don't execute
if test -x ./gradlew; then
    script_log "Executing gradle task: $TASK"
    ./gradlew $TASK
else
  script_log_err "./gradlew isn't executable, run chmod +x ./gradlew and re-run"
  return 1
fi

# Remove symlink file if present in project root dir
if test -f "$FILE"; then
    rm $FILE
    script_log "Removed file: $FILE"
fi

# Create symlink from profect root to build
script_log "Creating symlink to $BUILD_FILE from $FILE"
ln -s $BUILD_FILE $FILE

#Unset all variables just in case
unset TASK
unset BUILD_FILE
unset FILE

return 0
