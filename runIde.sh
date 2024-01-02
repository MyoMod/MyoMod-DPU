#!/bin/bash

# Path of the IDE
IDE_PATH="/usr/local/mcuxpressoide"

# Path to GNU tools and compiler
TOOLCHAIN_PATH="${IDE_PATH}/ide/tools/bin"

# Path to workspace
WORKSPACE_PATH="/home/leon/Documents/MCUXpresso_11.8.1_1197/cliWorkspace/"
PROJECT_NAME="DPU"

# Variable to the command line Eclipse IDE executable
IDE="${IDE_PATH}/ide/mcuxpressoide"

echo "Launching Eclipse IDE"

# Build on project: normal build, does a (re-)build of all targets
#"${IDE}" -nosplash --launcher.suppressErrors -application org.eclipse.cdt.managedbuilder.core.headlessbuild -data "${WORKSPACE_PATH}" -build "${PROJECT_NAME}"

# Build on a build target: only builds the target
"${IDE}" -nosplash --launcher.suppressErrors -application org.eclipse.cdt.managedbuilder.core.headlessbuild -data "${WORKSPACE_PATH}" -build "${PROJECT_NAME}"/Debug

# Clean build on project: this does a 'clean' on each build target followed by a build
# "${IDE}" -nosplash --launcher.suppressErrors -application org.eclipse.cdt.managedbuilder.core.headlessbuild -data "${WORKSPACE_PATH}" -cleanBuild "${PROJECT_NAME}"

# Clean build on a build target: this does a 'clean' only on the build target, no build
# "${IDE}" -nosplash --launcher.suppressErrors -application org.eclipse.cdt.managedbuilder.core.headlessbuild -data "${WORKSPACE_PATH}" -cleanBuild "${PROJECT_NAME}"/Debug
