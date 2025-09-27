#!/bin/bash

################################################################################
# Script to uninstall tghe ES-Lab-Kit environment for Linux Computer Rooms
#
# Author: Matthias Becker
# Date:   28. August 2025
# Note:   This script is specific for the installation on KTH Linux
#         Computer Rooms. Installation on private computers do not need to
#         following these steps, see ES-Lab-Kit and Raspberry Pi Pico
#         documentation if you use your own computer.
################################################################################

# Path definitions used for the installation
CMD_PATH=~/.local/bin
SDK_PATH=~/.pico-sdk
ES_LAB_KIT_Path=~/Documents/ES-Lab-Kit

# Remove the wrapper scripts we created to call the Python projects easily
if [ -e ${CMD_PATH}/newembproj ]
then
  printf "Delete ${CMD_PATH}/newembproj\n"
  rm ${CMD_PATH}/newembproj
fi

if [ -e ${CMD_PATH}/rttrace ]
then
  printf "Delete ${CMD_PATH}/rttrace\n"
  rm ${CMD_PATH}/rttrace
fi

# Remove the Pico SDK folder
if [[ -d $SDK_PATH ]]
then
  printf "Delete folder $SDK_PATH\n"
  rm -rf $SDK_PATH
fi

# Remove the Pico VSCode extensions and all extensions it installs
# See a list of all extensions here: https://github.com/raspberrypi/pico-vscode/blob/main/scripts/Pico.code-profile
yExpr=$(locale yesexpr)
printf 'Delete the Raspberry Pi Pico VSCode extension (y/n)?\n '
old_stty_cfg=$(stty -g)
stty raw -echo ; answer=$(head -c 1) ; stty $old_stty_cfg # Careful playing with stty
if [ "$answer" != "${answer#${yExpr#^}}" ];then
  code --uninstall-extension raspberry-pi.raspberry-pi-pico
else
    printf "Raspberry Pi Pico VSCode extension not deleted!\n"
fi


printf 'Delete all extensions installed by the Raspberry Pi Pico extension (y/n)?\n'
old_stty_cfg=$(stty -g)
stty raw -echo ; answer=$(head -c 1) ; stty $old_stty_cfg # Careful playing with stty
if [ "$answer" != "${answer#${yExpr#^}}" ];then
  code --uninstall-extension ms-vscode.cpptools
  code --uninstall-extension ms-vscode.cpptools-extension-pack
  #code --uninstall-extension ms-vscode.cpptools-themes
  #code --uninstall-extension ms-vscode.cmake-tools
  code --uninstall-extension ms-vscode.vscode-serial-monitor
  code --uninstall-extension marus25.cortex-debug
  code --uninstall-extension mcu-debug.rtos-views
  code --uninstall-extension mcu-debug.peripheral-viewer
  code --uninstall-extension mcu-debug.memory-view
  code --uninstall-extension mcu-debug.debug-tracker-vscode
else
    printf "Other extensions are not deleted!\n"
fi

# Ask if UV and Python should be deleted as well.
# See here for instructions: https://docs.astral.sh/uv/getting-started/installation/#shell-autocompletion
printf '\nDelete Python 3.12 and UV (y/n)?\n'
old_stty_cfg=$(stty -g)
stty raw -echo ; answer=$(head -c 1) ; stty $old_stty_cfg # Careful playing with stty
if [ "$answer" != "${answer#${yExpr#^}}" ];then
  #Remove all data that uv has stored
  printf "Clean UV cache\n"
  uv cache clean
  printf "Delete $(uv python dir)\n"
  rm -r "$(uv python dir)"
  printf "Delete $(uv tool dir)\n"
  rm -r "$(uv tool dir)"
  # Remove binaries. Hard coded paths as those are set by UV
  printf "Delete ~/.local/bin/uv\n"
  printf "Delete ~/.local/bin/uvx\n"
  rm ~/.local/bin/uv ~/.local/bin/uvx
else
    printf "UV and Python not deleted!\n"
fi

# Remove the script to start the project creator
if [ -e ${CMD_PATH}/newembproj ]
then
  printf "Delete ~/.local/bin/newembproj\n"
  rm ${CMD_PATH}/newembproj
fi

# Remove the simlinks for xx-multiarch we created
if [ -e ${CMD_PATH}/objdump-multiarch ]
then
  printf "Delete ${CMD_PATH}/objdump-multiarch\n"
  rm ${CMD_PATH}/objdump-multiarch
fi

if [ -e ${CMD_PATH}/nm-multiarch ]
then
  printf "Delete ${CMD_PATH}/nm-multiarch\n"
  rm ${CMD_PATH}/nm-multiarch
fi

# Lastly, remove the ES-Lab-Kit folder
if [[ -d $ES_LAB_KIT_Path ]]
then
  printf "Delete folder $ES_LAB_KIT_Path\n"
  rm -rf $ES_LAB_KIT_Path
fi
