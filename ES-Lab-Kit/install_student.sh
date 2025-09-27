#!/bin/bash

################################################################################
# Script to setup ES-Lab-Kit environment for Linux Computer Rooms
#
# Author: Matthias Becker
# Date:   28. August 2025
# Note:   This script is specific for the installation on KTH Linux
#         Computer Rooms. Installation on private computers do not need to
#         following these steps, see ES-Lab-Kit and Raspberry Pi Pico
#         documentation if you use your own computer.
################################################################################

set -e

# Path definitions used for the installation
CMD_PATH=~/.local/bin
SDK_PATH=~/.pico-sdk
ES_LAB_KIT_Path=~/Documents/ES-Lab-Kit/

printf "==========================================================================\n"
printf "Installing UV and Python 3.12\n"
printf "==========================================================================\n"

if command -v uv >&2;
then
  printf "UV already installed, skipping UV installation\n"
else
  curl -LsSf https://astral.sh/uv/install.sh | sh

  # Amend $PATH to make uv work if installed in this installation
  case ":${PATH}:" in
      *:"$HOME/.local/share/../bin":*)
          ;;
      *)
          # Prepending path in case a system-installed binary needs to be overridden
          export PATH="$HOME/.local/share/../bin:$PATH"
          ;;
  esac

fi

if [[ $(python3 --version) == "Python 3.12.11" ]]
then
  printf "Python 3.12.11 already installed, skipping Python3 installation\n"
else
  printf "Installing Python 3.12\n"
  uv python install --default --preview 3.12
fi

printf "==========================================================================\n"
printf "Setting simlinks for nm-multiarch and objdump-multiarch\n"
printf "==========================================================================\n"
if [[ $(which objdump-multiarch) == -1 ]]
then
  cd ${CMD_PATH}
  ln -s ${CMD_PATH}/objdump objdump-multiarch
else
  printf "objdump-multiarch already exists, skipping objdump-multiarch simlink\n"
fi

if [[ $(which nm-multiarch) == -1 ]]
then
  cd ${CMD_PATH}
  ln -s ${CMD_PATH}/nm nm-multiarch
else
  printf "nm-multiarch already exists, skipping nm-multiarch simlink\n"
fi

printf "\n==========================================================================\n"
printf "Cloning ES-Lab-Kit\n"
printf "==========================================================================\n"

if [[ ! -d ${ES_LAB_KIT_Path} ]]
then
  git clone --depth 1 --recurse-submodules --shallow-submodules git@gits-15.sys.kth.se:mabecker/ES-Lab-Kit.git ${ES_LAB_KIT_Path}
  cd ${ES_LAB_KIT_Path}
  git submodule update --init
else
  printf "${ES_LAB_KIT_Path} already exists, skipping ES-Lab-Kit\n"
fi

printf "\n==========================================================================\n"
printf "Cloning Pico SDK 2.2.0\n"
printf "==========================================================================\n"
if [[ ! -d ${SDK_PATH}/sdk/2.2.0 ]]
then
  git -c advice.detachedHead=false clone --depth 1 --recurse-submodules --shallow-submodules --branch 2.2.0 https://github.com/raspberrypi/pico-sdk.git ${SDK_PATH}/sdk/2.2.0
  #cd ${SDK_PATH}sdk/2.2.0
  #git submodule update --init
else
  printf "${SDK_PATH}/sdk/2.2.0 already exists, skipping Pico SDK 2.2.0\n"
fi

printf "\n==========================================================================\n"
printf "Cloning Pico Examples\n"
printf "==========================================================================\n"
if [[ ! -d ${SDK_PATH}/examples ]]
then
  git -c advice.detachedHead=false clone --depth 1 --recurse-submodules --shallow-submodules --branch master https://github.com/raspberrypi/pico-examples.git ${SDK_PATH}/examples
else
  printf "${SDK_PATH}/examples already exists, skipping Pico Examples\n"
fi

# The remaining things are downloaded as archive and extracted to .pico-sdk
# For this, a tmp folder is created and later deleted. To save space each archive is deleted directly after it was extracted.
if [[ ! -d ~/tmp ]]
then
  mkdir ~/tmp
fi
cd ~/tmp

printf "\n==========================================================================\n"
printf "Downloading CMake 3.31.5\n"
printf "==========================================================================\n"
if [[ ! -d ${SDK_PATH}/examples ]]
then
  wget https://github.com/Kitware/CMake/releases/download/v3.31.5/cmake-3.31.5-linux-x86_64.tar.gz
  mkdir -p ${SDK_PATH}/cmake/v3.31.5
  tar -xzf cmake-3.31.5-linux-x86_64.tar.gz --strip-components 1 -C ${SDK_PATH}/cmake/v3.31.5
  rm cmake-3.31.5-linux-x86_64.tar.gz
else
  printf "${SDK_PATH}/cmake/v3.31.5 already exists, skipping CMake 3.31.5\n"
fi

printf "\n==========================================================================\n"
printf "Downloading Ninja 1.12.1\n"
printf "==========================================================================\n"
if [[ ! -d ${SDK_PATH}/ninja/v1.12.1 ]]
then
  wget https://github.com/ninja-build/ninja/releases/download/v1.12.1/ninja-linux.zip
  mkdir -p ${SDK_PATH}/ninja/v1.12.1
  unzip ninja-linux.zip
  mv ninja ${SDK_PATH}/ninja/v1.12.1  # Needed to be placed at the correct location!
  rm ninja-linux.zip
else
  printf "${SDK_PATH}/ninja/v1.12.1 already exists, skipping Ninja 1.12.1\n"
fi

printf "\n==========================================================================\n"
printf "Downloading OpenOCD 0.12.0-dev\n"
printf "==========================================================================\n"
if [[ ! -d ${SDK_PATH}/openocd/0.12.0+dev ]]
then
  #wget https://github.com/raspberrypi/pico-sdk-tools/releases/download/v1.5.1-0/openocd-0.12.0%2Bdev-x86_64-lin.tar.gz
  #mkdir -p ${SDK_PATH}openocd/0.12.0+dev
  #tar -xzf openocd-0.12.0+dev-x86_64-lin.tar.gz -C ${SDK_PATH}openocd/0.12.0+dev
  #rm openocd-0.12.0+dev-x86_64-lin.tar.gz

  # This is a precompiler version for the libraries installed on the KTH lab computers. The binary distributed by Raspberry Pi can be found above.
  wget --no-check-certificate "https://kth-my.sharepoint.com/:u:/g/personal/mabecker_ug_kth_se/EWGCfwIzFptKm-iztXQ4FZUBG4jwyN8uBon4PQkVtOi4CA?e=0chMcH&download=1" -O openocd-0.12.0+dev_LinuxLabRooms.tar.gz
  mkdir -p ${SDK_PATH}/openocd/0.12.0+dev
  tar -xzf openocd-0.12.0+dev_LinuxLabRooms.tar.gz --strip-components 1 -C ${SDK_PATH}/openocd/0.12.0+dev
  rm openocd-0.12.0+dev_LinuxLabRooms.tar.gz

  # Make the file executable
  cd ${SDK_PATH}/openocd/0.12.0+dev
  chmod +x openocd.exe
  cd ~/tmp

else
  printf "${SDK_PATH}/openocd/0.12.0+dev already exists, skipping OpenOCD 0.12.0-dev\n"
fi

printf "\n==========================================================================\n"
printf "Downloading Picotool 2.2.0\n"
printf "==========================================================================\n"
if [[ ! -d ${SDK_PATH}/picotool/2.2.0 ]]
then
  wget https://github.com/raspberrypi/pico-sdk-tools/releases/download/v2.2.0-0/picotool-2.2.0-x86_64-lin.tar.gz
  mkdir -p ${SDK_PATH}/picotool/2.2.0
  tar -xzf picotool-2.2.0-x86_64-lin.tar.gz --strip-components 1 -C ${SDK_PATH}/picotool/2.2.0
  rm picotool-2.2.0-x86_64-lin.tar.gz
else
  printf "${SDK_PATH}/picotool/2.2.0 already exists, skipping Picotool 2.2.0\n"
fi

printf "\n==========================================================================\n"
printf "Downloading Pico SDK Tools 2.2.0\n"
printf "==========================================================================\n"
if [[ ! -d ${SDK_PATH}/tools/2.2.0 ]]
then
  wget https://github.com/raspberrypi/pico-sdk-tools/releases/download/v2.2.0-0/pico-sdk-tools-2.2.0-x86_64-lin.tar.gz
  mkdir -p ${SDK_PATH}/tools/2.2.0
  tar -xzf pico-sdk-tools-2.2.0-x86_64-lin.tar.gz -C ${SDK_PATH}/tools/2.2.0
  rm pico-sdk-tools-2.2.0-x86_64-lin.tar.gz
else
  printf "${SDK_PATH}/tools/2.2.0 already exists, skipping SDK Tools 2.2.0\n"
fi

printf "\n==========================================================================\n"
printf "Downloading ARM GNU Toolchain 14.2 Rel1\n"
printf "==========================================================================\n"
if [[ ! -d ${SDK_PATH}/toolchain/14_2_Rel1 ]]
then
  wget --no-check-certificate "https://kth-my.sharepoint.com/:u:/g/personal/mabecker_ug_kth_se/EWITZOMTYeNIp61vi2mzucQB1lORWy6j42IQpl94Awyeng?e=fLvTJ6&download=1" -O arm-gnu-toolchain-14.2.rel1-x86_64-arm-none-eabi.tar.gz
  mkdir -p ${SDK_PATH}/toolchain/14_2_Rel1
  tar -xzf arm-gnu-toolchain-14.2.rel1-x86_64-arm-none-eabi.tar.gz -C ${SDK_PATH}/toolchain/14_2_Rel1
  rm arm-gnu-toolchain-14.2.rel1-x86_64-arm-none-eabi.tar.gz
else
  printf "${SDK_PATH}/toolchain/14_2_Rel1 already exists, skipping ARM GNU Toolchain 14.2 Rel1\n"
fi

cd ~
rm -rf ~/tmp

printf "\n==========================================================================\n"
printf "Downloading Debugprobe Firmware v2.2.3\n"
printf "==========================================================================\n"
if [[ ! -d ${SDK_PATH}/debugprobe ]]
then
  mkdir -p ${SDK_PATH}/debugprobe/v2.2.3
  cd ${SDK_PATH}/debugprobe/v2.2.3
  wget https://github.com/raspberrypi/debugprobe/releases/download/debugprobe-v2.2.3/debugprobe_on_pico2.uf2
  cd ~
else
  printf "${SDK_PATH}/debugprobe/v2.2.3 already exists, skipping Debugprobe Firmware v2.2.3\n"
fi

printf "\n==========================================================================\n"
printf "Installing Raspberry Pi Pico Extension in VSCode\n"
printf "==========================================================================\n"
code --install-extension raspberry-pi.raspberry-pi-pico@0.18.0

printf "\n==========================================================================\n"
printf "Setting up ProjectCreator\n"
printf "==========================================================================\n"
# Creates a virtual environment for the Python project and installs the dependencies.
# Since AFS does not support hard-links, the pip option —link-mode=copy is needed to avoid warnings.
cd ${ES_LAB_KIT_Path}/Tools/ProjectCreator
uv venv --clear
source .venv/bin/activate # Activate the Python virtual environment
uv pip install -r requirements.txt --link-mode=copy
deactivate  # Leave the Python virtial environment again

if [ ! -e ${CMD_PATH}/newembproj ]
then
  # Make  the wrapper file executable and move it so it can be called from anywhere.
  cd ${ES_LAB_KIT_Path}/Tools/scripts
  chmod +x newembproj
  cp newembproj ${CMD_PATH}
fi

printf "\n==========================================================================\n"
printf "Summary:\n"
printf "==========================================================================\n"
printf "$(uv --version) -> $(which uv)\n"
printf "$(python3 --version) -> $(which python3)\n"
printf "ES-Lab-Kit -> ${ES_LAB_KIT_Path}\n"
printf "Pico SDK 2.2.0 -> ${SDK_PATH}/sdk/2.2.0\n"
printf "Pico Examples -> ${SDK_PATH}/examples\n"
printf "CMake 3.31.5 -> ${SDK_PATH}/cmake/v3.31.5\n"
printf "Ninja 1.12.1 -> ${SDK_PATH}/ninja/v1.12.1\n"
printf "OpenOCD 0.12.0-dev -> ${SDK_PATH}/openocd/0.12.0+dev\n"
printf "Picotool 2.2.0 -> ${SDK_PATH}/picotool/2.2.0\n"
printf "Pico SDK Tools 2.2.0 -> ${SDK_PATH}/tools/2.2.0\n"
printf "ARM GNU Toolchain 14.2 Rel1 -> ${SDK_PATH}/toolchain/14_2_Rel1\n"
printf "Debugprobe Firmware v2.2.3 -> ${SDK_PATH}/debugprobe/v2.2.3/debugprobe_on_pico2.uf2"
printf "newembproj -> ${CMD_PATH}\n"
printf "objdump-multiarch -> ${CMD_PATH}\n"
printf "nm-multiarch -> ${CMD_PATH}\n"
