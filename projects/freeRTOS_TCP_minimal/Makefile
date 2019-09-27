# Copyright 2016, Pablo Ridolfi
# Copyright 2017, Eric Pernia
# All rights reserved.
#
# This file is part of Workspace.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from this
#    software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# application name
PROJECT_NAME := $(notdir $(PROJECT))

# Modules needed by the application
PROJECT_MODULES := modules/$(TARGET)/vfs \
                   modules/$(TARGET)/lpcusblib \
		   modules/$(TARGET)/sapi_rtos \
                   modules/$(TARGET)/freertos \
                   modules/$(TARGET)/emc \
                   modules/$(TARGET)/base \
                   modules/$(TARGET)/board \
                   modules/$(TARGET)/chip \
                   modules/$(TARGET)/ooc \
                   modules/$(TARGET)/tlsf

# source files folder
PROJECT_SRC_FOLDERS := $(PROJECT)/src \
                       $(PROJECT)/FreeRTOS-Plus-TCP \
                       $(PROJECT)/FreeRTOS-Plus-TCP/portable/BufferManagement \
                       $(PROJECT)/FreeRTOS-Plus-TCP/portable/NetworkInterface/Common \
                       $(PROJECT)/FreeRTOS-Plus-TCP/portable/NetworkInterface/LPC18xx \
                       $(PROJECT)/FreeRTOS-Plus-FAT \
                       $(PROJECT)/FreeRTOS-Plus-FAT/portable/lpc18xx \
                       $(PROJECT)/FreeRTOS-Plus-FAT/portable/common \
                       $(PROJECT)/FreeRTOS_Plus_FAT_Demos \
                       $(PROJECT)/FreeRTOS_Plus_FAT_Demos/test \
                       $(PROJECT)/FreeRTOS-Plus-TCP/protocols/Common \
                       $(PROJECT)/FreeRTOS-Plus-TCP/protocols/FTP \
                       $(PROJECT)/Common/Demo_IP_Protocols/Common \
                       $(PROJECT)/Common/Demo_IP_Protocols/FTP \
                       $(PROJECT)/Common/Demo_IP_Protocols/HTTP \
                       $(PROJECT)/Common/Demo_IP_Protocols/NTP \
                       $(PROJECT)/Common/FreeRTOS_Plus_FAT_SL_Demos/CreateExampleFiles \
                       $(PROJECT)/Common/Utilities \
                       $(PROJECT)/ff13c/src

# header files folder
PROJECT_INC_FOLDERS := $(PROJECT)/inc \
                       $(PROJECT)/FreeRTOS-Plus-TCP/include \
                       $(PROJECT)/FreeRTOS-Plus-TCP/portable/Compiler/GCC \
                       $(PROJECT)/FreeRTOS-Plus-TCP/portable/NetworkInterface/include \
                       $(PROJECT)/FreeRTOS-Plus-FAT/include \
                       $(PROJECT)/FreeRTOS-Plus-FAT/portable/common \
                       $(PROJECT)/FreeRTOS-Plus-TCP/protocols/include \
                       $(PROJECT)/Common/Demo_IP_Protocols/include \
                       $(PROJECT)/Common/Utilities/include \
                       $(PROJECT)/ff13c/inc

# source files
PROJECT_C_FILES := $(wildcard $(PROJECT)/src/*.c) \
                   $(wildcard $(PROJECT)/FreeRTOS-Plus-TCP/*.c) \
                   $(wildcard $(PROJECT)/FreeRTOS-Plus-TCP/portable/BufferManagement/*.c) \
                   $(wildcard $(PROJECT)/FreeRTOS-Plus-TCP/portable/NetworkInterface/Common/*.c) \
                   $(wildcard $(PROJECT)/FreeRTOS-Plus-TCP/portable/NetworkInterface/LPC18xx/*.c) \
                   $(wildcard $(PROJECT)/FreeRTOS-Plus-FAT/*.c) \
                   $(wildcard $(PROJECT)/FreeRTOS-Plus-FAT/portable/lpc18xx/*.c) \
                   $(wildcard $(PROJECT)/FreeRTOS-Plus-FAT/portable/common/*.c) \
                   $(wildcard $(PROJECT)/FreeRTOS_Plus_FAT_Demos/*.c) \
                   $(wildcard $(PROJECT)/FreeRTOS_Plus_FAT_Demos/test/*.c) \
                   $(wildcard $(PROJECT)/FreeRTOS-Plus-TCP/protocols/Common/*.c) \
                   $(wildcard $(PROJECT)/FreeRTOS-Plus-TCP/protocols/FTP/*.c) \
                   $(wildcard $(PROJECT)/Common/Demo_IP_Protocols/Common/*.c) \
                   $(wildcard $(PROJECT)/Common/Demo_IP_Protocols/FTP/*.c) \
                   $(wildcard $(PROJECT)/Common/Demo_IP_Protocols/HTTP/*.c) \
                   $(wildcard $(PROJECT)/Common/Demo_IP_Protocols/NTP/*.c) \
                   $(wildcard $(PROJECT)/Common/FreeRTOS_Plus_FAT_SL_Demos/CreateExampleFiles/*.c) \
                   $(wildcard $(PROJECT)/Common/Utilities/*.c) \
                   $(wildcard $(PROJECT)/ff13c/src/*.c)


PROJECT_ASM_FILES := $(wildcard $(PROJECT)/src/*.S)

# usb host symbol
SYMBOLS += -DUSB_HOST_ONLY -DLWIP_DEBUG
