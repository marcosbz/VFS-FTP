/* Copyright 2014, ACSE & CADIEEL
 *    ACSE   : http://www.sase.com.ar/asociacion-civil-sistemas-embebidos/ciaa/
 *    CADIEEL: http://www.cadieel.org.ar
 * All rights reserved.
 *
 *    or
 *
 * Copyright 2014, Your Name <youremail@domain.com>
 * All rights reserved.
 *
 *    or
 *
 * Copyright 2014, ACSE & CADIEEL & Your Name <youremail@domain.com
 *    ACSE   : http://www.sase.com.ar/asociacion-civil-sistemas-embebidos/ciaa/
 *    CADIEEL: http://www.cadieel.org.ar
 * All rights reserved.
 *
 * This file is part of CIAA Firmware.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef RAMDISK_H
#define RAMDISK_H
/** \brief Short description of this file
 **
 ** Long description of this file
 **
 **/

/** \addtogroup CIAA_Firmware CIAA Firmware
 ** @{ */
/** \addtogroup Template Template to start a new module
 ** @{ */

/*==================[inclusions]=============================================*/
#include <stdint.h>
#include "device.h"
#include "ooc.h"
#include "blockDevice.h"

/*==================[cplusplus]==============================================*/
#ifdef __cplusplus
extern "C" {
#endif

/*==================[macros]=================================================*/
/*==================[typedef]================================================*/
/** \brief Describe RAM disk current state */
typedef enum
{
   RAMDISK_STATUS_UNINIT,
   RAMDISK_STATUS_READY,
   RAMDISK_STATUS_BUSY
} ramdisk_status_t;

typedef struct
{

} RAMDisk_constructor_params_t;

/*==================[external data declaration]==============================*/
/** \brief RAMDisk class declaration. RAMDisk inherits from Device.
 **
 **/
DeclareClass(RAMDisk, Device);

/*==================[external functions declaration]=========================*/
/** \brief Wrapper to class constructor with parameters. Use this function for object instantiation.
 **
 ** \return    The newly created Object
 **/
RAMDisk RAMDisk_new(void);

/** \brief RAM device initialization sequence
 **
 **
 ** \param[in] self RAM device handle
 ** \param[in] data pointer to first ram address
 ** \param[in] numSectors number of sectors in ramdisk
 ** \return    0 success, else error
 **/
int RAMDisk_init(RAMDisk self, void *data, uint32_t numSectors);

/** \brief get RAM device status
 **
 **
 ** \param[in] self RAM device handle
 ** \return    Ready: Ready to operate, Busy: Must wait for new operation.
 **/
ramdisk_status_t RAMDisk_getStatus(RAMDisk self);

/** \brief Check if RAM device present
 **
 ** \param[in] RAMDisk RAM device handle
 ** \return    0 if not present, not 0 otherwise
 **/
int RAMDisk_isInserted(RAMDisk self);

/** \brief Read single block
 **
 ** \param[in] RAMDisk RAM device handler
 ** \param[out] readBlock buffer in which to read block
 ** \param[in] sector Block offset
 ** \return    -1 for timeout and other errors, 0 if success
 **/
int RAMDisk_singleBlockRead(RAMDisk self, uint8_t *readBlock, uint32_t sector);

/** \brief Write single block
 **
 ** \param[in] RAMDisk RAM device handler
 ** \param[in] writeBlock buffer from which to write block
 ** \param[in] sector sector offset
 ** \return    -1 for timeout and other errors, 0 if success
 **/
int RAMDisk_singleBlockWrite(RAMDisk self, const uint8_t *writeBlock, uint32_t sector);

/** \brief get RAM disk size in sectors
 **
 ** \param[in] RAMDisk RAM device handler
 ** \return    number of sectors in card
 **/
uint32_t RAMDisk_getSize(RAMDisk self);


/** \brief erase multiple adjacent sectors
 **
 ** \param[in] RAMDisk RAM device handler
 ** \param[in] start sector from where to start erasing
 ** \param[in] end last sector to erase
 ** \return    -1 for timeout and other errors, 0 if success
 **/
int RAMDisk_blockErase(RAMDisk self, uint32_t start, uint32_t end);

//int RAMDisk_multipleBlockRead(RAMDisk self, uint8_t *readBuffer, uint32_t sector, uint32_t readCount);
//int RAMDisk_multipleBlockWrite(RAMDisk self, uint8_t *writeBuffer, uint32_t sector, uint32_t writeCount);
/* Virtual function definitions */
Virtuals( RAMDisk, Device )
   Interface(BlockDevice);
EndOfVirtuals;

/*==================[cplusplus]==============================================*/
#ifdef __cplusplus
}
#endif
/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/
#endif  /* #ifndef RAMDISK_H */
