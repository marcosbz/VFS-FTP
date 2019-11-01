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

/** \brief RAMDisk class implementation file
 **
 ** Class of RAM storage device
 **
 **/

/** \addtogroup CIAA_Firmware CIAA Firmware
 ** @{ */
/** \addtogroup Template Template to start a new module
 ** @{ */

/*==================[inclusions]=============================================*/
#include <string.h>
#include "board.h"
#include "chip.h"
#include "RAMDisk.h"
#include "implement/RAMDisk_impl_lpc4337.h"
#include "device.h"
#include "implement/device_impl_lpc4337.h"
/*==================[macros and definitions]=================================*/
#define RAMDISK_SECTORSIZE 512

/*==================[internal data definition]===============================*/

/*==================[internal functions declaration]=========================*/
/* BlockDevice interface implementation */
//static ssize_t RAMDisk_read(RAMDisk self, uint8_t * const buf, size_t const nbyte); //Old version
//static ssize_t RAMDisk_write(RAMDisk self, uint8_t const * const buf, size_t const nbyte); //Old version
static ssize_t RAMDisk_read(RAMDisk self, uint8_t * const buf, uint32_t sector, size_t count);
static ssize_t RAMDisk_write(RAMDisk self, uint8_t const * const buf, uint32_t sector, size_t count);
//static ssize_t RAMDisk_lseek(RAMDisk self, off_t const offset, uint8_t const whence); //Old version
static int RAMDisk_ioctl(RAMDisk self, int32_t request, void* param);
static int RAMDisk_connect(RAMDisk self);
static int RAMDisk_disconnect(RAMDisk self);
static int RAMDisk_getState(RAMDisk self, blockDevState_t *state);
static int RAMDisk_getInfo(RAMDisk self, blockDevInfo_t *info);

/*==================[internal data definition]===============================*/
/*==================[external data definition]===============================*/
/** \brief Allocating the class description table and the vtable
 */
InterfaceRegister(RAMDisk)
{
	AddInterface(RAMDisk, BlockDevice)
};

AllocateClassWithInterface(RAMDisk, Device);

/** \brief Class virtual function prototypes
 */

/*==================[internal functions definition]==========================*/

/** \brief Class initializing
 */

static void RAMDisk_initialize( Class this )
{
   /* Init vtable and override/assign virtual functions */
   RAMDiskVtable vtab = & RAMDiskVtableInstance;

   //vtab->BlockDevice.read = (ssize_t (*)(Object, uint8_t * const, size_t const))RAMDisk_read; //Old version
   //vtab->BlockDevice.write = (ssize_t (*)(Object, uint8_t const * const buf, size_t const))RAMDisk_write; //Old version
   vtab->BlockDevice.read = (ssize_t (*)(Object, uint8_t * const buf, uint32_t sector, size_t count))RAMDisk_read;
   vtab->BlockDevice.write = (ssize_t (*)(Object, uint8_t const * const buf, uint32_t sector, size_t count))RAMDisk_write;
   //vtab->BlockDevice.lseek = (ssize_t (*)(Object, off_t const, uint8_t const))RAMDisk_lseek; //Old version
   vtab->BlockDevice.ioctl = (int (*)(Object, int32_t, void*))RAMDisk_ioctl;
   vtab->BlockDevice.connect = (int (*)(Object))RAMDisk_connect;
   vtab->BlockDevice.disconnect = (int (*)(Object))RAMDisk_disconnect;
   vtab->BlockDevice.getState = (int (*)(Object, blockDevState_t *))RAMDisk_getState;
   vtab->BlockDevice.getInfo = (int (*)(Object, blockDevInfo_t *))RAMDisk_getInfo;
   /* Allocate global resources here */
}

/** \brief Class finalizing
 */

#ifndef OOC_NO_FINALIZE
static void RAMDisk_finalize( Class this )
{
   /* Release global resources! */
}
#endif

/** \brief Constructor */
static void RAMDisk_constructor( RAMDisk self, const void *params )
{
   RAMDisk_constructor_params_t *ramdisk_params;
   assert( ooc_isInitialized( RAMDisk ) );
   chain_constructor( RAMDisk, self, NULL );
   ramdisk_params = (RAMDisk_constructor_params_t *)params;
   if(ramdisk_params != NULL)
   {
      /* TODO */
   }
   else
   {
      /* TODO */
   }

   self->data = NULL;
   self->numSectors = 0;
   //self->position = 0;
   self->status = RAMDISK_STATUS_UNINIT;
}

/** \brief Destructor */
static void RAMDisk_destructor( RAMDisk self, RAMDiskVtable vtab )
{
   /* Nothing allocated, no resources to free. Do nothing */
}

/** \brief Copy constructor */
static int RAMDisk_copy( RAMDisk self, const RAMDisk from )
{
   /* Prevent object duplication */
   return OOC_NO_COPY;
}

/*==================[external functions definition]==========================*/
/** \brief Class member functions */


RAMDisk RAMDisk_new(void)
{
   RAMDisk_constructor_params_t ramdisk_params;
   return (RAMDisk) ooc_new(RAMDisk, (void *)&ramdisk_params);
}

ramdisk_status_t RAMDisk_getStatus(RAMDisk self)
{
   return self->status;
}

int RAMDisk_init(RAMDisk self, void *data, uint32_t numSectors)
{
   int ret = -1;

   self->status = RAMDISK_STATUS_UNINIT;

   self->data = data;
   self->numSectors = numSectors;
   memset(data, 0, numSectors * RAMDISK_SECTORSIZE);

   self->status = RAMDISK_STATUS_READY;

   ret = 0;

   return ret;
}

int RAMDisk_singleBlockRead(RAMDisk self, uint8_t *readBlock, uint32_t sector)
{
   int ret = -1;

   if( RAMDISK_STATUS_READY == self->status && sector < self->numSectors )
   {
      memcpy((void *)readBlock, (void *)((uint8_t *)self->data + (sector * RAMDISK_SECTORSIZE)), RAMDISK_SECTORSIZE );
      ret = 0;
   }

   return ret;
}

int RAMDisk_singleBlockWrite(RAMDisk self, const uint8_t *writeBlock, uint32_t sector)
{
   int ret = -1;

   if( RAMDISK_STATUS_READY == self->status && sector < self->numSectors )
   {
      memcpy( (void *)((uint8_t *)self->data + (sector * RAMDISK_SECTORSIZE)), (void *)writeBlock, RAMDISK_SECTORSIZE );
      ret = 0;
   }

   return ret;
}

int RAMDisk_blockErase(RAMDisk self, uint32_t start, uint32_t end)
{
   int ret = -1;
   uint32_t i;
   static const uint8_t zeroBlock[RAMDISK_SECTORSIZE] = {0};

   if(start <= end && end < self->numSectors)
   {
      for(i = start; i <= end; i++)
      {
         if( RAMDisk_singleBlockWrite(self, zeroBlock, i) )
         {
            break;
         }
      }
      if(i == end+1)
      {
         ret = 0;
      }
   }
   return ret;
}

uint32_t RAMDisk_getSize(RAMDisk self)
{
   return self->numSectors;
}

#if 0 //Old version
/* BlockDevice interface implementation */
static ssize_t RAMDisk_read(RAMDisk self, uint8_t * const buf, size_t const nbyte)
{
   ssize_t ret = -1;
   size_t bytes_left, bytes_read, i, sector, position, bytes_offset;
   assert(ooc_isInstanceOf(self, RAMDisk));

   i=0; bytes_left = nbyte; sector = self->position / RAMDISK_SECTORSIZE; position = self->position;
   while(bytes_left)
   {
      bytes_offset = position % RAMDISK_SECTORSIZE;
      bytes_read = (bytes_left > (RAMDISK_SECTORSIZE - bytes_offset)) ? (RAMDISK_SECTORSIZE - bytes_offset) : bytes_left;
      if(RAMDisk_singleBlockRead(self, self->block_buf, sector) == 0)
      {
         memcpy(buf + i, self->block_buf + bytes_offset, bytes_read);
         bytes_left -= bytes_read;
         i += bytes_read;
         position += bytes_read;
         sector++;
      }
      else
      {
         break;
      }
   }
   if(0 == bytes_left)
   {
      ret = i;
      self->position += i;
   }
   else
   {

   }

   return ret;
}
#endif

static ssize_t RAMDisk_read(RAMDisk self, uint8_t * const buf, uint32_t sector, size_t count)
{
   ssize_t ret = -1;
   uint32_t i, position;

   assert(ooc_isInstanceOf(self, RAMDisk));

   i=0;
   position = 0; /* Offset in destination buffer */
   while(count)
   {
      if( RAMDisk_singleBlockRead(self, self->block_buf, sector + i) == 0 )
      {
         memcpy(buf + position, self->block_buf, RAMDISK_SECTORSIZE);
         count--;
         position += RAMDISK_SECTORSIZE;
         i++;
      }
      else
      {
         break;
      }
   }
   if(0 == count)
   {
      ret = i;
   }
   else
   {

   }

   return ret;
}


#if 0
static ssize_t RAMDisk_write(RAMDisk self, uint8_t const * const buf, size_t const nbyte)
{
   ssize_t ret = -1;
   size_t bytes_left, bytes_write, i, sector, position, bytes_offset;
   assert(ooc_isInstanceOf(self, RAMDisk));

   i=0; bytes_left = nbyte; sector = self->position / RAMDISK_SECTORSIZE; position = self->position;
   while(bytes_left)
   {
      bytes_offset = position % RAMDISK_SECTORSIZE;
      bytes_write = bytes_left > (RAMDISK_SECTORSIZE - bytes_offset) ? (RAMDISK_SECTORSIZE - bytes_offset) : bytes_left;
      //printf("RAMDisk_write(): bytes_left: %d bytes_write: %d sector: %d\n",
      //                  bytes_left, bytes_write, sector);
      if(RAMDisk_singleBlockRead(self, self->block_buf, sector) == 0)
      {
         //printf("RAMDisk_write(): Block readed, now copy new data\n");
         memcpy(self->block_buf + bytes_offset, buf + i, bytes_write);
         //printf("RAMDisk_write(): Data copied. Now write back\n");
         if(RAMDisk_singleBlockWrite(self, self->block_buf, sector) == 0)
         {
            //printf("RAMDisk_write(): write back succesfull. Next iteration\n");
            bytes_left -= bytes_write;
            i += bytes_write;
            position += bytes_write;
            sector++;
         }
         else
         {

         }
      }
   }
   if(0 == bytes_left)
   {

      ret = i;
      self->position = position;
   }
   else
   {

   }

   return ret;
}
#endif

static ssize_t RAMDisk_write(RAMDisk self, uint8_t const * const buf, uint32_t sector, size_t count)
{
   ssize_t ret = -1;
   uint32_t i, position;

   assert(ooc_isInstanceOf(self, RAMDisk));

   i=0;
   position = 0; /* Offset in destination buffer */
   while(count)
   {
      if( RAMDisk_singleBlockWrite(self, buf + position, sector + i) == 0 )
      {
         count--;
         position += RAMDISK_SECTORSIZE;
         i++;
      }
      else
      {
         break;
      }
   }
   if(0 == count)
   {
      ret = i;
   }
   else
   {

   }

   return ret;
}


#if 0
static ssize_t RAMDisk_lseek(RAMDisk self, off_t const offset, uint8_t const whence)
{
   assert(ooc_isInstanceOf(self, RAMDisk));
   off_t destination = -1;
   size_t partition_size = RAMDISK_SECTORSIZE * self->numSectors;

   switch(whence)
   {
      case SEEK_END:
         destination = partition_size + offset;
         break;
      case SEEK_CUR:
         destination = self->position + offset;
         break;
      default:
         destination = offset;
         break;
   }

   if ((destination >= 0) && (destination < partition_size))
   {
      self->position = destination;
   }

   return destination;
}
#endif

static int RAMDisk_ioctl(RAMDisk self, int32_t request, void* param)
{
   assert(ooc_isInstanceOf(self, RAMDisk));

   int32_t ret = -1;

   blockDevInfo_t * blockInfo = (blockDevInfo_t *)param;

   switch(request)
   {
      case IOCTL_BLOCK_GETINFO:
         blockInfo->size = RAMDISK_SECTORSIZE;
         blockInfo->num = self->numSectors;
         ret = 1;
         break;
      default:
         break;
   }

   return ret;
}

static int RAMDisk_connect(RAMDisk self)
{
   assert(ooc_isInstanceOf(self, RAMDisk));
   return 0;
}
static int RAMDisk_disconnect(RAMDisk self)
{
   assert(ooc_isInstanceOf(self, RAMDisk));
   return 0;
}

static int RAMDisk_getState(RAMDisk self, blockDevState_t *state)
{
   assert(ooc_isInstanceOf(self, RAMDisk));
	 /* TODO: Should specify specific state, but quick fix now */
	 *state = BLKDEV_UNINIT;
	 if(RAMDISK_STATUS_READY == self->status)
	    *state = BLKDEV_READY;
   return 0;
}

static int RAMDisk_getInfo(RAMDisk self, blockDevInfo_t *info)
{
   assert(ooc_isInstanceOf(self, RAMDisk));
   return 0;
}

/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/
