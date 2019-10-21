/*
    FreeRTOS V9.0.0 - Copyright (C) 2016 Real Time Engineers Ltd.
    All rights reserved

    VISIT http://www.FreeRTOS.org TO ENSURE YOU ARE USING THE LATEST VERSION.

    This file is part of the FreeRTOS distribution.

    FreeRTOS is free software; you can redistribute it and/or modify it under
    the terms of the GNU General Public License (version 2) as published by the
    Free Software Foundation >>!AND MODIFIED BY!<< the FreeRTOS exception.

    ***************************************************************************
    >>!   NOTE: The modification to the GPL is included to allow you to     !<<
    >>!   distribute a combined work that includes FreeRTOS without being   !<<
    >>!   obliged to provide the source code for proprietary components     !<<
    >>!   outside of the FreeRTOS kernel.                                   !<<
    ***************************************************************************

    FreeRTOS is distributed in the hope that it will be useful, but WITHOUT ANY
    WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
    FOR A PARTICULAR PURPOSE.  Full license text is available on the following
    link: http://www.freertos.org/a00114.html

    ***************************************************************************
     *                                                                       *
     *    FreeRTOS provides completely free yet professionally developed,    *
     *    robust, strictly quality controlled, supported, and cross          *
     *    platform software that is more than just the market leader, it     *
     *    is the industry's de facto standard.                               *
     *                                                                       *
     *    Help yourself get started quickly while simultaneously helping     *
     *    to support the FreeRTOS project by purchasing a FreeRTOS           *
     *    tutorial book, reference manual, or both:                          *
     *    http://www.FreeRTOS.org/Documentation                              *
     *                                                                       *
    ***************************************************************************

    http://www.FreeRTOS.org/FAQHelp.html - Having a problem?  Start by reading
    the FAQ page "My application does not run, what could be wrong?".  Have you
    defined configASSERT()?

    http://www.FreeRTOS.org/support - In return for receiving this top quality
    embedded software for free we request you assist our global community by
    participating in the support forum.

    http://www.FreeRTOS.org/training - Investing in training allows your team to
    be as productive as possible as early as possible.  Now you can receive
    FreeRTOS training directly from Richard Barry, CEO of Real Time Engineers
    Ltd, and the world's leading authority on the world's leading RTOS.

    http://www.FreeRTOS.org/plus - A selection of FreeRTOS ecosystem products,
    including FreeRTOS+Trace - an indispensable productivity tool, a DOS
    compatible FAT file system, and our tiny thread aware UDP/IP stack.

    http://www.FreeRTOS.org/labs - Where new FreeRTOS products go to incubate.
    Come and try FreeRTOS+TCP, our new open source TCP/IP stack for FreeRTOS.

    http://www.OpenRTOS.com - Real Time Engineers ltd. license FreeRTOS to High
    Integrity Systems ltd. to sell under the OpenRTOS brand.  Low cost OpenRTOS
    licenses offer ticketed support, indemnification and commercial middleware.

    http://www.SafeRTOS.com - High Integrity Systems also provide a safety
    engineered and independently SIL3 certified version for use in safety and
    mission critical applications that require provable dependability.

    1 tab == 4 spaces!
*/

/*
 * Instructions for using this project are provided on:
 * http://www.freertos.org/FreeRTOS-Plus/FreeRTOS_Plus_TCP/examples_FreeRTOS_simulator.html
 *
 * NOTE: Some versions of Visual Studio will generate erroneous compiler
 * warnings about variables being used before they are set.
 */

/* Standard includes. */
#include <stdio.h>
#include <time.h>
#include <string.h>
#include <strings.h>

/* FreeRTOS includes. */
#include <FreeRTOS.h>
#include "task.h"
#include "timers.h"
#include "queue.h"
#include "semphr.h"

/* FreeRTOS+TCP includes. */
#include "FreeRTOS_IP.h"
#include "FreeRTOS_Sockets.h"
#include "FreeRTOS_TCP_server.h"

/* FreeRTOS+FAT includes. */
#include "ff_headers.h"
#include "ff_stdio.h"
#include "ff_ramdisk.h"

/* Demo application includes. */
#include "SimpleTCPEchoServer.h"

#include "sapi.h"
#include "EMC.h"             /* <= EMC module header */

#include "ff.h"
#include "diskio.h"

/* VFS application includes */
#include "test_fs.h"  /* <= own header */
#include "ext2.h"
#include "fat.h"
#include "ooc.h"
#include "device.h"
//#include "mmcSPI.h"
#include "storageUSB.h"
#include "RAMDisk.h"
//#include "nbdclient.h"

/* VFS example test buffer size */
#define TEST_BUFFER_SIZE 1024

/* Simple UDP client and server task parameters. */
#define mainSIMPLE_UDP_CLIENT_SERVER_TASK_PRIORITY		( tskIDLE_PRIORITY )
#define mainSIMPLE_UDP_CLIENT_SERVER_PORT				( 5005UL )

/* FTP and HTTP servers execute in the TCP server work task. */
#define mainTCP_SERVER_TASK_PRIORITY					( tskIDLE_PRIORITY + 2 )
//#define	mainTCP_SERVER_STACK_SIZE						1400 /* Not used in the Win32 simulator. */
#define	mainTCP_SERVER_STACK_SIZE						3000

/* Dimensions the buffer used to send UDP print and debug messages. */
#define cmdPRINTF_BUFFER_SIZE		512

/* The number and size of sectors that will make up the RAM disk.  The RAM disk
is huge to allow some verbose FTP tests. */
#define mainRAM_DISK_SECTOR_SIZE	512UL /* Currently fixed! */
#define mainRAM_DISK_SECTORS		( ( 5UL * 1024UL * 1024UL ) / mainRAM_DISK_SECTOR_SIZE ) /* 5M bytes. */
//#define mainIO_MANAGER_CACHE_SIZE	( 15UL * mainRAM_DISK_SECTOR_SIZE )
#define mainIO_MANAGER_CACHE_SIZE	( 30UL * mainRAM_DISK_SECTOR_SIZE )

/* Where the RAM disk is mounted. */
//#define mainRAM_DISK_NAME			"/"
#define mainRAM_DISK_NAME "/mount/fat"
/* Set to 0 to run the STDIO examples once only, or 1 to create multiple tasks
that run the tests continuously. */
#define mainRUN_STDIO_TESTS_IN_MULTIPLE_TASK 0

/* Set the following constants to 1 or 0 to define which tasks to include and
exclude:

mainCREATE_FTP_SERVER:  When set to 1 the TCP server task will include an FTP
server.
See http://www.freertos.org/FreeRTOS-Plus/FreeRTOS_Plus_TCP/FTP_Server.html

mainCREATE_HTTP_SERVER:  When set to 1 the TCP server task will include a basic
HTTP server.
See http://www.freertos.org/FreeRTOS-Plus/FreeRTOS_Plus_TCP/HTTP_web_Server.html

mainCREATE_UDP_CLI_TASKS:  When set to 1 a command console that uses a UDP port
for input and output is created using FreeRTOS+CLI.  The port number used is set
by the mainUDP_CLI_PORT_NUMBER constant above.  A dumb UDP terminal such as YAT
can be used to connect.
See http://www.freertos.org/FreeRTOS-Plus/FreeRTOS_Plus_TCP/UDP_CLI.html

mainCREATE_TCP_CLI_TASKS:  When set to 1 a command console that uses a TCP port
for input and output is created using FreeRTOS+CLI.  The port number used is set
by the mainTCP_CLI_PORT_NUMBER constant above.  A dumb UDP terminal such as YAT
can be used to connect.

mainCREATE_SIMPLE_UDP_CLIENT_SERVER_TASKS:  When set to 1 two UDP client tasks
and two UDP server tasks are created.  The clients talk to the servers.  One set
of tasks use the standard sockets interface, and the other the zero copy sockets
interface.  These tasks are self checking and will trigger a configASSERT() if
they detect a difference in the data that is received from that which was sent.
As these tasks use UDP, and can therefore loose packets, they will cause
configASSERT() to be called when they are run in a less than perfect networking
environment.
See http://www.freertos.org/FreeRTOS-Plus/FreeRTOS_Plus_TCP/UDP_client_server.html

mainCREATE_SELECT_UDP_SERVER_TASKS: Uses two tasks to demonstrate the use of the
FreeRTOS_select() function.

mainCREATE_UDP_ECHO_TASKS:  When set to 1 a two tasks are created that send
UDP echo requests to the standard echo port (port 7).  One task uses the
standard socket interface, the other the zero copy socket interface.  The IP
address of the echo server must be configured using the configECHO_SERVER_ADDR0
to configECHO_SERVER_ADDR3 constants in FreeRTOSConfig.h.  These tasks are self
checking and will trigger a configASSERT() if the received echo reply does not
match the transmitted echo request.  As these tasks use UDP, and can therefore
loose packets, they will cause configASSERT() to be called when they are run in
a less than perfect networking environment, or when connected to an echo server
that (legitimately as UDP is used) opts not to reply to every echo request.
See http://www.freertos.org/FreeRTOS-Plus/FreeRTOS_Plus_TCP/UDP_Echo_Clients.html

mainCREATE_TCP_ECHO_TASKS_SINGLE:  When set to 1 a set of tasks are created that
send TCP echo requests to the standard echo port (port 7), then wait for and
verify the echo reply, from within the same task (Tx and Rx are performed in the
same RTOS task).  The IP address of the echo server must be configured using the
configECHO_SERVER_ADDR0 to configECHO_SERVER_ADDR3 constants in
FreeRTOSConfig.h.
See http://www.freertos.org/FreeRTOS-Plus/FreeRTOS_Plus_TCP/TCP_Echo_Clients.html

mainCREATE_TCP_ECHO_TASKS_SEPARATE:  As per the description for the
mainCREATE_TCP_ECHO_TASKS_SINGLE constant above, except this time separate tasks
are used to send data to and receive data from the echo server (one task is used
for Tx and another task for Rx, using the same socket).
See http://www.freertos.org/FreeRTOS-Plus/FreeRTOS_Plus_TCP/TCP_Echo_Clients_Separate.html

mainCREATE_SIMPLE_TCP_ECHO_SERVER:  When set to 1 FreeRTOS tasks are used with
FreeRTOS+TCP to create a TCP echo server.  Echo clients are also created, but
the echo clients use Windows threads (as opposed to FreeRTOS tasks) and use the
Windows TCP library (Winsocks).  This creates a communication between the
FreeRTOS+TCP TCP/IP stack and the Windows TCP/IP stack.
See http://www.freertos.org/FreeRTOS-Plus/FreeRTOS_Plus_TCP/TCP_Echo_Server.html
*/
#define mainCREATE_UDP_CLI_TASKS					0
#define mainCREATE_TCP_CLI_TASKS					0
#define mainCREATE_SIMPLE_UDP_CLIENT_SERVER_TASKS	0
#define mainCREATE_SELECT_UDP_SERVER_TASKS			0 /* _RB_ Requires retest. */
#define mainCREATE_UDP_ECHO_TASKS					0
#define mainCREATE_TCP_ECHO_TASKS_SINGLE			0
#define mainCREATE_TCP_ECHO_TASKS_SEPARATE			0
#define mainCREATE_SIMPLE_TCP_ECHO_SERVER			1
#define mainCREATE_FTP_SERVER						1
#define mainCREATE_HTTP_SERVER 						0
#define mainCREATE_TFTP_SERVER						0

/* Set the following constant to pdTRUE to log using the method indicated by the
name of the constant, or pdFALSE to not log using the method indicated by the
name of the constant.  Options include to standard out (mainLOG_TO_STDOUT), to a
disk file (mainLOG_TO_DISK_FILE), and to a UDP port (mainLOG_TO_UDP).  If
mainLOG_TO_UDP is set to pdTRUE then UDP messages are sent to the IP address
configured as the echo server address (see the configECHO_SERVER_ADDR0
definitions in FreeRTOSConfig.h) and the port number set by configPRINT_PORT in
FreeRTOSConfig.h. */
#define mainLOG_TO_STDOUT 		pdTRUE
#define mainLOG_TO_DISK_FILE 	pdFALSE
#define mainLOG_TO_UDP 			pdFALSE

/*-----------------------------------------------------------*/

void Sleep( volatile uint32_t dwMs );
void myTask( void* taskParmPtr );

static void test_fill_buffer(uint8_t *buffer, uint16_t size);
static int test_check_buffer(uint8_t *buffer, uint16_t size);
static void vfs_task( void );

/*
 * Register commands that can be used with FreeRTOS+CLI through the UDP socket.
 * The commands are defined in CLI-commands.c and File-related-CLI-commands.c
 * respectively.
 */
extern void vRegisterCLICommands( void );
extern void vRegisterFileSystemCLICommands( void );
extern DRESULT ramdisk_start(BYTE drive, unsigned char *data, int numBytes, int mkfs);

/*
 * Just seeds the simple pseudo random number generator.
 */
static void prvSRand( UBaseType_t ulSeed );

/*
 * Miscellaneous initialisation including preparing the logging and seeding the
 * random number generator.
 */
static void prvMiscInitialisation( void );

/*
 * Creates a RAM disk, then creates files on the RAM disk.  The files can then
 * be viewed via the FTP server and the command line interface.
 */
static void prvCreateDiskAndExampleFiles( void );

/*
 * Functions used to create and then test files on a disk.
 */
extern void vCreateAndVerifyExampleFiles( const char *pcMountPath );
extern void vStdioWithCWDTest( const char *pcMountPath );
extern void vMultiTaskStdioWithCWDTest( const char *const pcMountPath, uint16_t usStackSizeWords );

/*
 * The task that runs the FTP and HTTP servers.
 */
#if( ( mainCREATE_FTP_SERVER == 1 ) || ( mainCREATE_HTTP_SERVER == 1 ) )
	static void prvServerWorkTask( void *pvParameters );
#endif


/* The default IP and MAC address used by the demo.  The address configuration
defined here will be used if ipconfigUSE_DHCP is 0, or if ipconfigUSE_DHCP is
1 but a DHCP server could not be contacted.  See the online documentation for
more information. */
static const uint8_t ucIPAddress[ 4 ] = { configIP_ADDR0, configIP_ADDR1, configIP_ADDR2, configIP_ADDR3 };
static const uint8_t ucNetMask[ 4 ] = { configNET_MASK0, configNET_MASK1, configNET_MASK2, configNET_MASK3 };
static const uint8_t ucGatewayAddress[ 4 ] = { configGATEWAY_ADDR0, configGATEWAY_ADDR1, configGATEWAY_ADDR2, configGATEWAY_ADDR3 };
static const uint8_t ucDNSServerAddress[ 4 ] = { configDNS_SERVER_ADDR0, configDNS_SERVER_ADDR1, configDNS_SERVER_ADDR2, configDNS_SERVER_ADDR3 };

/* Default MAC address configuration.  The demo creates a virtual network
connection that uses this MAC address by accessing the raw Ethernet data
to and from a real network connection on the host PC.  See the
configNETWORK_INTERFACE_TO_USE definition for information on how to configure
the real network connection to use. */
const uint8_t ucMACAddress[ 6 ] = { configMAC_ADDR0, configMAC_ADDR1, configMAC_ADDR2, configMAC_ADDR3, configMAC_ADDR4, configMAC_ADDR5 };

/* The UDP address to which print messages are sent. */
static struct freertos_sockaddr xPrintUDPAddress;

/* Use by the pseudo random number generator. */
static UBaseType_t ulNextRand;

/* Handle of the task that runs the FTP and HTTP servers. */
static TaskHandle_t xServerWorkTaskHandle = NULL;

/*-----------------------------------------------------------*/

__attribute__ ((section(".mysection"))) static uint8_t ucRAMDisk[ mainRAM_DISK_SECTORS * mainRAM_DISK_SECTOR_SIZE ];
DEBUG_PRINT_ENABLE;

/** \brief Filesystem drivers declaration
 *
 * Here are the drivers defined in the lower layer file system implementations (in ext2.c, fat.c, etc.)
 *
 */
extern filesystem_driver_t ext2_driver;
extern filesystem_driver_t fat_driver;

/*-----------------------------------------------------------*/

/*-----------------------------------------------------------*/

/*
 * NOTE: Some versions of Visual Studio will generate erroneous compiler
 * warnings about variables being used before they are set.
 */
int main( void )
{
const uint32_t ulLongTime_ms = 250UL;

   // Inicializar y configurar la plataforma
   Chip_SetupCoreClock(CLKIN_IRC, 77000000, true);
   boardConfig();
   //Chip_SetupCoreClock(CLKIN_IRC, 77000000, true);
   EMC_Initialize();

   // UART for debug messages
   debugPrintConfigUart( UART_USB, 115200 );
   debugPrintlnString( "Blinky con freeRTOS y sAPI." );
   stdioConfig(UART_USB);
   NVIC_SetPriority( USART2_IRQn, 20 );
   NVIC_SetPriority (SysTick_IRQn, 0);
   // Led para dar se�al de vida
   gpioWrite( DO4, ON );

	/* Crear tarea LED */
	xTaskCreate(
                myTask,                     // Funcion de la tarea a ejecutar
		          (const char *)"myTask",     // Nombre de la tarea como String amigable para el usuario
					 configMINIMAL_STACK_SIZE*2, // Tama�o del stack de la tarea
					 0,                          // Parametros de tarea
					 tskIDLE_PRIORITY+1,         // Prioridad de la tarea
					 0                           // Puntero a la tarea creada en el sistema
              );

	/*
	 * Instructions for using this project are provided on:
	 * http://www.freertos.org/FreeRTOS-Plus/FreeRTOS_Plus_TCP/examples_FreeRTOS_simulator.html
	 *
	 * NOTE: Some versions of Visual Studio will generate erroneous compiler
	 * warnings about variables being used before they are set.
	 */

	/* Miscellaneous initialisation including preparing the logging and seeding
	the random number generator. */
	prvMiscInitialisation();

	/* Initialise the network interface.

	***NOTE*** Tasks that use the network are created in the network event hook
	when the network is connected and ready for use (see the definition of
	vApplicationIPNetworkEventHook() below).  The address values passed in here
	are used if ipconfigUSE_DHCP is set to 0, or if ipconfigUSE_DHCP is set to 1
	but a DHCP server cannot be	contacted. */
	FreeRTOS_debug_printf( ( "FreeRTOS_IPInit\n" ) );
	FreeRTOS_IPInit( ucIPAddress, ucNetMask, ucGatewayAddress, ucDNSServerAddress, ucMACAddress );

	#if( ( mainCREATE_FTP_SERVER == 1 ) || ( mainCREATE_HTTP_SERVER == 1 ) )
	{
		/* Create the task that handles the FTP and HTTP servers.  This will
		initialise the file system then wait for a notification from the network
		event hook before creating the servers.  The task is created at the idle
		priority, and sets itself to mainTCP_SERVER_TASK_PRIORITY after the file
		system has initialised. */
		xTaskCreate( prvServerWorkTask, "SvrWork", mainTCP_SERVER_STACK_SIZE, NULL, tskIDLE_PRIORITY, &xServerWorkTaskHandle );
	}
	#endif

	/* Start the RTOS scheduler. */
	FreeRTOS_debug_printf( ("vTaskStartScheduler\n") );
	vTaskStartScheduler();

	/* If all is well, the scheduler will now be running, and the following
	line will never be reached.  If the following line does execute, then
	there was insufficient FreeRTOS heap memory available for the idle and/or
	timer tasks	to be created.  See the memory management section on the
	FreeRTOS web site for more details (this is standard text that is not not
	really applicable to the Win32 simulator port). */
	for( ;; )
	{
		Sleep( ulLongTime_ms );
	}
}
/*-----------------------------------------------------------*/

static void prvCreateDiskAndExampleFiles( void )
{

DRESULT res;
FF_Disk_t *pxDisk;
   uint32_t count;
    //FIL fp;
    //FATFS fs;
    //char buff[256];

	/* Create the RAM disk. */
	pxDisk = FF_RAMDiskInit( mainRAM_DISK_NAME, ucRAMDisk, mainRAM_DISK_SECTORS, mainIO_MANAGER_CACHE_SIZE );
	configASSERT( pxDisk );

	/* Print out information on the disk. */
	FF_RAMDiskShowPartition( pxDisk );

	/* Create a few example files on the disk.  These are not deleted again. */
	//vCreateAndVerifyExampleFiles( mainRAM_DISK_NAME );
	/* Create and verify VFS sample files in USB disk */
	vfs_task();

	/* A few sanity checks only - can only be called after
	vCreateAndVerifyExampleFiles(). */
	#if( mainRUN_STDIO_TESTS_IN_MULTIPLE_TASK == 1 )
	{
		/* Note the stack size is not actually used in the Windows port. */
		vMultiTaskStdioWithCWDTest( mainRAM_DISK_NAME, configMINIMAL_STACK_SIZE * 2U );
	}
	#else
	{
		//vStdioWithCWDTest( mainRAM_DISK_NAME );
	}
	#endif
}
/*-----------------------------------------------------------*/

#if( ( mainCREATE_FTP_SERVER == 1 ) || ( mainCREATE_HTTP_SERVER == 1 ) )

	static void prvServerWorkTask( void *pvParameters )
	{
	TCPServer_t *pxTCPServer = NULL;
	const TickType_t xInitialBlockTime = pdMS_TO_TICKS( 200UL );

	/* A structure that defines the servers to be created.  Which servers are
	included in the structure depends on the mainCREATE_HTTP_SERVER and
	mainCREATE_FTP_SERVER settings at the top of this file. */
	static const struct xSERVER_CONFIG xServerConfiguration[] =
	{
		#if( mainCREATE_HTTP_SERVER == 1 )
				/* Server type,		port number,	backlog, 	root dir. */
				{ eSERVER_HTTP, 	80, 			12, 		configHTTP_ROOT },
		#endif

		#if( mainCREATE_FTP_SERVER == 1 )
				/* Server type,		port number,	backlog, 	root dir. */
				{ eSERVER_FTP,  	21, 			12, 		"" }
		#endif
	};

		/* Remove compiler warning about unused parameter. */
		( void ) pvParameters;

		/* Create the RAM disk used by the FTP and HTTP servers. */
		prvCreateDiskAndExampleFiles();

		/* The priority of this task can be raised now the disk has been
		initialised. */
		vTaskPrioritySet( NULL, mainTCP_SERVER_TASK_PRIORITY );

		/* If the CLI is included in the build then register commands that allow
		the file system to be accessed. */
		#if( ( mainCREATE_UDP_CLI_TASKS == 1 ) || ( mainCREATE_TCP_CLI_TASKS == 1 ) )
		{
			vRegisterFileSystemCLICommands();
		}
		#endif /* mainCREATE_UDP_CLI_TASKS */


		/* Wait until the network is up before creating the servers.  The
		notification is given from the network event hook. */
		ulTaskNotifyTake( pdTRUE, portMAX_DELAY );

		/* Create the servers defined by the xServerConfiguration array above. */
		pxTCPServer = FreeRTOS_CreateTCPServer( xServerConfiguration, sizeof( xServerConfiguration ) / sizeof( xServerConfiguration[ 0 ] ) );
		configASSERT( pxTCPServer );

		for( ;; )
		{
			FreeRTOS_TCPServerWork( pxTCPServer, xInitialBlockTime );
		}
	}

#endif /* ( ( mainCREATE_FTP_SERVER == 1 ) || ( mainCREATE_HTTP_SERVER == 1 ) ) */
/*-----------------------------------------------------------*/

void vApplicationIdleHook( void )
{
const uint32_t ulMSToSleep = 1;

	/* This is just a trivial example of an idle hook.  It is called on each
	cycle of the idle task if configUSE_IDLE_HOOK is set to 1 in
	FreeRTOSConfig.h.  It must *NOT* attempt to block.  In this case the
	idle task just sleeps to lower the CPU usage. */
	Sleep( ulMSToSleep );
}
/*-----------------------------------------------------------*/

void vAssertCalled( const char *pcFile, uint32_t ulLine )
{
const uint32_t ulLongSleep = 1000UL;
volatile uint32_t ulBlockVariable = 0UL;
volatile char *pcFileName = ( volatile char *  ) pcFile;
volatile uint32_t ulLineNumber = ulLine;

	( void ) pcFileName;
	( void ) ulLineNumber;

	FreeRTOS_printf( ( "vAssertCalled( %s, %ld\n", pcFile, ulLine ) );

	/* Setting ulBlockVariable to a non-zero value in the debugger will allow
	this function to be exited. */
	taskDISABLE_INTERRUPTS();
	{
		while( ulBlockVariable == 0UL )
		{
			Sleep( ulLongSleep );
		}
	}
	taskENABLE_INTERRUPTS();
}
/*-----------------------------------------------------------*/

/* Called by FreeRTOS+TCP when the network connects or disconnects.  Disconnect
events are only received if implemented in the MAC driver. */
void vApplicationIPNetworkEventHook( eIPCallbackEvent_t eNetworkEvent )
{
uint32_t ulIPAddress, ulNetMask, ulGatewayAddress, ulDNSServerAddress;
char cBuffer[ 16 ];
static BaseType_t xTasksAlreadyCreated = pdFALSE;

	/* If the network has just come up...*/
	if( eNetworkEvent == eNetworkUp )
	{
		/* Create the tasks that use the IP stack if they have not already been
		created. */
		if( xTasksAlreadyCreated == pdFALSE )
		{
			/* See the comments above the definitions of these pre-processor
			macros at the top of this file for a description of the individual
			demo tasks. */
			#if( mainCREATE_SIMPLE_UDP_CLIENT_SERVER_TASKS == 1 )
			{
				vStartSimpleUDPClientServerTasks( configMINIMAL_STACK_SIZE, mainSIMPLE_UDP_CLIENT_SERVER_PORT, mainSIMPLE_UDP_CLIENT_SERVER_TASK_PRIORITY );
			}
			#endif /* mainCREATE_SIMPLE_UDP_CLIENT_SERVER_TASKS */

			#if( mainCREATE_SELECT_UDP_SERVER_TASKS == 1 )
			{
				vStartUDPSelectServerTasks( configMINIMAL_STACK_SIZE, mainUDP_SELECT_SERVER_PORT, mainUDP_SELECT_SERVER_TASK_PRIORITY );
			}
			#endif /* mainCREATE_SIMPLE_UDP_CLIENT_SERVER_TASKS */

			#if( mainCREATE_UDP_ECHO_TASKS == 1 )
			{
				vStartUDPEchoClientTasks( mainECHO_CLIENT_TASK_STACK_SIZE, mainECHO_CLIENT_TASK_PRIORITY );
			}
			#endif /* mainCREATE_UDP_ECHO_TASKS */

			#if( mainCREATE_TCP_ECHO_TASKS_SINGLE == 1 )
			{
				vStartTCPEchoClientTasks_SingleTasks( mainECHO_CLIENT_TASK_STACK_SIZE, mainECHO_CLIENT_TASK_PRIORITY );
			}
			#endif /* mainCREATE_TCP_ECHO_TASKS_SINGLE */

			#if( mainCREATE_TCP_ECHO_TASKS_SEPARATE == 1 )
			{
				vStartTCPEchoClientTasks_SeparateTasks( mainECHO_CLIENT_TASK_STACK_SIZE, mainECHO_CLIENT_TASK_PRIORITY );
			}
			#endif /* mainCREATE_TCP_ECHO_TASKS_SEPARATE */

			#if( mainCREATE_TFTP_SERVER == 1 )
			{
				vStartTFTPServerTask( mainTFTP_SERVER_STACK_SIZE, mainTFTP_SERVER_PRIORITY );
			}
			#endif /* mainCREATE_TFTP_SERVER */

			#if( mainCREATE_UDP_CLI_TASKS == 1 )
			{
				/* Register example commands with the FreeRTOS+CLI command
				interpreter via the UDP port specified by the
				mainUDP_CLI_PORT_NUMBER constant. */
				vRegisterCLICommands();
				vStartUDPCommandInterpreterTask( configMINIMAL_STACK_SIZE, mainUDP_CLI_PORT_NUMBER, mainUDP_CLI_TASK_PRIORITY );
			}
			#endif /* mainCREATE_UDP_CLI_TASKS */

			#if( mainCREATE_TCP_CLI_TASKS == 1 )
			{
				/* Register example commands with the FreeRTOS+CLI command
				interpreter via the TCP port specified by the
				mainTCP_CLI_PORT_NUMBER constant. */
				vRegisterCLICommands();
				vStartTCPCommandInterpreterTask( configMINIMAL_STACK_SIZE, mainTCP_CLI_PORT_NUMBER, mainTCP_CLI_TASK_PRIORITY );
			}
			#endif /* mainCREATE_TCPP_CLI_TASKS */

			#if( mainCREATE_SIMPLE_TCP_ECHO_SERVER == 1 )
			{
				/* TCP server on port 5001, using multiple threads */
				vStartSimpleTCPServerTasks( configMINIMAL_STACK_SIZE, mainSIMPLE_UDP_CLIENT_SERVER_TASK_PRIORITY );
			}
			#endif /* mainCREATE_SIMPLE_TCP_ECHO_SERVER */

			#if( ( mainCREATE_FTP_SERVER == 1 ) || ( mainCREATE_HTTP_SERVER == 1 ) )
			{
				/* See TBD.
				Let the server work task now it can now create the servers. */
				xTaskNotifyGive( xServerWorkTaskHandle );
			}
			#endif

			xTasksAlreadyCreated = pdTRUE;
		}

		/* Print out the network configuration, which may have come from a DHCP
		server. */
		FreeRTOS_GetAddressConfiguration( &ulIPAddress, &ulNetMask, &ulGatewayAddress, &ulDNSServerAddress );
		FreeRTOS_inet_ntoa( ulIPAddress, cBuffer );
		FreeRTOS_printf( ( "\r\n\r\nIP Address: %s\r\n", cBuffer ) );

		FreeRTOS_inet_ntoa( ulNetMask, cBuffer );
		FreeRTOS_printf( ( "Subnet Mask: %s\r\n", cBuffer ) );

		FreeRTOS_inet_ntoa( ulGatewayAddress, cBuffer );
		FreeRTOS_printf( ( "Gateway Address: %s\r\n", cBuffer ) );

		FreeRTOS_inet_ntoa( ulDNSServerAddress, cBuffer );
		FreeRTOS_printf( ( "DNS Server Address: %s\r\n\r\n\r\n", cBuffer ) );
	}
}
/*-----------------------------------------------------------*/

/* Called automatically when a reply to an outgoing ping is received. */
void vApplicationPingReplyHook( ePingReplyStatus_t eStatus, uint16_t usIdentifier )
{
static const char *pcSuccess = "Ping reply received - ";
static const char *pcInvalidChecksum = "Ping reply received with invalid checksum - ";
static const char *pcInvalidData = "Ping reply received with invalid data - ";

	switch( eStatus )
	{
		case eSuccess	:
			FreeRTOS_printf( ( pcSuccess ) );
			break;

		case eInvalidChecksum :
			FreeRTOS_printf( ( pcInvalidChecksum ) );
			break;

		case eInvalidData :
			FreeRTOS_printf( ( pcInvalidData ) );
			break;

		default :
			/* It is not possible to get here as all enums have their own
			case. */
			break;
	}

	FreeRTOS_printf( ( "identifier %d\r\n", ( int ) usIdentifier ) );

	/* Prevent compiler warnings in case FreeRTOS_debug_printf() is not defined. */
	( void ) usIdentifier;
}
/*-----------------------------------------------------------*/

void vApplicationMallocFailedHook( void )
{
	/* Called if a call to pvPortMalloc() fails because there is insufficient
	free memory available in the FreeRTOS heap.  pvPortMalloc() is called
	internally by FreeRTOS API functions that create tasks, queues, software
	timers, and semaphores.  The size of the FreeRTOS heap is set by the
	configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h. */
	vAssertCalled( __FILE__, __LINE__ );
}
/*-----------------------------------------------------------*/

/*-----------------------------------------------------------*/

UBaseType_t uxRand( void )
{
const uint32_t ulMultiplier = 0x015a4e35UL, ulIncrement = 1UL;

	/* Utility function to generate a pseudo random number. */

	ulNextRand = ( ulMultiplier * ulNextRand ) + ulIncrement;
	return( ( int ) ( ulNextRand >> 16UL ) & 0x7fffUL );
}
/*-----------------------------------------------------------*/

static void prvSRand( UBaseType_t ulSeed )
{
	/* Utility function to seed the pseudo random number generator. */
    ulNextRand = ulSeed;
}
/*-----------------------------------------------------------*/

static void prvMiscInitialisation( void )
{
time_t xTimeNow;
uint32_t ulLoggingIPAddress;

	ulLoggingIPAddress = FreeRTOS_inet_addr_quick( configECHO_SERVER_ADDR0, configECHO_SERVER_ADDR1, configECHO_SERVER_ADDR2, configECHO_SERVER_ADDR3 );
	//vLoggingInit( mainLOG_TO_STDOUT, mainLOG_TO_DISK_FILE, mainLOG_TO_UDP, ulLoggingIPAddress, configPRINT_PORT );

	/* Seed the random number generator. */
	time( &xTimeNow );
	FreeRTOS_debug_printf( ( "Seed for randomiser: %lu\n", xTimeNow ) );
	prvSRand( ( uint32_t ) xTimeNow );
	FreeRTOS_debug_printf( ( "Random numbers: %08X %08X %08X %08X\n", ipconfigRAND32(), ipconfigRAND32(), ipconfigRAND32(), ipconfigRAND32() ) );
}
/*-----------------------------------------------------------*/

struct tm *gmtime_r( const time_t *pxTime, struct tm *tmStruct )
{
	/* Dummy time functions to keep the file system happy in the absence of
	target support. */
	memcpy( tmStruct, gmtime( pxTime ), sizeof( * tmStruct ) );
	return tmStruct;
}
/*-----------------------------------------------------------*/

/*-----------------------------------------------------------*/

#if( ipconfigUSE_LLMNR != 0 ) || ( ipconfigUSE_NBNS != 0 ) || ( ipconfigDHCP_REGISTER_HOSTNAME != 0 )

	const char *pcApplicationHostnameHook( void )
	{
		/* Assign the name "FreeRTOS" to this network node.  This function will
		be called during the DHCP: the machine will be registered with an IP
		address plus this name. */
		return mainHOST_NAME;
	}

#endif
/*-----------------------------------------------------------*/

#if( ipconfigUSE_LLMNR != 0 ) || ( ipconfigUSE_NBNS != 0 )

	BaseType_t xApplicationDNSQueryHook( const char *pcName )
	{
	BaseType_t xReturn;

		/* Determine if a name lookup is for this node.  Two names are given
		to this node: that returned by pcApplicationHostnameHook() and that set
		by mainDEVICE_NICK_NAME. */
		if( FF_stricmp( pcName, pcApplicationHostnameHook() ) == 0 )
		{
			xReturn = pdPASS;
		}
		else if( FF_stricmp( pcName, mainDEVICE_NICK_NAME ) == 0 )
		{
			xReturn = pdPASS;
		}
		else
		{
			xReturn = pdFAIL;
		}

		return xReturn;
	}

#endif
/*-----------------------------------------------------------*/

/*
 * Callback that provides the inputs necessary to generate a randomized TCP
 * Initial Sequence Number per RFC 6528.  THIS IS ONLY A DUMMY IMPLEMENTATION
 * THAT RETURNS A PSEUDO RANDOM NUMBER SO IS NOT INTENDED FOR USE IN PRODUCTION
 * SYSTEMS.
 */
extern uint32_t ulApplicationGetNextSequenceNumber( uint32_t ulSourceAddress,
													uint16_t usSourcePort,
													uint32_t ulDestinationAddress,
													uint16_t usDestinationPort )
{
	( void ) ulSourceAddress;
	( void ) usSourcePort;
	( void ) ulDestinationAddress;
	( void ) usDestinationPort;

	return uxRand();
}

/*-----------------------------------------------------------*/

// Implementacion de funcion de la tarea
void myTask( void* taskParmPtr )
{
   // ---------- CONFIGURACIONES ------------------------------

   // ---------- REPETIR POR SIEMPRE --------------------------
	while(TRUE)
   {
      // Intercambia el estado del LEDB
		gpioToggle( DO7 );

      // Envia la tarea al estado bloqueado durante 500ms
		vTaskDelay( 500 / portTICK_RATE_MS );
	}
}

/*-----------------------------------------------------------*/

void Sleep( volatile uint32_t dwMs )
{

}

void prvGetRegistersFromStack( uint32_t *pulFaultStackAddress )
{
/* These are volatile to try and prevent the compiler/linker optimising them
away as the variables never actually get used.  If the debugger won't show the
values of the variables, make them global my moving their declaration outside
of this function. */
volatile uint32_t r0;
volatile uint32_t r1;
volatile uint32_t r2;
volatile uint32_t r3;
volatile uint32_t r12;
volatile uint32_t lr; /* Link register. */
volatile uint32_t pc; /* Program counter. */
volatile uint32_t psr;/* Program status register. */


uint32_t cfsr  = SCB->CFSR;
uint32_t hfsr  = SCB->HFSR;
uint32_t mmfar = SCB->MMFAR;
uint32_t bfar  = SCB->BFAR;


    r0 = pulFaultStackAddress[ 0 ];
    r1 = pulFaultStackAddress[ 1 ];
    r2 = pulFaultStackAddress[ 2 ];
    r3 = pulFaultStackAddress[ 3 ];

    r12 = pulFaultStackAddress[ 4 ];
    lr = pulFaultStackAddress[ 5 ];
    pc = pulFaultStackAddress[ 6 ];
    psr = pulFaultStackAddress[ 7 ];

    /* When the following line is hit, the variables contain the register values. */
    for( ;; );
}

/* The fault handler implementation calls a function called
prvGetRegistersFromStack(). */

__attribute__ ((section(".after_vectors"))) __attribute__ ((naked))
void HardFault_Handler(void)
{
    __asm volatile
    (
        " tst lr, #4                                                \n"
        " ite eq                                                    \n"
        " mrseq r0, msp                                             \n"
        " mrsne r0, psp                                             \n"
        " ldr r1, [r0, #24]                                         \n"
        " ldr r2, handler2_address_const                            \n"
        " bx r2                                                     \n"
        " handler2_address_const: .word prvGetRegistersFromStack    \n"
    );
}

static void vfs_task( void )
{
   //MmcSPI mmc0;
   StorageUSB usb0;
   //Nbd nbd0;
   RAMDisk ram0;
   filesystem_info_t *fs0, *fs1;
   file_desc_t *file0, *file1, *file2, *file3, *file4, *file5, *file6, *file7;

   uint8_t buffer[TEST_BUFFER_SIZE];
   fat_format_param_t fat_format_parameters;
   ext2_format_param_t e2_format_parameters;

   int32_t ret, ret0, ret1;
   uint32_t lret;
   uint32_t i;

   /* init CIAA kernel and devices */
   ret=vfs_init();
   if(ret < 0)
   {
      debugPrintlnString( "vfs_init()" );
      while(1);
   }
   /* Basic test
    * This secuence test the basic operation of MMC device.
    * Open the device, get info block, erase a block and verify or cleared,
    * write a block, read this block and verify the data read is same as writed
    * finally close the device
    */

   /* Create device and initialize it */
   //ooc_init_class(MmcSPI);
   ooc_init_class(StorageUSB);
   //ooc_init_class(Nbd);
   ooc_init_class(RAMDisk);

   //mmc0 = mmcSPI_new(); if(NULL == mmc0) while(1);
   usb0 = storageUSB_new(); if(NULL == usb0) while(1);
   //nbd0 = nbd_new(); if(NULL == nbd0) while(1);
   ram0 = RAMDisk_new(); if(NULL == ram0) while(1);

   ret0 = RAMDisk_init(ram0, (void *)ucRAMDisk, mainRAM_DISK_SECTORS); //if(ret < 0) while(1);
   ret1 = storageUSB_init(usb0); //if(ret < 0) while(1);
   //ret = mmcSPI_init(mmc0); //if(ret < 0) while(1);
   //ret = nbd_init(nbd0); //if(ret < 0) while(1);
   if(0 == ret0 && 0 == ret1)
   {
      // Turn ON LEDG if the write operation was successful
      gpioWrite( DO4, ON );
   }
   else
   {
      // Turn ON LEDR if the write operation was fail
      while(1);
   }

   /* Create file system object with device and fs driver */
   //ret = filesystem_create(&fs, (Device *) &mmc0, &fat_driver); if(ret < 0) while(1);
   //ret0 = filesystem_create(&fs0, (Device *) &usb0, &fat_driver); if(ret < 0) while(1);
   ret0 = filesystem_create(&fs0, (Device *) &usb0, &ext2_driver); if(ret < 0) while(1);
   //ret = filesystem_create(&fs, (Device *) &nbd0, &fat_driver); if(ret < 0) while(1);
   ret1 = filesystem_create(&fs1, (Device *) &ram0, &fat_driver); if(ret < 0) while(1);

   /* format */
   /* Set ext2 format parameters */
   e2_format_parameters.partition_size = 32*1024; //2GB0
   e2_format_parameters.block_size = 1024;
   e2_format_parameters.block_node_factor = EXT2_DEFAULT_BLOCKNODE_FACTOR;

   fat_format_parameters.partition_size = 32*1024; //2GB0

   //ret0 = vfs_format(&fs0, &e2_format_parameters);
   //ret0 = vfs_format(&fs0, &fat_format_parameters);
   ret0 = 0;
   ret1 = vfs_format(&fs1, NULL);
   if(0 == ret0 && 0 == ret1)
   {
      // Turn ON LEDG if the write operation was successful
      gpioWrite( DO5, ON );
   }
   else
   {
      // Turn ON LEDR if the write operation was fail
      while(1);
   }

   /* mount */
   ret = vfs_mkdir("/mount", 0); if(ret < 0) while(1);

   //ASSERT_SEQ(2);

   ret0 = vfs_mount("/mount/usb", &fs0);
   ret1 = vfs_mount("/mount/ram", &fs1);
   if(ret0 < 0 || ret1 < 0) while(1);

   //ASSERT_SEQ(3);
/***************************************************************/
/* Test write beyond direct blocks */
   vfs_unlink("/mount/usb/file0");
   ret = vfs_open("/mount/usb/file0", &file0, VFS_O_CREAT); if(ret < 0) while(1);
   test_fill_buffer(buffer, TEST_BUFFER_SIZE);
   for(i = 0; i<50; i++)
   {
      lret = vfs_write(&file0, buffer, TEST_BUFFER_SIZE);
      if(lret != TEST_BUFFER_SIZE) break;
   }
   while(1);
/***************************************************************/


   ret = vfs_mkdir("/mount/ram/dir0", 0); if(ret < 0) while(1);
   ret = vfs_mkdir("/mount/ram/dir1", 0); if(ret < 0) while(1);
   ret = vfs_mkdir("/mount/ram/dir2", 0); if(ret < 0) while(1);
   ret = vfs_mkdir("/mount/ram/dir2/dir3", 0); if(ret < 0) while(1);
   /*
   ret = vfs_mkdir("/mount/usb/dir4", 0); if(ret < 0) while(1);
   ret = vfs_mkdir("/mount/usb/dir5", 0); if(ret < 0) while(1);
   ret = vfs_mkdir("/mount/usb/dir6", 0); if(ret < 0) while(1);
   ret = vfs_mkdir("/mount/usb/dir6/dir7", 0); if(ret < 0) while(1);
   */
   //ASSERT_SEQ(4);
   /* Show actual vfs tree */

   //ASSERT_SEQ(5);
   /* Fixme: Duplicated file in FAT. Error not handled */
   ret = vfs_open("/mount/ram/file0", &file0, VFS_O_CREAT); if(ret < 0) while(1);
   ret = vfs_open("/mount/ram/file1", &file1, VFS_O_CREAT); if(ret < 0) while(1);
   /*
   ret = vfs_open("/mount/usb/file4", &file4, VFS_O_CREAT); if(ret < 0) while(1);
   ret = vfs_open("/mount/usb/file5", &file5, VFS_O_CREAT); if(ret < 0) while(1);
   */
   //ASSERT_SEQ(6);

   ret = vfs_open("/mount/ram/dir2/file2", &file2, VFS_O_CREAT); if(ret < 0) while(1);
   ret = vfs_open("/mount/ram/dir2/file3", &file3, VFS_O_CREAT); if(ret < 0) while(1);
   ret = vfs_close(&file0); if(ret < 0) while(1);
   ret = vfs_close(&file1); if(ret < 0) while(1);
   ret = vfs_close(&file2); if(ret < 0) while(1);
   ret = vfs_close(&file3); if(ret < 0) while(1);
   /*
   ret = vfs_open("/mount/usb/dir6/file6", &file6, VFS_O_CREAT);
   if(ret < 0) while(1);
   ret = vfs_open("/mount/usb/dir6/file7", &file7, VFS_O_CREAT); if(ret < 0) while(1);
   ret = vfs_close(&file4); if(ret < 0) while(1);
   ret = vfs_close(&file5); if(ret < 0) while(1);
   ret = vfs_close(&file6); if(ret < 0) while(1);
   ret = vfs_close(&file7); if(ret < 0) while(1);
   */
   //ASSERT_SEQ(7);

   ret = vfs_open("/mount/ram/file0", &file0, 0); if(ret < 0) while(1);
   test_fill_buffer(buffer, TEST_BUFFER_SIZE);
   lret = vfs_write(&file0, buffer, TEST_BUFFER_SIZE); if(lret < 0) while(1);
   ret = vfs_close(&file0); if(ret < 0) while(1);
   ret = vfs_open("/mount/ram/file0", &file0, 0); if(ret < 0) while(1);
   lret = vfs_size( &file0 );
   FreeRTOS_debug_printf( ( "file 0 size: %lu\n", lret ) );
   debugPrintlnString( "file 0 size: " ); debugPrintUInt(lret); debugPrintEnter();

   memset(buffer, 0, TEST_BUFFER_SIZE);
   lret = vfs_read(&file0, buffer, TEST_BUFFER_SIZE); if(lret != TEST_BUFFER_SIZE) while(1);
   ret = test_check_buffer(buffer, TEST_BUFFER_SIZE); if(ret < 0) while(1);
   lret = vfs_read(&file0, buffer, TEST_BUFFER_SIZE); if(lret != 0) while(1);
   /*
   ret = vfs_open("/mount/usb/file4", &file4, 0); if(ret < 0) while(1);
   test_fill_buffer(buffer, TEST_BUFFER_SIZE);
   lret = vfs_write(&file4, buffer, TEST_BUFFER_SIZE); if(lret < 0) while(1);
   ret = vfs_close(&file4); if(ret < 0) while(1);
   ret = vfs_open("/mount/usb/file4", &file4, 0); if(ret < 0) while(1);
   lret = vfs_size( &file4 );
   FreeRTOS_debug_printf( ( "file 4 size: %lu\n", lret ) );
   debugPrintlnString( "file 4 size: " ); debugPrintUInt(lret); debugPrintEnter();

   memset(buffer, 0, TEST_BUFFER_SIZE);
   lret = vfs_read(&file4, buffer, TEST_BUFFER_SIZE); if(lret != TEST_BUFFER_SIZE) while(1);
   ret = test_check_buffer(buffer, TEST_BUFFER_SIZE); if(ret < 0) while(1);
   lret = vfs_read(&file4, buffer, TEST_BUFFER_SIZE); if(lret != 0) while(1);
   */
   //ASSERT_SEQ(8);

   ret = vfs_close(&file0); if(ret < 0) while(1);
   ret = vfs_unlink("/mount/ram/dir2/file2"); if(ret < 0) while(1);
   ret = vfs_unlink("/mount/ram/dir2/file3"); if(ret < 0) while(1);
   ret = vfs_open("/mount/ram/dir2/file4", &file0, VFS_O_CREAT); if(ret < 0) while(1);
   ret = vfs_open("/mount/ram/dir2/file5", &file1, VFS_O_CREAT); if(ret < 0) while(1);
   ret = vfs_close(&file1); if(ret < 0) while(1);
   /*
   ret = vfs_close(&file4); if(ret < 0) while(1);
   ret = vfs_unlink("/mount/usb/dir6/file6"); if(ret < 0) while(1);
   ret = vfs_unlink("/mount/usb/dir6/file7"); if(ret < 0) while(1);
   ret = vfs_open("/mount/usb/dir6/file8", &file4, VFS_O_CREAT); if(ret < 0) while(1);
   ret = vfs_open("/mount/usb/dir6/file9", &file5, VFS_O_CREAT); if(ret < 0) while(1);
   ret = vfs_close(&file5); if(ret < 0) while(1);
   */
   //ASSERT_SEQ(9);

   ret = vfs_open("/mount/ram/dir2/file5", &file3, VFS_O_CREAT); if(ret < 0) while(1);
   test_fill_buffer(buffer, TEST_BUFFER_SIZE);
   lret = vfs_write(&file3, buffer, TEST_BUFFER_SIZE); if(lret != TEST_BUFFER_SIZE) while(1);
   memset(buffer, 0, TEST_BUFFER_SIZE);
   lret = vfs_lseek(&file3, 0, SEEK_SET); if(lret != 0) while(1);
   lret = vfs_read(&file3, buffer, TEST_BUFFER_SIZE); if(lret != TEST_BUFFER_SIZE) while(1);
   ret = test_check_buffer(buffer, TEST_BUFFER_SIZE); if(ret < 0) while(1);
   ret = vfs_close(&file3); if(ret < 0) while(1);
   /*
   ret = vfs_open("/mount/usb/dir6/file9", &file7, VFS_O_CREAT); if(ret < 0) while(1);
   test_fill_buffer(buffer, TEST_BUFFER_SIZE);
   lret = vfs_write(&file7, buffer, TEST_BUFFER_SIZE); if(lret != TEST_BUFFER_SIZE) while(1);
   memset(buffer, 0, TEST_BUFFER_SIZE);
   lret = vfs_lseek(&file7, 0, SEEK_SET); if(lret != 0) while(1);
   lret = vfs_read(&file7, buffer, TEST_BUFFER_SIZE); if(lret != TEST_BUFFER_SIZE) while(1);
   ret = test_check_buffer(buffer, TEST_BUFFER_SIZE); if(ret < 0) while(1);
   ret = vfs_close(&file7); if(ret < 0) while(1);
   */
   gpioWrite( DO6, ON );
   //while(1);
}

static void test_fill_buffer(uint8_t *buffer, uint16_t size)
{
   uint16_t i;

   for(i=0; i<size; i++)
   {
      buffer[i] = i%10 + 0x30;   /* ASCII desde 0 a 9 */
   }
}

static int test_check_buffer(uint8_t *buffer, uint16_t size)
{
   uint16_t i;
   //printf( "test_assert_buffer(): Differences\n",
   //                  "buff\t\tseq\n");
   for(i=0; i<size; i++)
   {
      if(buffer[i] != i%10 + 0x30)
      {
         //printf("%c\t\t%c\n",buffer[i],i%10 + 0x30);
      }
   }
   if(i==size)
   {
   //printf( "test_assert_buffer(): OK, no differences\n");
   }
   return 0;
}

