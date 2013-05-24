//*****************************************************************************
//
// os.cxx - The OS specific functions.
//
// Copyright (c) 2009-2012 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
// 
// Texas Instruments (TI) is supplying this software for use solely and
// exclusively on TI's microcontroller products. The software is owned by
// TI and/or its suppliers, and is protected under applicable copyright
// laws. You may not combine this software with "viral" open-source
// software in order to form a larger program.
// 
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
// 
// This is part of revision 9107 of the Stellaris Firmware Development Package.
//
//*****************************************************************************

#include "cart_sensors/cart_sensors.h"

//*****************************************************************************
//
// This function is used to create a thread for the application.
//
//*****************************************************************************
void
OSThreadCreate(void *(WorkerThread)(void *pvData))
{
#ifdef __WIN32
    //
    // Create the requested thread.
    //
    _beginthread((void (*)(void *))WorkerThread, 0, 0);
#else
    pthread_t thread;

    //
    // Create the requested thread.
    //
    pthread_create(&thread, 0, WorkerThread, 0);
#endif
}

//*****************************************************************************
//
// This function is used to kill a thread for the application.
//
//*****************************************************************************
void
OSThreadExit(void)
{
#ifdef __WIN32
    _endthread();
#else
    pthread_exit(0);
#endif
}

//*****************************************************************************
//
// This function is used to sleep for a given number of seconds.
//
//*****************************************************************************
void
OSSleep(uint32_t ulSeconds)
{
#ifdef __WIN32
    Sleep(ulSeconds * 1000);
#else
    sleep(ulSeconds);
#endif
}


//*****************************************************************************
//
// The global mutex used to protect the counter used by threads.
//
//*****************************************************************************
MUTEX mMutex;


//*****************************************************************************
//
// This function is used to create the MUTEX used by the application.
//
//*****************************************************************************
int MutexInit(MUTEX *mutex) {
#ifdef __WIN32
	//
	// Use the Windows call to create a mutex.
	//
	*mutex = CreateMutex(0, FALSE, 0);

	return(*mutex == 0);
#else
	return (pthread_mutex_init(mutex, NULL));
#endif
}

//*****************************************************************************
//
// Take the mutex and wait forever until it is released.
//
//*****************************************************************************
int MutexLock(MUTEX *mutex) {
	//
	// Wait for the mutex to be released.
	//
#ifdef __WIN32
	if(WaitForSingleObject(*mutex, INFINITE) == WAIT_FAILED)
	{
		return(1);
	}
	else
	{
		return(0);
	}
#else
	return (pthread_mutex_lock(mutex));
#endif
}

//*****************************************************************************
//
// Release the mutex.
//
//*****************************************************************************
int MutexUnlock(MUTEX *mutex) {
	//
	// Release the mutex.
	//
#ifdef __WIN32
	return(ReleaseMutex(*mutex) == 0);
#else
	return (pthread_mutex_unlock(mutex));
#endif
}

