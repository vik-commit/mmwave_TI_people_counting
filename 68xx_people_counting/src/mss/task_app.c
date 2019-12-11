/**
 *   @file  task_mbox.c
 *
 *   @brief
 *     MSS main implementation of the millimeter wave Demo
 *
 *  Copyright (C) 2017 Texas Instruments Incorporated - http://www.ti.com/
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**************************************************************************
 *************************** Include Files ********************************
 **************************************************************************/

/* Standard Include Files. */
#include <stdint.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <math.h>


/* BIOS/XDC Include Files. */
#include <xdc/std.h>
#include <xdc/cfg/global.h>
#include <xdc/runtime/IHeap.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Error.h>
#include <xdc/runtime/Memory.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/heaps/HeapBuf.h>
#include <ti/sysbios/heaps/HeapMem.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/family/arm/v7a/Pmu.h>
#include <ti/sysbios/family/arm/v7r/vim/Hwi.h>

/* mmWave SDK Include Files: */
#include <ti/common/sys_common.h>
#include <ti/drivers/soc/soc.h>
#include <ti/drivers/esm/esm.h>
#include <ti/drivers/crc/crc.h>
#include <ti/drivers/uart/UART.h>
#include <ti/drivers/gpio/gpio.h>
#include <ti/drivers/mailbox/mailbox.h>
#include <ti/control/mmwave/mmwave.h>
#include <ti/utils/cli/cli.h>
#include <ti/drivers/osal/MemoryP.h>
#include <ti/drivers/osal/DebugP.h>
#include <ti/drivers/osal/HwiP.h>
#include <ti/utils/cycleprofiler/cycle_profiler.h>

//#if (defined (GTRACK_2D)) || (defined (GTRACK_3D))
#include <gtrack.h>
//#else
//#include <ti/alg/gtrack/gtrack.h>
//#endif
#include "float.h"

/* Demo Include Files */
#include "mss_mmw.h"
#include <mmw_messages.h>
#include <mmw_output.h>

extern int32_t MmwDemo_mboxWrite(MmwDemo_message *message);
extern void MmwDemo_printHeapStats(void);

/**************************************************************************
 *************************** Local Definitions ****************************
 **************************************************************************/

/**************************************************************************
 *************************** Global Definitions ***************************
 **************************************************************************/

typedef enum {
    TRACKING_DEFAULT_PARAM_SET = 0,
    TRACKING_TRAFFIC_MONITORING_PARAM_SET,
    TRACKING_PEOPLE_COUNTING_PARAM_SET
} TRACKING_ADVANCED_PARAM_SET;

typedef enum {
    TRACKING_PARAM_SET_TM = 0,
    TRACKING_PARAM_SET_PC
} TRACKING_ADVANCED_PARAM_SET_TABLE;

/* This test application (traffic monitoring), wants to modify default parameters */
GTRACK_sceneryParams appSceneryParamTable[2] = {
	1,{{-1.f,12.f,15.f,75.f},{0.f,0.f,0.f,0.f}},1,{{0.f,11.f,19.f,50.f},{0.f,0.f,0.f,0.f}}, 	/* boundary box: {-1,12,15,75}, static box {0,11,19,50}, both in (left x, right x, bottom y, top y) format */
	0,{{0.f,0.f,0.f,0.f,-10.f,10.f},{0.f,0.f,0.f,0.f,-10.f,10.f}},0,{{0.f,0.f,0.f,0.f},{0.f,0.f,0.f,0.f}} 			/* no boundary boxes, static boxes */
};

#if (defined (GTRACK_2D)) || (defined (GTRACK_3D))
GTRACK_gatingParams appGatingParamTable[2] = {
    {16.f, {12.f, 8.f, 0.f, 0.f}},	/* TM: 16 gating volume, Limits are set to 8m in length, 2m in width, no limit in doppler */
	{2.f, {2.f, 2.f, 2.f, 0.f}}		/* PC: 2 gating volume, Limits are set to 2m in length, 2m in width, no limit in doppler */
};
#else
GTRACK_gatingParams appGatingParamTable[2] = {
    {16.f, {12.f, 8.f, 0.f}},	/* TM: 16 gating volume, Limits are set to 8m in length, 2m in width, no limit in doppler */
	{2.f, {2.f, 2.f, 2.f, 0.f}}		/* PC: 2 gating volume, Limits are set to 2m in length, 2m in width, no limit in doppler */
};
#endif
GTRACK_stateParams appStateParamTable[2] = {
	{3U, 3U, 5U, 5U, 5U},       /* TM: 3 frames to activate, 3 to forget, 5 to delete irrespective */
	{10U, 5U, 10U, 100U, 5U}    /* PC: det2act, det2free, act2free, stat2free, exit2free */
};
GTRACK_allocationParams appAllocationParamTable[2] = {
	{-1.f, -1.f, 1.f, 3U, 4.f, 2.f},  	/* TM: any SNRs, 1m/s minimal velocity, 3 points with 4m in distance, 2m/c in velocity  separation */
	{150.f, 250.f, 0.1f, 5U, 1.f, 2.f}	/* PC: SNRs 150 (250 obscured), 0.1 m/s minimal velocity, 5 points, with 1m in distance, 2m/c in velocity in separation */
};
/* Using standard deviation of uniformly distributed variable in the range [a b]: 1/sqrt(12)*(b-a) */
GTRACK_varParams appVariationParamTable[2] = {
     /* Standard deviation of uniformly distributed number in range [a b]: sqrt(1/12)*(b-a) */
	 {1.f/3.46f, 1.f/3.46f, 1.f},	/* TM: 1m height, 1m in width, 2 m/s for doppler */
	 {1.f/3.46f, 1.f/3.46f, 1.f}	/* PC: 1m height, 1m in width, 1 m/s for doppler */
};

/**
 * @brief
 *  Global Variable for tracking information required by the mmw Demo
 */
extern MmwDemo_MCB    gMmwMssMCB;
float maxAccelerationParams[3] = {0.1, 0.1, 0.1};

/**
 *  @b Description
 *  @n
 *      The Task is used to handle  the mmw demo messages received from
 *      Mailbox virtual channel.
 *
 *  @param[in]  arg0
 *      arg0 of the Task. Not used
 *  @param[in]  arg1
 *      arg1 of the Task. Not used
 *
 *  @retval
 *      Not Applicable.
 */
void MmwDemo_appTask(UArg arg0, UArg arg1)
{
    MmwDemo_ApplicationCfg appConfig;
    MmwDemo_output_message_targetList *targetList;
    MmwDemo_output_message_targetIndex *targetIndex;
    GTRACK_targetDesc targetDescr[20];


	GTRACK_measurementPoint *points;
#if (!defined (GTRACK_2D)) && (!defined (GTRACK_3D))
    GTRACK_measurementVariance *variances;
#endif
    uint32_t timeStart;
    uint32_t    *benchmarks;
    uint16_t    mNum;
    uint16_t    tNum;
    uint16_t    n;
    _Bool currentDescr;

    memset ((void *)&appConfig, 0, sizeof(MmwDemo_ApplicationCfg));
    appConfig.sensorAzimuthTilt = 0;
    /* Save Configuration to use later */
    memcpy((void *)&gMmwMssMCB.cfg.applicationCfg, (void *)&appConfig, sizeof(MmwDemo_ApplicationCfg));

	benchmarks = gMmwMssMCB.mssDataPathObj.cycleLog.benchmarks;
    /* wait for new message and process all the messages received from the peer */
    while(1)
    {
        Semaphore_pend(gMmwMssMCB.appSemHandle, BIOS_WAIT_FOREVER);

        timeStart = Cycleprofiler_getTimeStamp();

        if(gMmwMssMCB.pointCloud->header.length > sizeof(MmwDemo_output_message_tl))
            mNum = (gMmwMssMCB.pointCloud->header.length-sizeof(MmwDemo_output_message_tl))/sizeof(MmwDemo_output_message_point);
        else
            mNum = 0;

        currentDescr = gMmwMssMCB.targetDescrHandle->currentDescr;
        targetList = gMmwMssMCB.targetDescrHandle->tList[currentDescr];
        targetIndex = gMmwMssMCB.targetDescrHandle->tIndex[currentDescr];
        points = (GTRACK_measurementPoint *)gMmwMssMCB.pointCloud->point;
#if (defined (GTRACK_2D)) || (defined (GTRACK_3D))
			gtrack_step(gMmwMssMCB.gtrackHandle, points, NULL, mNum, targetDescr, &tNum, targetIndex->index, benchmarks);
#else
			variances = NULL;
			gtrack_step(gMmwMssMCB.gtrackHandle, points, variances, mNum, targetDescr, &tNum, targetIndex->index, benchmarks);
#endif

        for(n=0; n<tNum; n++) {
            targetList->target[n].tid  = (uint32_t)targetDescr[n].uid;

            targetList->target[n].posX = targetDescr[n].S[0];
            targetList->target[n].posY = targetDescr[n].S[1];
            targetList->target[n].velX = targetDescr[n].S[2];
            targetList->target[n].velY = targetDescr[n].S[3];
            targetList->target[n].accX = targetDescr[n].S[4];
            targetList->target[n].accY = targetDescr[n].S[5];

            memcpy(targetList->target[n].ec, targetDescr[n].EC, sizeof(targetDescr[n].EC));

            targetList->target[n].g = targetDescr[n].G;
        }

        if(tNum > 0)
            targetList->header.length = sizeof(MmwDemo_output_message_tl) + tNum*sizeof(MmwDemo_output_message_target);
        else
            targetList->header.length = 0;

        if((mNum > 0) && (tNum > 0))
            /* Target Indices exist only when we have both points AND targets */
            targetIndex->header.length = sizeof(MmwDemo_output_message_tl) + mNum*sizeof(uint8_t);
        else
            targetIndex->header.length = 0;

        gMmwMssMCB.mssDataPathObj.cycleLog.trackingTimeCurrInusec = ((float)(Cycleprofiler_getTimeStamp() - timeStart))/(float)R4F_CLOCK_MHZ;
        if ((gMmwMssMCB.mssDataPathObj.cycleLog.trackingTimeCurrInusec > 0) && (gMmwMssMCB.mssDataPathObj.cycleLog.trackingTimeCurrInusec > gMmwMssMCB.mssDataPathObj.cycleLog.trackingTimeMaxInusec))
            gMmwMssMCB.mssDataPathObj.cycleLog.trackingTimeMaxInusec = gMmwMssMCB.mssDataPathObj.cycleLog.trackingTimeCurrInusec;

	}
}

/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for tracking configuration
 *
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t MmwDemo_CLITrackingCfg (int32_t argc, char* argv[])
{
	int32_t 					errCode;	
    GTRACK_moduleConfig         config;
    GTRACK_advancedParameters   advParams;

    TRACKING_ADVANCED_PARAM_SET trackingParamSet;

    MmwDemo_message         message;
    uint32_t                pointCloudSize;
    uint32_t                targetListSize;
    uint32_t                targetIndexSize;

    //Memory_Stats            startMemoryStats;
    //Memory_Stats            endMemoryStats;

    if (argc >= 1) {
        gMmwMssMCB.mssDataPathObj.groupTrackerEnabled = (uint16_t) atoi (argv[1]);
    }

    if(gMmwMssMCB.mssDataPathObj.groupTrackerEnabled != 1) {
        return 0;
    }

    /* Sanity Check: Minimum argument check */
    if (argc != 9)
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }

    System_printf("Debug: Heap before creating a tracker\n");
    MmwDemo_printHeapStats();

    /* Initialize CLI configuration: */
    memset ((void *)&config, 0, sizeof(GTRACK_moduleConfig));

    trackingParamSet            = (TRACKING_ADVANCED_PARAM_SET) atoi (argv[2]);
    switch(trackingParamSet)
    {
        case TRACKING_DEFAULT_PARAM_SET:
            // Do not configure advanced parameters, use library default parameters
            config.advParams = 0;
            break;

        case TRACKING_TRAFFIC_MONITORING_PARAM_SET:
            /* Initialize CLI configuration: */
            memset ((void *)&advParams, 0, sizeof(GTRACK_advancedParameters));
            advParams.sceneryParams = &appSceneryParamTable[TRACKING_PARAM_SET_TM];
            advParams.allocationParams = &appAllocationParamTable[TRACKING_PARAM_SET_TM];
            advParams.gatingParams = &appGatingParamTable[TRACKING_PARAM_SET_TM];
            advParams.stateParams = &appStateParamTable[TRACKING_PARAM_SET_TM];
            advParams.variationParams = &appVariationParamTable[TRACKING_PARAM_SET_TM];

            config.advParams = &advParams;
            config.initialRadialVelocity = -8; 	// for TM, detected targets are approaching
#if (defined (GTRACK_2D)) || (defined (GTRACK_3D))
            config.maxAcceleration[0] = 2;        // for TM, maximum acceleration in lateral direction is set to 2m/s2
            config.maxAcceleration[1] = 20;       // for TM, maximum acceleration is longitudinal direction set to 20m/s2
            config.maxAcceleration[2] = 2;       // for TM, maximum acceleration is longitudinal direction set to 20m/s2
#else
            config.maxAccelerationX = 2;        // for TM, maximum acceleration in lateral direction is set to 2m/s2
            config.maxAccelerationY = 20;       // for TM, maximum acceleration is longitudinal direction set to 20m/s2
#endif			
            break;

        case TRACKING_PEOPLE_COUNTING_PARAM_SET:

            /* Initialize CLI configuration: */
            memset ((void *)&advParams, 0, sizeof(GTRACK_advancedParameters));
            advParams.sceneryParams = &appSceneryParamTable[TRACKING_PARAM_SET_PC];
            advParams.allocationParams = &appAllocationParamTable[TRACKING_PARAM_SET_PC];
            advParams.gatingParams = &appGatingParamTable[TRACKING_PARAM_SET_PC];
            advParams.stateParams = &appStateParamTable[TRACKING_PARAM_SET_PC];
            advParams.variationParams = &appVariationParamTable[TRACKING_PARAM_SET_PC];

            config.advParams = &advParams;
            config.initialRadialVelocity = 0; //For PC, detected target velocity is unknown
#if (defined (GTRACK_2D)) || (defined (GTRACK_3D))
            config.maxAcceleration[0] = 0.5;        // for PC, maximum acceleration in lateral direction is set to 5m/s2
            config.maxAcceleration[1] = 0.5;       // for PC, maximum acceleration is longitudinal direction set to 5m/s2
            config.maxAcceleration[2] = 0.5;       // for PC, maximum acceleration is vertical direction set to 5m/s2
#else
            config.maxAccelerationX = 5;        // for PC, maximum acceleration in lateral direction is set to 5m/s2
            config.maxAccelerationY = 5;       // for PC, maximum acceleration is longitudinal direction set to 5m/s2
#endif			
            break;

        default:
            CLI_write ("Error: Invalid usage of the CLI command\n");
            return -1;
    }

    config.stateVectorType      	= GTRACK_STATE_VECTORS_2DA; // Track two dimensions with acceleration
    config.verbose              	= GTRACK_VERBOSE_NONE;
    config.maxNumPoints         	= (uint16_t) atoi(argv[3]);
    config.maxNumTracks         	= (uint16_t) atoi(argv[4]);
    config.maxRadialVelocity   		= (float) atoi(argv[5]) *0.1f;
    config.radialVelocityResolution	= (float) atoi(argv[6]) *0.001f;
    config.deltaT               	= (float) atoi(argv[7]) *0.001f;

    gMmwMssMCB.cfg.applicationCfg.sensorAzimuthTilt = (90-atoi(argv[8]))*3.14f/180;
    gMmwMssMCB.mssDataPathObj.maxNumTracks = config.maxNumTracks;

    if(gMmwMssMCB.cfg.trackingCfg.config.maxNumPoints != 0) {
        pointCloudSize = sizeof(MmwDemo_output_message_tl) + gMmwMssMCB.cfg.trackingCfg.config.maxNumPoints*sizeof(GTRACK_measurementPoint);
        if(gMmwMssMCB.pointCloud != NULL) {
            MemoryP_ctrlFree(gMmwMssMCB.pointCloud, pointCloudSize);
        }
    }
    if(gMmwMssMCB.cfg.trackingCfg.config.maxNumTracks != 0) {
        targetListSize = sizeof(MmwDemo_output_message_tl) + gMmwMssMCB.cfg.trackingCfg.config.maxNumTracks*sizeof(GTRACK_targetDesc);
        targetIndexSize = sizeof(MmwDemo_output_message_tl) + gMmwMssMCB.cfg.trackingCfg.config.maxNumPoints*sizeof(uint8_t);
        if(gMmwMssMCB.targetDescrHandle != NULL) {
            /* Free Target List Arrays */
            if(gMmwMssMCB.targetDescrHandle->tList[0] != NULL)
                MemoryP_ctrlFree(gMmwMssMCB.targetDescrHandle->tList[0], targetListSize);
            if(gMmwMssMCB.targetDescrHandle->tList[1] != NULL)
                MemoryP_ctrlFree(gMmwMssMCB.targetDescrHandle->tList[1], targetListSize);

            /* Free Target Index Arrays */
            if(gMmwMssMCB.targetDescrHandle->tIndex[0] != NULL)
                MemoryP_ctrlFree(gMmwMssMCB.targetDescrHandle->tIndex[0], targetIndexSize);
            if(gMmwMssMCB.targetDescrHandle->tIndex[1] != NULL)
                MemoryP_ctrlFree(gMmwMssMCB.targetDescrHandle->tIndex[1], targetIndexSize);
        }
    }
    if(gMmwMssMCB.targetDescrHandle != NULL) {
        MemoryP_ctrlFree(gMmwMssMCB.targetDescrHandle, sizeof(MmwDemo_targetDescrHandle));
    }

    /* Save Configuration to use later */
    memcpy((void *)&gMmwMssMCB.cfg.trackingCfg.config, (void *)&config, sizeof(GTRACK_moduleConfig));
    memcpy((void *)&gMmwMssMCB.cfg.trackingCfg.params, (void *)&advParams, sizeof(GTRACK_advancedParameters));

    //HeapMem_getStats (heap0, &startMemoryStats);
    //MmwDemo_printHeapStats();

    /* Allocate memory for Point Cloud TLV */
    pointCloudSize = sizeof(MmwDemo_output_message_tl) + config.maxNumPoints*sizeof(GTRACK_measurementPoint);
    gMmwMssMCB.pointCloud = (MmwDemo_output_message_pointCloud *)MemoryP_ctrlAlloc(pointCloudSize, sizeof(float));
    gMmwMssMCB.pointCloud->header.type = MMWDEMO_OUTPUT_MSG_POINT_CLOUD;

    if(gMmwMssMCB.pointCloud == NULL) {
        System_printf("Error: Unable to allocate %d bytes for pointCloud\n", config.maxNumPoints*sizeof(GTRACK_measurementPoint));
        DebugP_assert(0);
    }

    /* Allocate memory for Target Descriptor handle */
    gMmwMssMCB.targetDescrHandle = (MmwDemo_targetDescrHandle *)MemoryP_ctrlAlloc(sizeof(MmwDemo_targetDescrHandle), sizeof(float));
    if(gMmwMssMCB.targetDescrHandle == NULL) {
        System_printf("Error: Unable to allocate %d bytes for targetDescr handle\n", sizeof(MmwDemo_targetDescrHandle));
        DebugP_assert(0);
    }
    memset ((void *)gMmwMssMCB.targetDescrHandle, 0, sizeof(MmwDemo_targetDescrHandle));

    /* Allocate memory for ping/pong target lists */
    targetListSize = sizeof(MmwDemo_output_message_tl) + config.maxNumTracks*sizeof(GTRACK_targetDesc);
    gMmwMssMCB.targetDescrHandle->tList[0] = (MmwDemo_output_message_targetList *)MemoryP_ctrlAlloc(targetListSize, sizeof(float));
    gMmwMssMCB.targetDescrHandle->tList[1] = (MmwDemo_output_message_targetList *)MemoryP_ctrlAlloc(targetListSize, sizeof(float));

    if((gMmwMssMCB.targetDescrHandle->tList[0] == NULL) || (gMmwMssMCB.targetDescrHandle->tList[1] == NULL)){
        System_printf("Error: Unable to allocate %d bytes for targetLists\n", targetListSize*2);
        DebugP_assert(0);
    }

    gMmwMssMCB.targetDescrHandle->tList[0]->header.type = MMWDEMO_OUTPUT_MSG_TARGET_LIST;
    gMmwMssMCB.targetDescrHandle->tList[0]->header.length = 0;
    gMmwMssMCB.targetDescrHandle->tList[1]->header.type = MMWDEMO_OUTPUT_MSG_TARGET_LIST;
    gMmwMssMCB.targetDescrHandle->tList[1]->header.length = 0;

    /* Allocate memory for ping/pong target indices */
    targetIndexSize = sizeof(MmwDemo_output_message_tl) + config.maxNumPoints*sizeof(uint8_t);
    gMmwMssMCB.targetDescrHandle->tIndex[0] = (MmwDemo_output_message_targetIndex *)MemoryP_ctrlAlloc(targetIndexSize, sizeof(float));
    gMmwMssMCB.targetDescrHandle->tIndex[1] = (MmwDemo_output_message_targetIndex *)MemoryP_ctrlAlloc(targetIndexSize, sizeof(float));

    if((gMmwMssMCB.targetDescrHandle->tIndex[0] == NULL) || (gMmwMssMCB.targetDescrHandle->tIndex[1] == NULL)){
        System_printf("Error: Unable to allocate %d bytes for targetIndices\n", targetIndexSize*2);
        DebugP_assert(0);
    }

    gMmwMssMCB.targetDescrHandle->tIndex[0]->header.type = MMWDEMO_OUTPUT_MSG_TARGET_INDEX;
    gMmwMssMCB.targetDescrHandle->tIndex[0]->header.length = 0;
    gMmwMssMCB.targetDescrHandle->tIndex[1]->header.type = MMWDEMO_OUTPUT_MSG_TARGET_INDEX;
    gMmwMssMCB.targetDescrHandle->tIndex[1]->header.length = 0;

    //MmwDemo_printHeapStats();

    /* Create a Tracker */
    if(gMmwMssMCB.gtrackHandle != NULL)
        gtrack_delete(gMmwMssMCB.gtrackHandle);

    gMmwMssMCB.gtrackHandle = gtrack_create(&config, &errCode);
    if(gMmwMssMCB.gtrackHandle == NULL) {
        System_printf("Error: Unable to allocate memory for Tracker\n");
        DebugP_assert(0);
    }
    System_printf("Debug: (GtrackModuleInstance *)0x%x\n", (uint32_t)gMmwMssMCB.gtrackHandle);
    MmwDemo_printHeapStats();

    /* Get the heap statistics at the beginning of the tests */
    //HeapMem_getStats (heap0, &endMemoryStats);
    // System_printf ("Debug: Tracker %d used bytes from System Heap\n", startMemoryStats.totalFreeSize - endMemoryStats.totalFreeSize);

    /* Send configuration to DSS */
    memset((void *)&message, 0, sizeof(MmwDemo_message));

    message.type = MMWDEMO_MSS2DSS_TRACKING_CFG;
    memcpy((void *)&message.body.tracking, (void *)&config, sizeof(GTRACK_moduleConfig));

    if (MmwDemo_mboxWrite(&message) == 0)
        return 0;
    else
        return -1;
}

 int32_t MmwDemo_CLISceneryParamCfg (int32_t argc, char* argv[])
 {


     /* Sanity Check: Minimum argument check */
     if (argc != 5)
     {
         CLI_write ("Error: Invalid usage of the CLI command\n");
         return -1;
     }
    
     /* Initialize the ADC Output configuration: */
     memset ((void *)&appSceneryParamTable, 0, sizeof(appSceneryParamTable));

    
     /* Populate configuration: */
     appSceneryParamTable[TRACKING_PARAM_SET_PC].numBoundaryBoxes = 1;
     appSceneryParamTable[TRACKING_PARAM_SET_PC].boundaryBox[0].x1 = (float) atof (argv[1]);
     appSceneryParamTable[TRACKING_PARAM_SET_PC].boundaryBox[0].x2 = (float) atof (argv[2]);
     appSceneryParamTable[TRACKING_PARAM_SET_PC].boundaryBox[0].y1 = (float) atof (argv[3]);
     appSceneryParamTable[TRACKING_PARAM_SET_PC].boundaryBox[0].y2 = (float) atof (argv[4]);

     appSceneryParamTable[TRACKING_PARAM_SET_PC].staticBox[0].x1 = (float) atof (argv[1]);
     appSceneryParamTable[TRACKING_PARAM_SET_PC].staticBox[0].x2 = (float) atof (argv[2]);
     appSceneryParamTable[TRACKING_PARAM_SET_PC].staticBox[0].y1 = (float) atof (argv[3]);
     appSceneryParamTable[TRACKING_PARAM_SET_PC].staticBox[0].y2 = (float) atof (argv[4]);
   //  appSceneryParamTable[1].boundaryBox[1].z1 = (int16_t) atoi (argv[5]);
  //   appSceneryParamTable[1].boundaryBox[1].z1 = (int16_t) atoi (argv[6]);

     return 0;
    
 }



 /**
  *  @b Description
  *  @n
  *      This is the CLI Handler for GatingParam configuration
  *
  *  @param[in] argc
  *      Number of arguments
  *  @param[in] argv
  *      Arguments
  *
  *  @retval
  *      Success -   0
  *  @retval
  *      Error   -   <0
  */
 int32_t MmwDemo_CLIGatingParamCfg (int32_t argc, char* argv[])
 {


     /* Sanity Check: Minimum argument check */
     if (argc != 5)
     {
         CLI_write ("Error: Invalid usage of the CLI command\n");
         return -1;
     }
     /* Initialize the ADC Output configuration: */
     memset ((void *)&appGatingParamTable, 0, sizeof(appGatingParamTable));
 //
 //  appGatingParamTable[0].volume = (float) atof (argv[1]);
 //  appGatingParamTable[0].limits.depth = (float) atof (argv[2]);
 //  appGatingParamTable[0].limits.width = (float) atof (argv[3]);
 //  appGatingParamTable[0].limits.vel = (float) atof (argv[4]);
    
     appGatingParamTable[TRACKING_PARAM_SET_PC].gain = (float) atof (argv[1]);
     appGatingParamTable[TRACKING_PARAM_SET_PC].limits.width = (float) atof (argv[2]);
     appGatingParamTable[TRACKING_PARAM_SET_PC].limits.depth = (float) atof (argv[3]);
     appGatingParamTable[TRACKING_PARAM_SET_PC].limits.vel = (float) atof (argv[4]);
    


     return 0;
    
 }


 /**
  *  @b Description
  *  @n
  *      This is the CLI Handler for StateParam configuration
  *
  *  @param[in] argc
  *      Number of arguments
  *  @param[in] argv
  *      Arguments
  *
  *  @retval
  *      Success -   0
  *  @retval
  *      Error   -   <0
  */
 int32_t MmwDemo_CLIStateParamCfg (int32_t argc, char* argv[])
 {


     /* Sanity Check: Minimum argument check */
     if (argc != 6)
     {
         CLI_write ("Error: Invalid usage of the CLI command\n");
         return -1;
     }

     /* Initialize the ADC Output configuration: */
     memset ((void *)&appStateParamTable, 0, sizeof(appStateParamTable));

    
     /* Populate configuration: */
     appStateParamTable[TRACKING_PARAM_SET_PC].det2actThre = (uint16_t) atoi (argv[1]);
     appStateParamTable[TRACKING_PARAM_SET_PC].det2freeThre= (uint16_t) atoi (argv[2]);
     appStateParamTable[TRACKING_PARAM_SET_PC].active2freeThre= (uint16_t) atoi (argv[3]);
     appStateParamTable[TRACKING_PARAM_SET_PC].static2freeThre= (uint16_t) atoi (argv[4]);
     appStateParamTable[TRACKING_PARAM_SET_PC].exit2freeThre= (uint16_t) atoi (argv[5]);

 //  appStateParamTable[0].det2actThre = (uint16_t) atoi (argv[1]);
 //    appStateParamTable[0].det2freeThre= (uint16_t) atoi (argv[2]);
 //    appStateParamTable[0].active2freeThre= (uint16_t) atoi (argv[3]);
 //    appStateParamTable[0].static2freeThre= (uint16_t) atoi (argv[4]);
 //    appStateParamTable[0].exit2freeThre= (uint16_t) atoi (argv[5]);

     return 0;
    
 }

 /**
  *  @b Description
  *  @n
  *      This is the CLI Handler for AllocationParam configuration
  *
  *  @param[in] argc
  *      Number of arguments
  *  @param[in] argv
  *      Arguments
  *
  *  @retval
  *      Success -   0
  *  @retval
  *      Error   -   <0
  */
 int32_t MmwDemo_CLIAllocationParamCfg (int32_t argc, char* argv[])
 {


     /* Sanity Check: Minimum argument check */
     if (argc != 7)
     {
         CLI_write ("Error: Invalid usage of the CLI command\n");
         return -1;
     }

     /* Initialize the ADC Output configuration: */
     memset ((void *)&appAllocationParamTable, 0, sizeof(appAllocationParamTable));

    
     /* Populate configuration: */
 //  appAllocationParamTable[0].snrThre = (float) atof (argv[1]);
 //  appAllocationParamTable[0].velocityThre = (float) atof (argv[2]);
 //  appAllocationParamTable[0].pointsThre = (uint16_t) atoi (argv[3]);
 //  appAllocationParamTable[0].maxDistanceThre = (float) atof (argv[4]);
 //  appAllocationParamTable[0].maxVelThre = (float) atof (argv[5]);
    
     appAllocationParamTable[TRACKING_PARAM_SET_PC].snrThre = (float) atof (argv[1]);
     appAllocationParamTable[TRACKING_PARAM_SET_PC].snrThreObscured = (float) atof (argv[2]);
     appAllocationParamTable[TRACKING_PARAM_SET_PC].velocityThre = (float) atof (argv[3]);
     appAllocationParamTable[TRACKING_PARAM_SET_PC].pointsThre = (uint16_t) atoi (argv[4]);
     appAllocationParamTable[TRACKING_PARAM_SET_PC].maxDistanceThre = (float) atof (argv[5]);
     appAllocationParamTable[TRACKING_PARAM_SET_PC].maxVelThre = (float) atof (argv[6]);

     return 0;
    
 }

 /**
  *  @b Description
  *  @n
  *      This is the CLI Handler for VariationParam configuration
  *
  *  @param[in] argc
  *      Number of arguments
  *  @param[in] argv
  *      Arguments
  *
  *  @retval
  *      Success -   0
  *  @retval
  *      Error   -   <0
  */
 int32_t MmwDemo_CLIVariationParamCfg (int32_t argc, char* argv[])
 {


     /* Sanity Check: Minimum argument check */
     if (argc != 4)
     {
         CLI_write ("Error: Invalid usage of the CLI command\n");
         return -1;
     }

     /* Initialize the ADC Output configuration: */
     memset ((void *)&appVariationParamTable, 0, sizeof(appVariationParamTable));

    
     /* Populate configuration: */
 //  appVariationParamTable[0].depthStd = (float) atof (argv[1]);
 //  appVariationParamTable[0].widthStd = (float) atof (argv[2]);
 //  appVariationParamTable[0].dopplerStd = (float) atof (argv[3]);

     appVariationParamTable[TRACKING_PARAM_SET_PC].widthStd = (float) atof (argv[1]);
     appVariationParamTable[TRACKING_PARAM_SET_PC].depthStd = (float) atof (argv[2]);
     appVariationParamTable[TRACKING_PARAM_SET_PC].dopplerStd = (float) atof (argv[3]);

     return 0;
    
 }

 int32_t MmwDemoCLIMaxAccelerationParamCfg(int32_t argc, char* argv[])
 {
     if (argc != 4 || argc != 3)
     {
         CLI_write ("Error: Invalid usage of the CLI command\n");
         return -1;
     } else if (argc == 4) {
         maxAccelerationParams[0] = (float) atof (argv[1]);
         maxAccelerationParams[1] = (float) atof (argv[2]);
         maxAccelerationParams[2] = (float) atof (argv[3]);
     } else {
         maxAccelerationParams[0] = (float) atof (argv[1]);
         maxAccelerationParams[1] = (float) atof (argv[2]);
     }

     return 0;

 }

 int32_t MmwDemo_CLIPointCloudEn (int32_t argc, char* argv[]) {
     if (argc != 2)
     {
         CLI_write ("Error: Invalid usage of PointCloudEn command\n");
         return -1;
     }
     gMmwMssMCB.pcEnable = (bool)atoi(argv[1]);
     return 0;
 }
/*reconfigure device without pausing - not available for all params, but works for these*/
 int32_t LiveCFG_DOA (int32_t argc, char* argv[]) {
    return 0;
 }

 int32_t LiveCFG_Allocation (int32_t argc, char* argv[]) {
    /* Sanity Check: Minimum argument check */
     if (argc != 7)
     {
         CLI_write ("Error: Invalid usage of the CLI command\n");
         return -1;
     }

     /* Initialize the ADC Output configuration: */
     memset ((void *)&appAllocationParamTable, 0, sizeof(appAllocationParamTable));

     /* Populate configuration: */  
     appAllocationParamTable[TRACKING_PARAM_SET_PC].snrThre = (float) atof (argv[1]);
     appAllocationParamTable[TRACKING_PARAM_SET_PC].snrThreObscured = (float) atof (argv[2]);
     appAllocationParamTable[TRACKING_PARAM_SET_PC].velocityThre = (float) atof (argv[3]);
     appAllocationParamTable[TRACKING_PARAM_SET_PC].pointsThre = (uint16_t) atoi (argv[4]);
     appAllocationParamTable[TRACKING_PARAM_SET_PC].maxDistanceThre = (float) atof (argv[5]);
     appAllocationParamTable[TRACKING_PARAM_SET_PC].maxVelThre = (float) atof (argv[6]);

     gtrack_setAllocationParams(gMmwMssMCB.gtrackHandle, &appAllocationParamTable[TRACKING_PARAM_SET_PC]);
     return 0;
 }

 int32_t LiveCFG_Gating (int32_t argc, char* argv[]) {
    /* Sanity Check: Minimum argument check */
     if (argc != 5)
     {
         CLI_write ("Error: Invalid usage of the CLI command\n");
         return -1;
     }
     /* Initialize the ADC Output configuration: */
     memset ((void *)&appGatingParamTable, 0, sizeof(appGatingParamTable));
    
     appGatingParamTable[TRACKING_PARAM_SET_PC].gain = (float) atof (argv[1]);
     appGatingParamTable[TRACKING_PARAM_SET_PC].limits.width = (float) atof (argv[2]);
     appGatingParamTable[TRACKING_PARAM_SET_PC].limits.depth = (float) atof (argv[3]);
     appGatingParamTable[TRACKING_PARAM_SET_PC].limits.vel = (float) atof (argv[4]);

     gtrack_setGatingParams(gMmwMssMCB.gtrackHandle, &appGatingParamTable[TRACKING_PARAM_SET_PC]);
     return 0;
 }

 int32_t LiveCFG_State (int32_t argc, char* argv[]) {
    /* Sanity Check: Minimum argument check */
     if (argc != 6)
     {
         CLI_write ("Error: Invalid usage of the CLI command\n");
         return -1;
     }

     /* Initialize the ADC Output configuration: */
     memset ((void *)&appStateParamTable, 0, sizeof(appStateParamTable));

    
     /* Populate configuration: */
     appStateParamTable[TRACKING_PARAM_SET_PC].det2actThre = (uint16_t) atoi (argv[1]);
     appStateParamTable[TRACKING_PARAM_SET_PC].det2freeThre= (uint16_t) atoi (argv[2]);
     appStateParamTable[TRACKING_PARAM_SET_PC].active2freeThre= (uint16_t) atoi (argv[3]);
     appStateParamTable[TRACKING_PARAM_SET_PC].static2freeThre= (uint16_t) atoi (argv[4]);
     appStateParamTable[TRACKING_PARAM_SET_PC].exit2freeThre= (uint16_t) atoi (argv[5]);

     gtrack_setStateParams(gMmwMssMCB.gtrackHandle, &appStateParamTable[TRACKING_PARAM_SET_PC]);
     return 0;
 }

 int32_t LiveCFG_Scenery (int32_t argc, char* argv[]) {
     /* Sanity Check: Minimum argument check */
     if (argc != 5)
     {
         CLI_write ("Error: Invalid usage of the CLI command\n");
         return -1;
     }
    
     /* Initialize the ADC Output configuration: */
     memset ((void *)&appSceneryParamTable, 0, sizeof(appSceneryParamTable));

    
     /* Populate configuration: */
     appSceneryParamTable[TRACKING_PARAM_SET_PC].numBoundaryBoxes = 1;
     appSceneryParamTable[TRACKING_PARAM_SET_PC].boundaryBox[0].x1 = (float) atof (argv[1]);
     appSceneryParamTable[TRACKING_PARAM_SET_PC].boundaryBox[0].x2 = (float) atof (argv[2]);
     appSceneryParamTable[TRACKING_PARAM_SET_PC].boundaryBox[0].y1 = (float) atof (argv[3]);
     appSceneryParamTable[TRACKING_PARAM_SET_PC].boundaryBox[0].y2 = (float) atof (argv[4]);

     appSceneryParamTable[TRACKING_PARAM_SET_PC].staticBox[0].x1 = (float) atof (argv[1]);
     appSceneryParamTable[TRACKING_PARAM_SET_PC].staticBox[0].x2 = (float) atof (argv[2]);
     appSceneryParamTable[TRACKING_PARAM_SET_PC].staticBox[0].y1 = (float) atof (argv[3]);
     appSceneryParamTable[TRACKING_PARAM_SET_PC].staticBox[0].y2 = (float) atof (argv[4]);

     gtrack_setSceneryParams(gMmwMssMCB.gtrackHandle, &appSceneryParamTable[TRACKING_PARAM_SET_PC]);
     return 0;
 }
