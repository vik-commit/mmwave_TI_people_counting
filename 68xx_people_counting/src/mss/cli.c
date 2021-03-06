/*
 *   @file  cli.c
 *
 *   @brief
 *      Mmw (Milli-meter wave) DEMO CLI Implementation
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

/* mmWave SDK Include Files: */
#include <ti/common/sys_common.h>
#include <ti/drivers/uart/UART.h>
#include <ti/drivers/osal/MemoryP.h>
#include <ti/drivers/osal/DebugP.h>
#include <ti/control/mmwavelink/mmwavelink.h>
#include <ti/utils/cli/cli.h>

#include <gtrack.h>

/* Demo Include Files */
#include "mss_mmw.h"
#include <mmw_messages.h>

/**************************************************************************
 *************************** Local Definitions ****************************
 **************************************************************************/

/* CLI Extended Command Functions */
static int32_t MmwDemo_CLICfarCfg (int32_t argc, char* argv[]);
static int32_t MmwDemo_CLIDoACfg (int32_t argc, char* argv[]);
static int32_t MmwDemo_CLISensorStart (int32_t argc, char* argv[]);
static int32_t MmwDemo_CLIFrameStart (int32_t argc, char* argv[]);
static int32_t MmwDemo_CLISensorStop (int32_t argc, char* argv[]);
static int32_t MmwDemo_CLIGuiMonSel (int32_t argc, char* argv[]);
static int32_t MmwDemo_CLISetDataLogger (int32_t argc, char* argv[]);
static int32_t MmwDemo_CLIADCBufCfg (int32_t argc, char* argv[]);
static int32_t MmwDemo_CLICompRangeBiasAndRxChanPhaseCfg (int32_t argc, char* argv[]);

/* Tracking configuration is done by application */
extern int32_t MmwDemo_CLITrackingCfg (int32_t argc, char* argv[]);

/* SceneryParam configuration is done by application */
 extern int32_t MmwDemo_CLISceneryParamCfg (int32_t argc, char* argv[]);
 /* GatingParam configuration is done by application */
 extern int32_t MmwDemo_CLIGatingParamCfg (int32_t argc, char* argv[]);
 /* StateParam configuration is done by application */
 extern int32_t MmwDemo_CLIStateParamCfg (int32_t argc, char* argv[]);
 /* AllocationParam configuration is done by application */
 extern int32_t MmwDemo_CLIAllocationParamCfg (int32_t argc, char* argv[]);
 /* VariationParam configuration is done by application */
 extern int32_t MmwDemo_CLIVariationParamCfg (int32_t argc, char* argv[]);
 /*Max acceleration set in task_app.c*/
 extern int32_t MmwDemoCLIMaxAccelerationParamCfg(int32_t arc, char* argv[]);

 /* Point Cloud enable */
 extern int32_t MmwDemo_CLIPointCloudEn (int32_t argc, char* argv[]);

/* Live config commands:
* CFAR - only thresholds
* DOA - only 1 param for now
* Allocation
* Gating
* State
*/
 //extern int32_t LiveCFG_CFAR (int32_t argc, char* argv[]);
 extern int32_t LiveCFG_DOA (int32_t argc, char* argv[]);
 extern int32_t LiveCFG_Allocation (int32_t argc, char* argv[]);
 extern int32_t LiveCFG_Gating (int32_t argc, char* argv[]);
 extern int32_t LiveCFG_State (int32_t argc, char* argv[]);
 extern int32_t LiveCFG_Scenery (int32_t argc, char* argv[]);

/**************************************************************************
 *************************** External Definitions *************************
 **************************************************************************/

extern MmwDemo_MCB    gMmwMssMCB;
extern int32_t MmwDemo_mboxWrite(MmwDemo_message     * message);
extern void MmwDemo_printHeapStats(void);

/**************************************************************************
 *************************** CLI  Function Definitions **************************
 **************************************************************************/

/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for the sensor start command
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
static int32_t MmwDemo_CLISensorStart (int32_t argc, char* argv[])
{
    gMmwMssMCB.stats.cliSensorStartEvt ++;

    /* Get the configuration from the CLI mmWave Extension */
    CLI_getMMWaveExtensionConfig (&gMmwMssMCB.cfg.ctrlCfg);

    /* Get the open configuration from the CLI mmWave Extension */
    CLI_getMMWaveExtensionOpenConfig (&gMmwMssMCB.cfg.openCfg);

    /* Post sensorSTart event to notify configuration is done */
    Event_post(gMmwMssMCB.eventHandle, MMWDEMO_CLI_SENSORSTART_EVT);
    return 0;
}

/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for the frame start command
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
static int32_t MmwDemo_CLIFrameStart (int32_t argc, char* argv[])
{
    gMmwMssMCB.stats.cliFrameStartEvt ++;

    /* Post sensorSTart event to notify configuration is done */
    Event_post(gMmwMssMCB.eventHandle, MMWDEMO_CLI_FRAMESTART_EVT);
    return 0;
}


/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for the sensor stop command
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
static int32_t MmwDemo_CLISensorStop (int32_t argc, char* argv[])
{
    gMmwMssMCB.stats.cliSensorStopEvt ++;

    /* Post sensorSTOP event to notify sensor stop command */
    Event_post(gMmwMssMCB.eventHandle, MMWDEMO_CLI_SENSORSTOP_EVT);

    return 0;
}

/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for gui monitoring configuration
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
static int32_t MmwDemo_CLIGuiMonSel (int32_t argc, char* argv[])
{
    MmwDemo_GuiMonSel   guiMonSel;
    MmwDemo_message     message;

    /* Sanity Check: Minimum argument check */
    if (argc != 5)
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }

    /* Initialize the ADC Output configuration: */
    memset ((void *)&guiMonSel, 0, sizeof(MmwDemo_GuiMonSel));

    /* Populate configuration: */
    guiMonSel.detectedObjects           = atoi (argv[1]);
    guiMonSel.logMagRange               = atoi (argv[2]);
    guiMonSel.rangeAzimuthHeatMap       = atoi (argv[3]);
    guiMonSel.rangeDopplerHeatMap       = atoi (argv[4]);

    /* Save Configuration to use later */
    memcpy((void *)&gMmwMssMCB.cfg.guiMonSel, (void *)&guiMonSel, sizeof(MmwDemo_GuiMonSel));
    
    /* Send configuration to DSS */
    memset((void *)&message, 0, sizeof(MmwDemo_message));

    message.type = MMWDEMO_MSS2DSS_GUIMON_CFG;
    memcpy((void *)&message.body.guiMonSel, (void *)&guiMonSel, sizeof(MmwDemo_GuiMonSel));

    if (MmwDemo_mboxWrite(&message) == 0)
        return 0;
    else
        return -1;
}

/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for CFAR configuration
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
static int32_t MmwDemo_CLICfarCfg (int32_t argc, char* argv[])
{
	mmwDemoCfarConfig     cfarCfg;
    MmwDemo_message     message;

    /* Sanity Check: Minimum argument check */
    if (argc != 13)
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }

    /* Initialize the ADC Output configuration: */
    memset ((void *)&cfarCfg, 0, sizeof(mmwDemoCfarConfig));

    //System_printf("CFAR config\n");

    //cliCfg.tableEntry[4].helpString     = "<detMode> <discardLeft> <discardRight> <refWinSize1> <refWinSize2> <guardWinSize1> <guardWinSize2> <thre>";
    /* Populate configuration: */
    cfarCfg.cfarMethod       = (uint16_t) atoi (argv[1]);
    cfarCfg.cfarDiscardRangeLeft  = (uint16_t) atoi (argv[2]);
    cfarCfg.cfarDiscardRangeRight = (uint16_t) atoi (argv[3]);
    cfarCfg.cfarDiscardAngleLeft  = (uint16_t) atoi (argv[4]);
    cfarCfg.cfarDiscardAngleRight = (uint16_t) atoi (argv[5]);
    cfarCfg.refWinSize[0]    = (uint16_t) atoi (argv[6]);
    cfarCfg.refWinSize[1]    = (uint16_t) atoi (argv[7]);
    cfarCfg.guardWinSize[0]  = (uint16_t) atoi (argv[8]);
    cfarCfg.guardWinSize[1]  = (uint16_t) atoi (argv[9]);
    cfarCfg.rangeThre             = (float) atoi (argv[10]) * 0.1f;
    cfarCfg.azimuthThre     = (float) atoi (argv[11]) * 0.1f;
    cfarCfg.log2MagFlag        = (uint16_t) atoi (argv[12]);
    //System_printf("CFAR config:method = %d\n", cfarCfg.cfarMethod);

    /* Save Configuration to use later */
    memcpy((void *)&gMmwMssMCB.cfg.cfarCfg, (void *)&cfarCfg, sizeof(mmwDemoCfarConfig));

    /* Send configuration to DSS */
    memset((void *)&message, 0, sizeof(MmwDemo_message));

    message.type = MMWDEMO_MSS2DSS_CFAR_CFG;
    memcpy((void *)&message.body.cfar, (void *)&cfarCfg, sizeof(mmwDemoCfarConfig));

    if (MmwDemo_mboxWrite(&message) == 0)
        return 0;
    else
        return -1;    
}

/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for DOA configuration
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
static int32_t MmwDemo_CLIDoACfg (int32_t argc, char* argv[])
{
	mmwDemoDoaConfig     doaCfg;
    MmwDemo_message     message;

    /* Sanity Check: Minimum argument check */
    if (argc != 7)
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }

    /* Initialize the ADC Output configuration: */
    memset ((void *)&doaCfg, 0, sizeof(mmwDemoDoaConfig));

    //cliCfg.tableEntry[5].helpString     = "<doaMode> <doaGamma> <sideLobe_dB> <searchRange> <searchRes> <varThre>";
    /* Populate configuration: */
    doaCfg.estAngleRange       	= (float) atoi (argv[1]) * 0.1;
    doaCfg.estAngleResolution      = (float) atoi (argv[2]) * 0.001;
    doaCfg.gamma            = (float) atoi (argv[3]) * 0.001f;
    doaCfg.clutterRemovalFlag = (uint8_t) atoi (argv[4]);
    doaCfg.dopplerOversampleFactor = (uint8_t) atoi (argv[5]);
    doaCfg.useCFAR4DopDet = (uint8_t) atoi (argv[6]);

    /* Save Configuration to use later */
    memcpy((void *)&gMmwMssMCB.cfg.doaCfg, (void *)&doaCfg, sizeof(mmwDemoDoaConfig));

    /* Send configuration to DSS */
    memset((void *)&message, 0, sizeof(MmwDemo_message));

    message.type = MMWDEMO_MSS2DSS_DOA_CFG;
    memcpy((void *)&message.body.doa, (void *)&doaCfg, sizeof(mmwDemoDoaConfig));

    if (MmwDemo_mboxWrite(&message) == 0)
        return 0;
    else
        return -1;
}

/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for data logger set command
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
static int32_t MmwDemo_CLIADCBufCfg (int32_t argc, char* argv[])
{
    MmwDemo_ADCBufCfg   adcBufCfg;
    MmwDemo_message     message;

    /* Sanity Check: Minimum argument check */
    if (argc != 5)
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }

    /* Initialize the ADC Output configuration: */
    memset ((void *)&adcBufCfg, 0, sizeof(MmwDemo_ADCBufCfg));

    /* Populate configuration: */
    adcBufCfg.adcFmt          = (uint8_t) atoi (argv[1]);
    adcBufCfg.iqSwapSel       = (uint8_t) atoi (argv[2]);
    adcBufCfg.chInterleave    = (uint8_t) atoi (argv[3]);
    adcBufCfg.chirpThreshold  = (uint8_t) atoi (argv[4]);

    /* Save Configuration to use later */
    memcpy((void *)&gMmwMssMCB.cfg.adcBufCfg, (void *)&adcBufCfg, sizeof(MmwDemo_ADCBufCfg));

    /* Send configuration to DSS */
    memset((void *)&message, 0, sizeof(MmwDemo_message));
    message.type = MMWDEMO_MSS2DSS_ADCBUFCFG;
    memcpy((void *)&message.body.adcBufCfg, (void *)&adcBufCfg, sizeof(MmwDemo_ADCBufCfg));

    if (MmwDemo_mboxWrite(&message) == 0)
        return 0;
    else
        return -1;
}


/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for compensation of range bias and channel phase offsets
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
static int32_t MmwDemo_CLICompRangeBiasAndRxChanPhaseCfg (int32_t argc, char* argv[])
{
    MmwDemo_compRxChannelBiasCfg_t   cfg;
    MmwDemo_message     message;
    int32_t argInd;
    int32_t i;

    /* Sanity Check: Minimum argument check */
    if (argc != (1+1+SYS_COMMON_NUM_TX_ANTENNAS*SYS_COMMON_NUM_RX_CHANNEL*2))
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }

    /* Initialize configuration: */
    memset ((void *)&cfg, 0, sizeof(MmwDemo_compRxChannelBiasCfg_t));

    /* Populate configuration: */
    cfg.rangeBias          = (float) atof (argv[1]);

    argInd = 2;
    for (i=0; i < SYS_COMMON_NUM_TX_ANTENNAS*SYS_COMMON_NUM_RX_CHANNEL; i++)
    {
        cfg.rxChPhaseComp[2 * i + 0] = (float) atof (argv[argInd++]);
        cfg.rxChPhaseComp[2 * i + 1] = (float) atof (argv[argInd++]);
    }
    /* Save Configuration to use later */
    memcpy((void *) &gMmwMssMCB.cfg.antCompCfg, &cfg, sizeof(MmwDemo_compRxChannelBiasCfg_t));

    /* Send configuration to DSS */
    memset((void *)&message, 0, sizeof(MmwDemo_message));
    message.type = MMWDEMO_MSS2DSS_COMP_RANGE_BIAS_AND_RX_CHAN_PHASE;
    memcpy((void *)&message.body.antCompCfg, (void *)&cfg, sizeof(MmwDemo_compRxChannelBiasCfg_t));

    if (MmwDemo_mboxWrite(&message) == 0)
        return 0;
    else
        return -1;
}


/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for data logger set command
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
static int32_t MmwDemo_CLISetDataLogger (int32_t argc, char* argv[])
{
    MmwDemo_message     message;

    /* Sanity Check: Minimum argument check */
    if (argc != 2)
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }


    /* Save Configuration to use later */
    if (strcmp(argv[1], "mssLogger") == 0)  
        gMmwMssMCB.cfg.dataLogger = 0;
    else if (strcmp(argv[1], "dssLogger") == 0)  
        gMmwMssMCB.cfg.dataLogger = 1;
    else
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }
       
    /* Send configuration to DSS */
    memset((void *)&message, 0, sizeof(MmwDemo_message));

    message.type = MMWDEMO_MSS2DSS_SET_DATALOGGER;
    message.body.dataLogger = gMmwMssMCB.cfg.dataLogger;

    if (MmwDemo_mboxWrite(&message) == 0)
        return 0;
    else
        return -1;
}

int32_t LiveCFG_CFAR (int32_t argc, char* argv[]) {
	MmwDemo_message     message;
	mmwDemoCfarConfig     cfarCfg;

	/* Sanity Check*/
	if (argc != 4) {
	   CLI_write ("Error: Invalid usage of the CLI command\n");
       return -1;
	}

	/*init the message values*/
  	memset((void *)&message, 0, sizeof(MmwDemo_message));
	memset ((void *)&cfarCfg, 0, sizeof(mmwDemoCfarConfig));

  	cfarCfg.rangeThre             = (float) atoi (argv[1]) * 0.1f;
    cfarCfg.azimuthThre     = (float) atoi (argv[2]) * 0.1f;
    cfarCfg.cfarMethod       = (uint16_t) atoi (argv[3]);

    message.type = MMWDEMO_MSS2DSS_LIVE_CFAR_THRESH_CFG;
    memcpy((void *)&message.body.cfar, (void *)&cfarCfg, sizeof(mmwDemoCfarConfig));

    if (MmwDemo_mboxWrite(&message) == 0)
        return 0;
    else
        return -1;
}


/**
 *  @b Description
 *  @n
 *      This is the CLI Execution Task
 *
 *  @retval
 *      Not Applicable.
 */
void MmwDemo_CLIInit (void)
{
    CLI_Cfg     cliCfg;
    char        demoBanner[256];

    /* Create Demo Banner to be printed out by CLI */
    sprintf(&demoBanner[0], "******************************************\n" \
                       "MMW TM Demo %s\n"  \
                       "******************************************\n", MMW_VERSION);
    
    /* Initialize the CLI configuration: */
    memset ((void *)&cliCfg, 0, sizeof(CLI_Cfg));

    /* Populate the CLI configuration: */
    cliCfg.cliPrompt                    = "mmwDemo:/>";
    cliCfg.cliBanner                    = demoBanner;
    cliCfg.cliUartHandle                = gMmwMssMCB.commandUartHandle;
    cliCfg.socHandle                    = gMmwMssMCB.socHandle;
    cliCfg.taskPriority                 = 3;
    cliCfg.mmWaveHandle                 = gMmwMssMCB.ctrlHandle;
    cliCfg.enableMMWaveExtension        = 1U;
    cliCfg.usePolledMode                = true;
    cliCfg.tableEntry[0].cmd            = "sensorStart";
    cliCfg.tableEntry[0].helpString     = "No arguments";
    cliCfg.tableEntry[0].cmdHandlerFxn  = MmwDemo_CLISensorStart;
    cliCfg.tableEntry[1].cmd            = "sensorStop";
    cliCfg.tableEntry[1].helpString     = "No arguments";
    cliCfg.tableEntry[1].cmdHandlerFxn  = MmwDemo_CLISensorStop;
    cliCfg.tableEntry[2].cmd            = "frameStart";
    cliCfg.tableEntry[2].helpString     = "No arguments";
    cliCfg.tableEntry[2].cmdHandlerFxn  = MmwDemo_CLIFrameStart;
    cliCfg.tableEntry[3].cmd            = "guiMonitor";
    cliCfg.tableEntry[3].helpString     = "<detectedObjects> <logMagRange> <rangeAzimuthHeatMap> <rangeDopplerHeatMap>";
    cliCfg.tableEntry[3].cmdHandlerFxn  = MmwDemo_CLIGuiMonSel;
    cliCfg.tableEntry[4].cmd            = "cfarCfg";
    cliCfg.tableEntry[4].helpString     = "<detMode> <discardLeft> <discardRight> <refWinSize1> <refWinSize2> <guardWinSize1> <guardWinSize2> <thre>";
    cliCfg.tableEntry[4].cmdHandlerFxn  = MmwDemo_CLICfarCfg;
    cliCfg.tableEntry[5].cmd            = "doaCfg";
    cliCfg.tableEntry[5].helpString     = "<doaMode> <doaGamma> <sideLobe_dB> <searchRange> <searchRes> <varThre>";
    cliCfg.tableEntry[5].cmdHandlerFxn  = MmwDemo_CLIDoACfg;
    cliCfg.tableEntry[6].cmd            = "trackingCfg";
    cliCfg.tableEntry[6].helpString     = "<enable> <paramSet> <numPoints> <numTracks> <maxDoppler> <framePeriod>";
    cliCfg.tableEntry[6].cmdHandlerFxn  = MmwDemo_CLITrackingCfg;
    cliCfg.tableEntry[7].cmd            = "dataLogger";
    cliCfg.tableEntry[7].helpString     = "<mssLogger | dssLogger>";
    cliCfg.tableEntry[7].cmdHandlerFxn  = MmwDemo_CLISetDataLogger;
    cliCfg.tableEntry[8].cmd            = "adcbufCfg";
    cliCfg.tableEntry[8].helpString     = "<adcOutputFmt> <SampleSwap> <ChanInterleave> <ChirpThreshold>";
    cliCfg.tableEntry[8].cmdHandlerFxn  = MmwDemo_CLIADCBufCfg;
    cliCfg.tableEntry[9].cmd            = "compRangeBiasAndRxChanPhase";
    cliCfg.tableEntry[9].helpString     = "<rangeBias> <Re00> <Im00> <Re01> <Im01> <Re02> <Im02> <Re03> <Im03> <Re10> <Im10> <Re11> <Im11> <Re12> <Im12> <Re13> <Im13> ";
    cliCfg.tableEntry[9].cmdHandlerFxn  = MmwDemo_CLICompRangeBiasAndRxChanPhaseCfg;

     cliCfg.tableEntry[10].cmd            = "SceneryParam";// PC OFFICE Left Wall (-1.5), Right Wall (1.5), Bottom Exit Zone (1m), Upper Exit Zone (4.5m)
     cliCfg.tableEntry[10].helpString     = "<Left Wall> <Right Wall> <Bottom Exit Zone> <Upper Exit Zone>";
     cliCfg.tableEntry[10].cmdHandlerFxn  = MmwDemo_CLISceneryParamCfg;
     cliCfg.tableEntry[11].cmd            = "GatingParam";// PC: 4 gating volume, Limits are set to 3m in length, 2m in width, 0 no limit in doppler
     cliCfg.tableEntry[11].helpString     = "<gating volume> <length> <width> <doppler>";
     cliCfg.tableEntry[11].cmdHandlerFxn  = MmwDemo_CLIGatingParamCfg;
     cliCfg.tableEntry[12].cmd            = "StateParam";// PC: 10 frames to activate, 5 to forget, 10 active to free, 1000 static to free, 5 exit to free
     cliCfg.tableEntry[12].helpString     = "<det2act> <det2free> <act2free> <stat2free> <exit2free>";//det2act, det2free, act2free, stat2free, exit2free
     cliCfg.tableEntry[12].cmdHandlerFxn  = MmwDemo_CLIStateParamCfg;
     cliCfg.tableEntry[13].cmd            = "AllocationParam";// PC: 250 SNR, 0.1 minimal velocity, 5 points, 1m in distance, 2m/s in velocity
     cliCfg.tableEntry[13].helpString     = "<SNRs> <minimal velocity> <points> <in distance> <in velocity>";
     cliCfg.tableEntry[13].cmdHandlerFxn  = MmwDemo_CLIAllocationParamCfg;
     cliCfg.tableEntry[14].cmd            = "VariationParam";// PC: 1m height, 1m in width, 1 m/s for doppler
     cliCfg.tableEntry[14].helpString     = "<height> <width> <doppler>";
     cliCfg.tableEntry[14].cmdHandlerFxn  = MmwDemo_CLIVariationParamCfg;
     cliCfg.tableEntry[15].cmd            = "MaxAcceleration";
     cliCfg.tableEntry[15].helpString     = "<max X Acceleration> <max Y> <max Z>";
     cliCfg.tableEntry[15].cmdHandlerFxn  = MmwDemoCLIMaxAccelerationParamCfg;

         /*! Enable Point Cloud */
     cliCfg.tableEntry[16].cmd            = "PointCloudEn"; //1 is on, 0 is off
     cliCfg.tableEntry[16].helpString     = "<1 is on, 0 is off>";
     cliCfg.tableEntry[16].cmdHandlerFxn  = MmwDemo_CLIPointCloudEn;

     /* Live config commands:
      * CFAR - only thresholds
      * DOA - only 1 param for now
      * Allocation
      * Gating
      * State
      */
     cliCfg.tableEntry[17].cmd 			  = "LiveCFARThresh";
 	 cliCfg.tableEntry[17].helpString     = "<Range Thresh,  Angle Thresh, clutterRemovalFlag>";
	 cliCfg.tableEntry[17].cmdHandlerFxn  = LiveCFG_CFAR;
	 cliCfg.tableEntry[18].cmd 			  = "LiveDOA";
 	 cliCfg.tableEntry[18].helpString     = "Sets DOA angle step size";
	 cliCfg.tableEntry[18].cmdHandlerFxn  = LiveCFG_DOA;
  	 cliCfg.tableEntry[19].cmd            = "LiveAllocation";
 	 cliCfg.tableEntry[19].helpString 	  = "<SNRs> <SNR Obscured Threshold> <minimal velocity> <points> <in distance> <in velocity>";
	 cliCfg.tableEntry[19].cmdHandlerFxn  = LiveCFG_Allocation;
     cliCfg.tableEntry[20].cmd 			  = "LiveGating";
 	 cliCfg.tableEntry[20].helpString     = "<gating volume> <length> <width> <doppler>";
	 cliCfg.tableEntry[20].cmdHandlerFxn  = LiveCFG_Gating;
     cliCfg.tableEntry[21].cmd 			  = "LiveState";
 	 cliCfg.tableEntry[21].helpString     = "<det2act> <det2free> <act2free> <stat2free> <exit2free>";
	 cliCfg.tableEntry[21].cmdHandlerFxn  = LiveCFG_State;
     cliCfg.tableEntry[22].cmd 			  = "LiveScenery";
 	 cliCfg.tableEntry[22].helpString     = "<Left Wall> <Right Wall> <Bottom Exit Zone> <Upper Exit Zone>";
	 cliCfg.tableEntry[22].cmdHandlerFxn  = LiveCFG_Scenery;

    /* Open the CLI: */
    if (CLI_open (&cliCfg) < 0)
    {
        System_printf ("Error: Unable to open the CLI\n");
        return;
    }
    System_printf ("Debug: CLI is operational\n");
    return;
}


