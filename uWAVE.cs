using System;
using System.Collections.Generic;

namespace uWAVELib
{
    #region Custom enums

    public enum RC_CODES_Enum
    {
        RC_PING = 0,
        RC_PONG = 1,
        RC_DPT_GET = 2,
        RC_TMP_GET = 3,
        RC_BAT_V_GET = 4,
        RC_ERR_NSUP = 5,
        RC_ACK = 6,
        RC_USR_CMD_000 = 7,
        RC_USR_CMD_001 = 8,
        RC_USR_CMD_002 = 9,
        RC_USR_CMD_003 = 10,
        RC_USR_CMD_004 = 11,
        RC_USR_CMD_005 = 12,
        RC_USR_CMD_006 = 13,
        RC_USR_CMD_007 = 14,
        RC_USR_CMD_008 = 15,
        RC_INVALID
    }
    
    public enum LocalError_Enum
    {
        LOC_ERR_NO_ERROR = 0,
        LOC_ERR_INVALID_SYNTAX = 1,
        LOC_ERR_UNSUPPORTED = 2,
        LOC_ERR_TRANSMITTER_BUSY = 3,
        LOC_ERR_ARGUMENT_OUT_OF_RANGE = 4,
        LOC_ERR_INVALID_OPERATION = 5,
        LOC_ERR_UNKNOWN_FIELD_ID = 6,
        LOC_ERR_VALUE_UNAVAILABLE = 7,
        LOC_ERR_RECEIVER_BUSY = 8,
        LOC_ERR_TX_BUFFER_OVERRUN = 9,
        LOC_ERR_CHKSUM_ERROR = 10,
        LOC_ERR_TX_FINISHED = 11,

        // 12.06.2019
        LOC_ACK_BEFORE_STANDBY = 12,
        LOC_ACK_AFTER_WAKEUP = 13,

        // 30.10.2019
        LOC_ERR_SVOLTAGE_TOO_HIGH = 14,

        LOC_ERR_UNKNOWN
    }

    public enum ICs
    {
        IC_D2H_ACK,
        IC_H2D_SETTINGS_WRITE,
        IC_H2D_RC_REQUEST,
        IC_D2H_RC_RESPONSE,
        IC_D2H_RC_TIMEOUT,
        IC_D2H_RC_ASYNC_IN,
        IC_H2D_AMB_DTA_CFG,
        IC_D2H_AMB_DTA,        
        IC_H2D_DINFO_GET,
        IC_D2H_DINFO,

        IC_H2D_PT_SETTINGS_READ,
        IC_D2H_PT_SETTINGS,
        IC_H2H_PT_SETTINGS_WRITE,
        IC_H2D_PT_SEND,
        IC_D2H_PT_FAILED,
        IC_D2H_PT_DLVRD,
        IC_D2H_PT_RCVD,

        IC_D2H_PT_HEARD,
        IC_D2H_CM_HEARD,

        IC_D2H_ANY,
        IC_INVALID
    }

    public enum uLBLBaseIDs
    {
        BaseID_1 = 0,
        BaseID_2 = 1,
        BaseID_3 = 2,
        BaseID_4 = 3,
        Invalid
    }

    public enum uLBLPingerDataIDs
    {
        DPT,
        TMP,
        BAT,
        Invalid
    }
    
    #endregion
    
    public static class uWAVE
    {
        public static readonly double MIN_TEMPERATURE_C = -4.0;
        public static readonly double MAX_TEMPERATURE_C = 46.0;
        public static readonly double MIN_DEPTH_M = 0.0;
        public static readonly double MAX_DEPTH_M = 300.0;
        public static readonly double MIN_BAT_VOLTAGE = 0.0;
        public static readonly double MAX_BAT_VOLTAGE = 20.0;
        public static readonly double CR_DPT_OFFSET = 0;
        public static readonly double CR_DPT_CRANGE = 3000;
        public static readonly double CR_TMP_OFFSET = 3001;
        public static readonly double CR_TMP_CRANGE = 500;
        public static readonly double CR_BAT_OFFSET = 3502;
        public static readonly double CR_BAT_CRANGE = 200;
        public static readonly double CR_DPT_RANGE = (MAX_DEPTH_M - MIN_DEPTH_M);
        public static readonly double CR_DPT_SCALE = (CR_DPT_RANGE / CR_DPT_CRANGE);
        public static readonly double CR_TMP_RANGE = (MAX_TEMPERATURE_C - MIN_TEMPERATURE_C);
        public static readonly double CR_TMP_SCALE = (CR_TMP_RANGE / CR_TMP_CRANGE);
        public static readonly double CR_BAT_RANGE = (MAX_BAT_VOLTAGE - MIN_BAT_VOLTAGE);
        public static readonly double CR_BAT_SCALE = (CR_BAT_RANGE / CR_BAT_CRANGE);

        public static readonly int CR_STRONG_MAX_CODE_CHANNELS = 28;

        public static readonly int PT_MAX_PACKET_SIZE = 64;


        static Dictionary<string, ICs> ICsIdxArray = new Dictionary<string, ICs>()
        {
            { "0", ICs.IC_D2H_ACK },
            { "1", ICs.IC_H2D_SETTINGS_WRITE },
            { "2", ICs.IC_H2D_RC_REQUEST },
            { "3", ICs.IC_D2H_RC_RESPONSE },
            { "4", ICs.IC_D2H_RC_TIMEOUT },
            { "5", ICs.IC_D2H_RC_ASYNC_IN },
            { "6", ICs.IC_H2D_AMB_DTA_CFG },
            { "7", ICs.IC_D2H_AMB_DTA },               
            { "?", ICs.IC_H2D_DINFO_GET },
            { "!", ICs.IC_D2H_DINFO },

            { "D", ICs.IC_H2D_PT_SETTINGS_READ },
            { "E", ICs.IC_D2H_PT_SETTINGS },
            { "F", ICs.IC_H2H_PT_SETTINGS_WRITE },
            { "G", ICs.IC_H2D_PT_SEND },
            { "H", ICs.IC_D2H_PT_FAILED },
            { "I", ICs.IC_D2H_PT_DLVRD },
            { "J", ICs.IC_D2H_PT_RCVD },

            {"K", ICs.IC_D2H_PT_HEARD },
            {"L", ICs.IC_D2H_CM_HEARD },

            { "-", ICs.IC_D2H_ANY }
        };

        public static void Rise(this EventHandler handler, object sender, EventArgs e)
        {
            if (handler != null)
                handler(sender, e);
        }

        public static void Rise<TEventArgs>(this EventHandler<TEventArgs> handler,
            object sender, TEventArgs e) where TEventArgs : EventArgs
        {
            if (handler != null)
                handler(sender, e);
        }      

        public static ICs ICsByMessageID(string msgID)
        {
            if (ICsIdxArray.ContainsKey(msgID))
                return ICsIdxArray[msgID];
            else
                return ICs.IC_INVALID;
        }

        public static string BCDVersionToStr(int versionData)
        {
            return string.Format("{0}.{1}", (versionData >> 0x08).ToString(), (versionData & 0xff).ToString("X2"));
        }
    }
}
