using System;
using System.Collections.Generic;
using System.IO.Ports;
using UCNLDrivers;
using UCNLNMEA;

namespace uWAVELib
{
    #region Custom EventArgs

    public class RCAsyncInReceivedEventArgs : EventArgs
    {
        #region Properties

        public RC_CODES_Enum RCCmdID { get; private set; }
        public double SNR_db { get; private set; }
        public double Azimuth { get; private set; }
        public bool IsAzimuthPresent
        {
            get
            {
                return !double.IsNaN(Azimuth);                     
            }
        }

        #endregion

        #region Constructor

        public RCAsyncInReceivedEventArgs(RC_CODES_Enum rcCmdID, double snr_db)
            : this(rcCmdID, snr_db, double.NaN)
        {
        }

        public RCAsyncInReceivedEventArgs(RC_CODES_Enum rcCmdID, double snr_db, double azimuth)
        {
            RCCmdID = rcCmdID;
            SNR_db = snr_db;
            Azimuth = azimuth;
        }

        #endregion
    }

    public class RCTimeoutReceivedEventArgs : EventArgs
    {
        #region Properties

        public int TxChID { get; private set; }
        public RC_CODES_Enum RCCmdID { get; private set; }
        
        #endregion

        #region Constructor

        public RCTimeoutReceivedEventArgs(int txChID, RC_CODES_Enum rcCmdID)
        {
            TxChID = txChID;
            RCCmdID = rcCmdID;
        }

        #endregion
    }

    public class RCResponseReceivedEventArgs : EventArgs
    {
        #region Properties

        public int TxChID { get; private set; }
        public RC_CODES_Enum RCCmdID { get; private set; }
        public double PropTime_sec { get; private set; }
        public double SNR_db { get; private set; }
        public double Value { get; private set; }
        public double Azimuth { get; private set; }

        public bool IsValuePresent
        {
            get
            {
                return !double.IsNaN(Value);
            }
        }

        public bool IsAzimuthPresent
        {
            get
            {
                return !double.IsNaN(Azimuth);
            }
        }

        #endregion

        #region Constructor

        public RCResponseReceivedEventArgs(int txChID, RC_CODES_Enum rcCmdID, double pTime_sec, double snr_db)
            : this(txChID, rcCmdID, pTime_sec, snr_db, double.NaN, double.NaN)
        {
        }

        public RCResponseReceivedEventArgs(int txChID, RC_CODES_Enum rcCmdID, double pTime_sec, double snr_db, double value)
            : this(txChID, rcCmdID, pTime_sec, snr_db, value, double.NaN)
        {
        }

        public RCResponseReceivedEventArgs(int txChID, RC_CODES_Enum rcCmdID, double pTime_sec, double snr_db, double value, double azimuth)
        {
            TxChID = txChID;
            RCCmdID = rcCmdID;
            PropTime_sec = pTime_sec;
            SNR_db = snr_db;
            Value = value;
            Azimuth = azimuth;
        }

        #endregion
    }

    public class DeviceTimeoutEventArgs : EventArgs
    {
        #region Properties

        public ICs SentenceID { get; private set; }

        #endregion

        #region Constructor

        public DeviceTimeoutEventArgs(ICs sntID)
        {
            SentenceID = sntID;
        }

        #endregion
    }

    public class ACKReceivedEventArgs : EventArgs
    {
        #region Properties

        public ICs SentenceID { get; private set; }
        public LocalError_Enum ErrorID { get; private set; }

        #endregion

        #region Constructor

        public ACKReceivedEventArgs(ICs sntID, LocalError_Enum errID)
        {
            SentenceID = sntID;
            ErrorID = errID;
        }

        #endregion
    }    

    public class UnknownSentenceEventArgs : EventArgs
    {
        #region Properties

        public NMEASentence Sentence { get; private set; }

        #endregion

        #region Constructor

        public UnknownSentenceEventArgs(NMEASentence sentence)
        {
            Sentence = sentence;
        }

        #endregion
    }


    public class PacketEventArgs : EventArgs
    {
        #region Properties

        public byte Target_ptAddress { get; private set; }
        public byte TriesTaken { get; private set; }
        public byte[] DataPacket { get; private set; }

        #endregion

        #region Constructor

        public PacketEventArgs(byte target_ptAddress, byte triesTaken, byte[] dataPacket)
        {
            Target_ptAddress = target_ptAddress;
            TriesTaken = triesTaken;
            DataPacket = dataPacket;
        }

        #endregion
    }

    public class PacketReceivedEventArgs : EventArgs
    {
        #region Properties

        public byte Target_ptAddress { get; private set; }
        public byte[] DataPacket { get; private set; }

        #endregion

        #region Constructor

        public PacketReceivedEventArgs(byte target_ptAddress, byte[] dataPacket)
        {
            Target_ptAddress = target_ptAddress;
            DataPacket = dataPacket;
        }

        #endregion
    }

    #endregion
    
    public class uWAVEPort : IDisposable
    {
        #region Properties

        bool disposed = false;
        static bool singleton = false;

        NMEASerialPort port;
        PrecisionTimer timer;

        public bool IsOpen
        {
            get { return port.IsOpen; }
        }

        public bool IsCommandMode { get; set; }

        public string PortName
        {
            get
            {
                return port.PortName;
            }
        }

        public BaudRate PortBaudrate
        {
            get { return port.PortBaudRate; }
        }


        ICs lastQueryID = ICs.IC_INVALID;

        static readonly int waitingRemoteMaxCnt = 5;
        static readonly int waitingLocalMaxCnt = 3;

        int waitingRemoteCnt = 0;
        bool isWaitingRemote = false;
        public bool IsWaitingRemote
        {
            get { return isWaitingRemote; }
            private set
            {
                isWaitingRemote = value;
                if (isWaitingRemote)
                    waitingRemoteCnt = 0;
            }
        }

        int waitingLocalCnt = 0;
        bool isWaitingLocal = false;
        public bool IsWaitingLocal
        {
            get { return isWaitingLocal; }
            private set
            {
                isWaitingLocal = value;
                if (isWaitingLocal)
                {
                    waitingLocalCnt = 0;
                }
            }
        }
        
        #region Device information

        public string SerialNumber { get; private set; }
        public string SystemMoniker { get; private set; }
        public string SystemVersion { get; private set; }
        public string CoreMoniker { get; private set; }
        public string CoreVersion { get; private set; }
        public int RxChID { get; private set; }
        public int TxChID { get; private set; }
        public int TotalCodeChannels { get; private set; }
        public double AcousticBaudrate { get; private set; }
        public double SalinityPSU { get; private set; }

        public bool IsPTS { get; private set; }
        public bool IsCommandModeByDefault { get; private set; }
        
        public bool IsDeviceInfo
        {
            get
            {
                return !string.IsNullOrEmpty(SystemMoniker) && !string.IsNullOrEmpty(SystemVersion) &&
                        !string.IsNullOrEmpty(CoreMoniker) && !string.IsNullOrEmpty(CoreVersion) &&
                        (RxChID >= 0) && (TxChID >= 0) && (TotalCodeChannels > 0) &&
                        !double.IsNaN(AcousticBaudrate) &&
                        !double.IsNaN(SalinityPSU);
            }
        }


        public bool IsPacketMode { get; private set; }
        public byte PacketMode_Address { get; private set; }

        #endregion

        #region Ambient & supply voltage data

        public double Pressure_mBar { get; private set; }
        public double Temperature_C { get; private set; }
        public double Depth_m { get; private set; }
        public double SupplyVoltage_V { get; private set; }

        #endregion      

        private delegate void parserDelegate(object[] parameters);
        private Dictionary<ICs, parserDelegate> parsers;

        delegate T NullChecker<T>(object parameter);
        NullChecker<int> intNullChecker = (x => x == null ? -1 : (int)x);
        NullChecker<double> doubleNullChecker = (x => x == null ? double.NaN : (double)x);
        NullChecker<string> stringNullChecker = (x => x == null ? string.Empty : (string)x);

        #endregion

        #region Constructor

        public uWAVEPort(string portName, BaudRate portBaudrate)
        {
            #region NMEA init

            if (!singleton)
            {                              
                NMEAParser.AddManufacturerToProprietarySentencesBase(ManufacturerCodes.UWV);

                #region Common sentences

                // IC_D2H_ACK             $PUWV0,cmdID,errCode
                NMEAParser.AddProprietarySentenceDescription(ManufacturerCodes.UWV, "0", "c--c,x");

                // IC_H2D_SETTINGS_WRITE  $PUWV1,rxChID,txChID,styPSU,isCmdMode,isACKOnTXFinished,gravityAcc
                NMEAParser.AddProprietarySentenceDescription(ManufacturerCodes.UWV, "1", "x,x,x.x,x,x,x.x");

                #endregion

                #region Short code messages management sentences

                // IC_H2D_RC_REQUEST      $PUWV2,txChID,rxChID,rcCmdID
                NMEAParser.AddProprietarySentenceDescription(ManufacturerCodes.UWV, "2", "x,x,x");

                // IC_D2H_RC_RESPONSE     $PUWV3,txChID,rcCmdID,propTime_seс,snr,[value],[azimuth]
                NMEAParser.AddProprietarySentenceDescription(ManufacturerCodes.UWV, "3", "x,x,x.x,x.x,x.x,x.x");

                // IC_D2H_RC_TIMEOUT      $PUWV4,txChID,rcCmdID
                NMEAParser.AddProprietarySentenceDescription(ManufacturerCodes.UWV, "4", "x,x");

                // IC_D2H_RC_ASYNC_IN     $PUWV5,rcCmdID,snr,[azimuth]
                NMEAParser.AddProprietarySentenceDescription(ManufacturerCodes.UWV, "5", "x,x.x,x.x");

                #endregion

                #region Ambient data management sentences

                // IC_H2D_AMB_DTA_CFG     $PUWV6,isWriteInFlash,periodMs,isPrs,isTemp,isDpt,isBatV
                NMEAParser.AddProprietarySentenceDescription(ManufacturerCodes.UWV, "6", "x,x,x,x,x,x");

                // IC_D2H_AMB_DTA         $PUWV7,prs_mBar,temp_C,dpt_m,batVoltage_V
                NMEAParser.AddProprietarySentenceDescription(ManufacturerCodes.UWV, "7", "x.x,x.x,x.x,x.x");

                #endregion                

                #region Device info

                // IC_H2D_DINFO_GET       $PUWV?,reserved
                NMEAParser.AddProprietarySentenceDescription(ManufacturerCodes.UWV, "?", "x");

                // IC_D2H_DINFO $PUWV!,serial_number,sys_moniker,sys_version,core_moniker [release],core_version,acBaudrate,rxChID,txChID,totalCh,styPSU,isPTS,isCmdModeDefault                
                NMEAParser.AddProprietarySentenceDescription(ManufacturerCodes.UWV, "!", "c--c,c--c,x,c--c,x,x.x,x,x,x,x.x,x,x");

                #endregion

                #region Packet mode

                // IC_H2D_PT_SETTINGS_READ   $PUWVD,reserved
                NMEAParser.AddProprietarySentenceDescription(ManufacturerCodes.UWV, "D", "x");

                // IC_D2H_PT_SETTINGS        $PUWVE,isPTMode,ptAddress
                NMEAParser.AddProprietarySentenceDescription(ManufacturerCodes.UWV, "E", "x,x");

                // IC_H2H_PT_SETTINGS_WRITE  $PUWVF,isSaveInFlash,isPTMode,ptAddress
                NMEAParser.AddProprietarySentenceDescription(ManufacturerCodes.UWV, "F", "x,x,x");

                // IC_H2D_PT_SEND            $PUWVG,tareget_ptAddress,[maxTries],data
                NMEAParser.AddProprietarySentenceDescription(ManufacturerCodes.UWV, "G", "x,x,h--h");

                // IC_D2H_PT_FAILED          $PUWVH,tareget_ptAddress,triesTaken,data
                NMEAParser.AddProprietarySentenceDescription(ManufacturerCodes.UWV, "H", "x,x,h--h");

                // IC_D2H_PT_DLVRD           $PUWVI,tareget_ptAddress,triesTaken,data
                NMEAParser.AddProprietarySentenceDescription(ManufacturerCodes.UWV, "I", "x,x,h--h");

                // IC_D2H_PT_RCVD            $PUWVJ,sender_ptAddress,data
                NMEAParser.AddProprietarySentenceDescription(ManufacturerCodes.UWV, "J", "x,h--h");

                #endregion

                #region LBL-related

                // IC_D2H_PT_HEARD          $PUWVK,b_id,b_lat,b_lon,b_toa_s,sender_pt_Address,dataPacket
                NMEAParser.AddProprietarySentenceDescription(ManufacturerCodes.UWV, "K", "x,x.x,x.x,x.x,x,h--h");

                //IC_D2H_CM_HEARD           $PUWVL,b_id,b_lat,b_lon,b_toa_s,data_ID,dataValue,msr_dB
                NMEAParser.AddProprietarySentenceDescription(ManufacturerCodes.UWV, "L", "x,x.x,x.x,x.x,x,x.x,x.x");

                #endregion

                singleton = true;
            }

            #endregion

            #region Parsers

            parsers = new Dictionary<ICs, parserDelegate>()
            {
                { ICs.IC_D2H_ACK, new parserDelegate(ACK_Parse) },
                { ICs.IC_D2H_AMB_DTA, new parserDelegate(AMB_DTA_Parse) },
                { ICs.IC_D2H_DINFO, new parserDelegate(DINFO_Parse) },
                { ICs.IC_D2H_RC_ASYNC_IN, new parserDelegate(RC_ASYNC_IN_Parse) },
                { ICs.IC_D2H_RC_RESPONSE, new parserDelegate(RC_RESPONSE_Parse) },
                { ICs.IC_D2H_RC_TIMEOUT, new parserDelegate(RC_TIMEOUT_Parse) },
        
                { ICs.IC_D2H_PT_SETTINGS, new parserDelegate(PT_SETTINGS_Parse) },
                { ICs.IC_D2H_PT_FAILED, new parserDelegate(PT_FAILED_Parse) },
                { ICs.IC_D2H_PT_DLVRD, new parserDelegate(PT_DLVRD_Parse) },
                { ICs.IC_D2H_PT_RCVD, new parserDelegate(PT_RCVD_Parse) },

                { ICs.IC_D2H_CM_HEARD, new parserDelegate(CM_HEARD) },
                { ICs.IC_D2H_PT_HEARD, new parserDelegate(PT_HEARD) },
            };

            #endregion

            #region port
           
            port = new NMEASerialPort(new SerialPortSettings(portName, 
                portBaudrate, 
                System.IO.Ports.Parity.None, 
                DataBits.dataBits8, 
                System.IO.Ports.StopBits.One, 
                System.IO.Ports.Handshake.None));

            port.NewNMEAMessage += new EventHandler<NewNMEAMessageEventArgs>(port_NewNMEAMessageReceived);
            port.PortError += new EventHandler<SerialErrorReceivedEventArgs>(port_ErrorReceived);
            port.RawDataReceived = new EventHandler<RawDataReceivedEventArgs>(port_RawDataReceived);

            #endregion

            #region timer

            timer = new PrecisionTimer();
            timer.Mode = Mode.Periodic;
            timer.Period = 1000;
            timer.Tick += new EventHandler(timer_Tick);

            #endregion

            #region Misc.

            InitDeviceInformation();

            #endregion
        }

        #endregion

        #region Methods

        #region Private

        private uLBLPingerDataIDs PingerDataDecode(int data, out double value)
        {
            uLBLPingerDataIDs result = uLBLPingerDataIDs.Invalid;
            value = double.NaN;

            if (data < uWAVE.CR_TMP_OFFSET)
            {
                value = (data - uWAVE.CR_DPT_OFFSET) * uWAVE.CR_DPT_SCALE + uWAVE.MIN_DEPTH_M;
                result = uLBLPingerDataIDs.DPT;
            }
            else if (data < uWAVE.CR_BAT_OFFSET)
            {
                value = (data - uWAVE.CR_TMP_OFFSET) * uWAVE.CR_TMP_SCALE + uWAVE.MIN_TEMPERATURE_C;
                result = uLBLPingerDataIDs.TMP;
            }
            else if (data < uWAVE.CR_BAT_OFFSET + uWAVE.CR_BAT_CRANGE)
            {
                value = (data - uWAVE.CR_BAT_OFFSET) * uWAVE.CR_BAT_SCALE + uWAVE.MIN_BAT_VOLTAGE;
                result = uLBLPingerDataIDs.BAT;
            }

            return result;
        }
        
        private void InitDeviceInformation()
        {
            SystemMoniker = string.Empty;
            SystemVersion = string.Empty;
            CoreMoniker = string.Empty;
            CoreVersion = string.Empty;
            AcousticBaudrate = double.NaN;
            RxChID = -1;
            TxChID = -1;
            TotalCodeChannels = -1;
            SalinityPSU = double.NaN;

            Pressure_mBar = double.NaN;
            Temperature_C = double.NaN;
            Depth_m = double.NaN;
            SupplyVoltage_V = double.NaN;

            IsWaitingRemote = false;
            IsWaitingLocal = false;
        }    
      
        private bool TrySend(string message, ICs queryID)
        {
            bool result = IsCommandMode && port.IsOpen && !IsWaitingLocal;

            if (result)
            {
                try
                {
                    port.SendData(message);
                    IsWaitingLocal = true;
                    lastQueryID = queryID;
                    result = true;
                    OnInfoEvent(string.Format("<< {0}", message));
                }
                catch (Exception ex)
                {
                    OnInfoEvent(ex);
                }
            }
            
            return result;
        }
        
        private void OnInfoEvent(Exception ex)
        {
            OnInfoEvent(string.Format("{0}, {1}", ex.Message, ex.TargetSite));
        }

        private void OnInfoEvent(string msg)
        {
            InfoEvent.Rise(this, new LogEventArgs(LogLineType.INFO, msg));
        }

        #region Parsers

        private void ACK_Parse(object[] parameters)
        {            
            try
            {
                ICs sntID = uWAVE.ICsByMessageID((string)parameters[0]);
                LocalError_Enum errID = (LocalError_Enum)(int)parameters[1];

                IsWaitingLocal = false;

                if (sntID == ICs.IC_H2D_RC_REQUEST)
                    IsWaitingRemote = true;
                
                ACKReceived.Rise(this, new ACKReceivedEventArgs(sntID, errID));
            }
            catch (Exception ex)
            {
                OnInfoEvent(ex);
            }
        }

        private void AMB_DTA_Parse(object[] parameters)
        {
            try
            {
                // IC_D2H_AMB_DTA         $PUWV7,prs_mBar,temp_C,dpt_m,batVoltage_V
                double prs = doubleNullChecker(parameters[0]);
                double temp = doubleNullChecker(parameters[1]);
                double dpt = doubleNullChecker(parameters[2]);
                double vcc = doubleNullChecker(parameters[3]);

                Pressure_mBar = prs;
                Temperature_C = temp;
                Depth_m = dpt;
                SupplyVoltage_V = vcc;

                AMBDataReceived.Rise(this, new EventArgs());
            }
            catch (Exception ex)
            {
                OnInfoEvent(ex);
            }
        }

        private void DINFO_Parse(object[] parameters)
        {
            try
            {
                // $PUWV!,serial,sys_moniker,sys_version,core_moniker [release],core_version,acBaudrate,rxChID,txChID,maxChannels,styPSU,isPTS,isCmdMode                
                var serialNumber = stringNullChecker(parameters[0]);
                var sysMoniker = stringNullChecker(parameters[1]);
                var sysVersion = uWAVE.BCDVersionToStr((int)parameters[2]);
                var creMoniker = stringNullChecker(parameters[3]);
                var creVersion = uWAVE.BCDVersionToStr((int)parameters[4]);
                var acBaudrate = doubleNullChecker(parameters[5]);
                var rxChID = (int)parameters[6];
                var txChID = (int)parameters[7];
                var totalCh = (int)parameters[8];
                var styPSU = doubleNullChecker(parameters[9]);
                int isPTSFlag = (int)parameters[10];

                bool isPTS = false;
                if (isPTSFlag == 0)
                    isPTS = false;
                else
                    isPTS = true;

                var isCmdMode = Convert.ToBoolean((int)parameters[11]);

                IsWaitingLocal = false;

                SerialNumber = serialNumber;
                SystemMoniker = sysMoniker;
                SystemVersion = sysVersion;
                CoreMoniker = creMoniker;
                CoreVersion = creVersion;
                AcousticBaudrate = acBaudrate;
                RxChID = rxChID;
                TxChID = txChID;
                TotalCodeChannels = totalCh;
                SalinityPSU = styPSU;
                IsPTS = isPTS;
                IsCommandModeByDefault = isCmdMode;
                
                DeviceInfoReceived.Rise(this, new EventArgs());
            }
            catch (Exception ex)
            {
                OnInfoEvent(ex);
            }
        }

        private void RC_ASYNC_IN_Parse(object[] parameters)
        {
            try
            {
                // IC_D2H_RC_ASYNC_IN     $PUWV5,rcCmdID,snr,[azimuth]
                RC_CODES_Enum rcCmdID = (RC_CODES_Enum)(int)parameters[0];
                double snr = doubleNullChecker(parameters[1]);
                double azimuth = doubleNullChecker(parameters[2]);

                RCAsyncInReceived.Rise(this, new RCAsyncInReceivedEventArgs(rcCmdID, snr, azimuth));
            }
            catch (Exception ex)
            {
                OnInfoEvent(ex);
            }
        }

        private void RC_RESPONSE_Parse(object[] parameters)
        {
            try
            {
                // IC_D2H_RC_RESPONSE     $PUWV3,txChID,rcCmdID,propTime_seс,snr,[value],[azimuth]
                int txChID = (int)parameters[0];
                RC_CODES_Enum rcCmdID = (RC_CODES_Enum)(int)parameters[1];
                double pTime = (double)parameters[2];
                double snr = (double)parameters[3];
                double value = doubleNullChecker(parameters[4]);
                double azimuth = doubleNullChecker(parameters[5]);

                IsWaitingRemote = false;

                RCResponseReceived.Rise(this, new RCResponseReceivedEventArgs(txChID, rcCmdID, pTime, snr, value, azimuth));
            }
            catch (Exception ex)
            {
                OnInfoEvent(ex);
            }            
        }

        private void RC_TIMEOUT_Parse(object[] parameters)
        {          
            try
            {
                // IC_D2H_RC_TIMEOUT      $PUWV4,txChID,rcCmdID
                int txChID = (int)parameters[0];
                RC_CODES_Enum rcCmdID = (RC_CODES_Enum)(int)parameters[1];

                IsWaitingRemote = false;

                RCTimeoutReceived.Rise(this, new RCTimeoutReceivedEventArgs(txChID, rcCmdID));
            }
            catch (Exception ex)
            {
                OnInfoEvent(ex);
            }            
        }



        private void PT_SETTINGS_Parse(object[] parameters)
        {
            try
            {
                bool isPTMode = Convert.ToBoolean(intNullChecker(parameters[0]));
                byte ptAddress = Convert.ToByte(intNullChecker(parameters[1]));

                IsPacketMode = isPTMode;
                PacketMode_Address = ptAddress;

                IsWaitingLocal = false;

                PacketModeSettingsReceived.Rise(this, new EventArgs());
            }
            catch (Exception ex)
            {
                OnInfoEvent(ex);
            }
        }

        private void PT_FAILED_Parse(object[] parameters)
        {
            try
            {
                byte target_ptAddress = Convert.ToByte(intNullChecker(parameters[0]));
                byte triesTaken = Convert.ToByte(intNullChecker(parameters[1]));
                byte[] dataPacket = (byte[])(parameters[2]);

                PacketTransferFailed.Rise(this, new PacketEventArgs(target_ptAddress, triesTaken, dataPacket));
            }
            catch (Exception ex)
            {
                OnInfoEvent(ex);
            }
        }
        
        private void PT_DLVRD_Parse(object[] parameters)
        {
            // $PUWVI,tareget_ptAddress,triesTaken,dataPacket
            try
            {
                byte target_ptAddress = Convert.ToByte(intNullChecker(parameters[0]));
                byte triesTaken = Convert.ToByte(intNullChecker(parameters[1]));
                byte[] dataPacket = (byte[])parameters[2];

                PacketTransferred.Rise(this, new PacketEventArgs(target_ptAddress, triesTaken, dataPacket));
            }
            catch (Exception ex)
            {
                OnInfoEvent(ex);
            }
        }

        private void PT_RCVD_Parse(object[] parameters)
        {
            // $PUWVJ,sender_ptAddress,dataPacket
            try
            {
                byte sender_ptAddress = Convert.ToByte(intNullChecker(parameters[0]));
                byte[] dataPacket = (byte[])parameters[1];

                PacketReceived.Rise(this, new PacketReceivedEventArgs(sender_ptAddress, dataPacket));
            }
            catch (Exception ex)
            {
                OnInfoEvent(ex);
            }
        }


        
        private void CM_HEARD(object[] parameters)
        {
            // $PUWVL,b_id,b_lat,b_lon,b_toa_s,data_ID,dataValue,msr_dB
            try
            {
                int b_id = intNullChecker(parameters[0]);
                double b_lat = doubleNullChecker(parameters[1]);
                double b_lon = doubleNullChecker(parameters[2]);
                double b_toa_s = doubleNullChecker(parameters[3]);
                RC_CODES_Enum dataID = (RC_CODES_Enum)intNullChecker(parameters[4]);
                double dataValue = doubleNullChecker(parameters[5]);
                double msr_dB = doubleNullChecker(parameters[6]);

                throw new NotImplementedException();
            }
            catch (Exception ex)
            {
                OnInfoEvent(ex);
            }
        }

        private void PT_HEARD(object[] parameters)
        {
            // $PUWVK,b_id,b_lat,b_lon,b_toa_s,target_pt_Address,sender_pt_Address,dataPacket

            try
            {
                int b_id = intNullChecker(parameters[0]);
                double b_lat = doubleNullChecker(parameters[1]);
                double b_lon = doubleNullChecker(parameters[2]);
                double b_toa_s = doubleNullChecker(parameters[3]);
                byte target_ptAddress = Convert.ToByte(intNullChecker(parameters[4]));
                byte sender_ptAddress = Convert.ToByte(intNullChecker(parameters[5]));
                byte[] dataPacket = (byte[])parameters[6];


                throw new NotImplementedException();
            }
            catch (Exception ex)
            {
                OnInfoEvent(ex);
            }
        }


        #endregion

        #endregion

        #region Public

        public void Open()
        {
            InitDeviceInformation();
            port.Open();
            timer.Start();
        }

        public void Close()
        {
            timer.Stop();
            port.Close();
        }


        public void RawDataSend(byte[] data)
        {
            port.SendRaw(data);
        }

        public bool DeviceInfoQuery()
        {
            var msg = NMEAParser.BuildProprietarySentence(ManufacturerCodes.UWV, "?", new object[] { 0 });
            return TrySend(msg, ICs.IC_H2D_DINFO_GET);            
        }

        public bool SettingsWriteQuery(int txChID, int rxChID, double salinityPSU, bool isCmdMode, bool isACKOnTXFinished, double gravityAcc)
        {
            var msg = NMEAParser.BuildProprietarySentence(ManufacturerCodes.UWV, "1", new object[] 
            { 
                txChID, 
                rxChID, 
                salinityPSU, 
                Convert.ToInt32(isCmdMode),
                Convert.ToInt32(isACKOnTXFinished),
                gravityAcc
            });

            return TrySend(msg, ICs.IC_H2D_SETTINGS_WRITE);
        }

        public bool RCRequestQuery(int txChID, int rxChID, RC_CODES_Enum cmdID)
        {
            if (!IsWaitingRemote)
            {
                var msg = NMEAParser.BuildProprietarySentence(ManufacturerCodes.UWV, "2", new object[] 
                {
                    txChID,
                    rxChID,
                    (int)cmdID 
                });

                return TrySend(msg, ICs.IC_H2D_RC_REQUEST);
            }
            else
            {
                OnInfoEvent("Unable to perform a remote request due to waiting for previous");
                return false;
            }
        }

        public bool AMBDTAConfigQuery(bool isSaveToFlash, int periodMs, bool isPressure, bool isTemperature, bool isDepth, bool isVCC)
        {
            var msg = NMEAParser.BuildProprietarySentence(ManufacturerCodes.UWV, "6", new object[] 
            { 
                Convert.ToInt32(isSaveToFlash),
                periodMs,
                Convert.ToInt32(isPressure),
                Convert.ToInt32(isTemperature),
                Convert.ToInt32(isDepth),
                Convert.ToInt32(isVCC) 
            });

            return TrySend(msg, ICs.IC_H2D_AMB_DTA_CFG);
        }


        public bool PacketMode_SettingQuery()
        {
            var msg = NMEAParser.BuildProprietarySentence(ManufacturerCodes.UWV, "D", new object[] { 0 });
            return TrySend(msg, ICs.IC_H2D_PT_SETTINGS_READ);
        }

        public bool PacketMode_SettingsWrite(bool isSaveInFlash, bool isPTMode, byte ptAddress)
        {
            var msg = NMEAParser.BuildProprietarySentence(ManufacturerCodes.UWV, "F", new object[] { Convert.ToInt32(isSaveInFlash), Convert.ToInt32(isPTMode), (int)ptAddress });
            return TrySend(msg, ICs.IC_H2H_PT_SETTINGS_WRITE);
        }


        public bool PacketMode_AbortSend()
        {
            var msg = NMEAParser.BuildProprietarySentence(ManufacturerCodes.UWV, "G", new object[] { null, null, null });
            return TrySend(msg, ICs.IC_H2D_PT_SEND);
        }

        public bool PacketMode_Send(byte target_ptAddress, byte[] data)
        {
            if (data.Length <= uWAVE.PT_MAX_PACKET_SIZE)
            {
                var msg = NMEAParser.BuildProprietarySentence(ManufacturerCodes.UWV, "G", new object[] { (int)target_ptAddress, null, data });
                return TrySend(msg, ICs.IC_H2D_PT_SEND);
            }
            return false;
        }

        public bool PacketMode_Send(byte target_ptAddress, byte maxTries, byte[] data)
        {
            if (data.Length <= uWAVE.PT_MAX_PACKET_SIZE)
            {
                var msg = NMEAParser.BuildProprietarySentence(ManufacturerCodes.UWV, "G", new object[] { (int)target_ptAddress, (int)maxTries, data });
                return TrySend(msg, ICs.IC_H2D_PT_SEND);
            }
            return false;
        }

         
        public void EmulateInput(string message)
        {
            port_NewNMEAMessageReceived(port, new NewNMEAMessageEventArgs(message));
        }

        #endregion

        #endregion

        #region Handlers

        #region port

        private void port_NewNMEAMessageReceived(object sender, NewNMEAMessageEventArgs e)
        {
            bool isParsed = false;
            NMEASentence result = null;

            OnInfoEvent(string.Format(">> {0}", e.Message));

            try
            {
                result = NMEAParser.Parse(e.Message);
                isParsed = true;
            }
            catch (Exception ex)
            {
                OnInfoEvent(string.Format("\"{0}\" caused \"{1}\", TargetSite: {2}", e.Message, ex.Message, ex.TargetSite));
            }

            if (isParsed)
            {
                if (result is NMEAProprietarySentence)
                {
                    NMEAProprietarySentence pResult = (result as NMEAProprietarySentence);

                    if (pResult.Manufacturer == ManufacturerCodes.UWV)
                    {
                        ICs sentenceID = uWAVE.ICsByMessageID(pResult.SentenceIDString);

                        if (sentenceID != ICs.IC_INVALID)
                        {
                            if (parsers.ContainsKey(sentenceID))
                                parsers[sentenceID](pResult.parameters);
                            else
                            {
                                // skip unsupported sentence
                                if (UnknownSentenceReceived != null)
                                    UnknownSentenceReceived.Rise(this, new UnknownSentenceEventArgs(result));
                                else
                                    OnInfoEvent(string.Format("WARNING: unsupported sentence identifier \"{0}\" (\"{1}\") in \"{2}\"", sentenceID, pResult.SentenceIDString, e.Message));
                            }
                        }
                        else
                        {
                            // skip unknown sentence ID
                            if (UnknownSentenceReceived != null)
                                UnknownSentenceReceived.Rise(this, new UnknownSentenceEventArgs(result));
                            else
                                OnInfoEvent(string.Format("WARNING: unsupported sentence identifier \"{0}\" in \"{1}\"", pResult.SentenceIDString, e.Message));
                        }
                    }
                    else
                    {
                        // skip unsupported manufacturer ID
                        if (UnknownSentenceReceived != null)
                            UnknownSentenceReceived.Rise(this, new UnknownSentenceEventArgs(result));
                        else
                            OnInfoEvent(string.Format("WARNING: unsupported manufacturer identifier \"{0}\" in \"{1}\"", pResult.SentenceIDString, e.Message));
                    }
                }
                else
                {
                    // skip standard sentence
                    if (UnknownSentenceReceived != null)
                        UnknownSentenceReceived.Rise(this, new UnknownSentenceEventArgs(result));
                    else
                        OnInfoEvent(string.Format("WARNING: unsupported standard sentence \"{0}\"", e.Message));
                }
            }

        }

        private void port_ErrorReceived(object sender, SerialErrorReceivedEventArgs e)
        {
            PortError.Rise(this, e);
        }

        private void port_RawDataReceived(object sender, RawDataReceivedEventArgs e)
        {
            if (!IsCommandMode)
                RawDataReceived.Rise(this, e);
        }

        #endregion

        #region timer

        private void timer_Tick(object sender, EventArgs e)
        {
            if (IsWaitingLocal)
            {
                if (++waitingLocalCnt > waitingLocalMaxCnt)
                {
                    IsWaitingLocal = false;
                    DeviceTimeout.Rise(this, new DeviceTimeoutEventArgs(lastQueryID));
                    lastQueryID = ICs.IC_INVALID;
                }
            }

            if (IsWaitingRemote)
            {
                if (++waitingRemoteCnt > waitingRemoteMaxCnt)
                {
                    IsWaitingRemote = false;

                    // Device timeout handler, because it probably has caused by poor connection between the modem and this wrapper
                    DeviceTimeout.Rise(this, new DeviceTimeoutEventArgs(ICs.IC_H2D_RC_REQUEST));
                }
            }
        }

        #endregion

        #endregion

        #region Events

        public EventHandler<RawDataReceivedEventArgs> RawDataReceived;
        public EventHandler<SerialErrorReceivedEventArgs> PortError;
        public EventHandler<DeviceTimeoutEventArgs> DeviceTimeout;
        public EventHandler<ACKReceivedEventArgs> ACKReceived;

        public EventHandler DeviceInfoReceived;        
        public EventHandler AMBDataReceived;

        public EventHandler<RCResponseReceivedEventArgs> RCResponseReceived;
        public EventHandler<RCTimeoutReceivedEventArgs> RCTimeoutReceived;
        public EventHandler<RCAsyncInReceivedEventArgs> RCAsyncInReceived;

        public EventHandler<LogEventArgs> InfoEvent;
        public EventHandler<UnknownSentenceEventArgs> UnknownSentenceReceived;

        public EventHandler PacketModeSettingsReceived;
        public EventHandler<PacketEventArgs> PacketTransferFailed;
        public EventHandler<PacketEventArgs> PacketTransferred;
        public EventHandler<PacketReceivedEventArgs> PacketReceived;

        #endregion
        
        #region IDisposable

        public void Dispose()
        {
            Dispose(true);
            GC.SuppressFinalize(this);
        }

        protected virtual void Dispose(bool disposing)
        {            
            if (!this.disposed)
            {        
                if (disposing)
                {
                    if (IsOpen)
                        Close();

                    port.Dispose();
                    timer.Dispose();                    
                }

                disposed = true;
            }
        }

        #endregion
    }
}
