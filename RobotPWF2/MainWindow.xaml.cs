using System;
using System.Collections.Generic;
using System.Drawing;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;
using System.IO;
using System.IO.Ports;
using System.Timers;
using System.Windows.Threading;
using Microsoft.Win32;
using ExtendedSerialPort;
using GeometryElements;
using Lidar;
using LidarProcessor;
using MathNet.Numerics;
using Utilities;
using ZedGraph;
using ZedGraphNavigatorDll;

namespace RobotPWF2
{
    /// <summary>
    /// Logique d'interaction pour MainWindow.xaml
    /// </summary>
    public partial class MainWindow : System.Windows.Window
    {
        private ReliableSerialPort Sp1;
        private DispatcherTimer timerByteReceived = new DispatcherTimer(DispatcherPriority.Send);
        private DispatcherTimer timerStrat = new DispatcherTimer(DispatcherPriority.Send);
        private DispatcherTimer timerDisplay = new DispatcherTimer(DispatcherPriority.Send);
        private ZedGraphNavigator graph = new ZedGraphNavigator();
        private LidarDataProcessor lidarDataProcessor = new LidarDataProcessor();

        private SickLidar frontLidar = new SickLidar(18240595);         //18341184);       //18240595);    //17422959); //17422959 //17361121
        private Robot robotState = new Robot();

        private Queue<byte> byteListReceived = new Queue<byte>();
        private string ComPort = "COM5";
        private string ReceivedData;
        private int BufferSize = 128;
        private int timestamp = 0;

        public MainWindow()
        {
            InitializeComponent();

            Sp1 = new ReliableSerialPort(ComPort, 115200, Parity.None, 8, StopBits.One); 
            Sp1.Open();

            //Abonnements aux events
            Sp1.DataReceived += Sp1_DataReceived;
            Message.msgReady += SendEncodedMessage;
            timerByteReceived.Tick += TimerByteReceived_Tick;
            timerStrat.Tick += TimerStrat_Tick;
            timerDisplay.Tick += TimerDisplay_Tick;
            frontLidar.PointsAvailable += lidarDataProcessor.OnPointsAvailableReceived;
            lidarDataProcessor.OnProcessedDataEvent += OnProcessedDataReceived;

            
            InitTimer(1, timerByteReceived);
            InitTimer(50, timerStrat);
            InitTimer(15, timerDisplay);

            frontLidar.AngleMin = -120;
            frontLidar.AngleMax = 120;
            frontLidar.IsUpsideDown = true;
            frontLidar.Start();

            robotState.VitesseDroite = 0;
            robotState.VitesseGauche = 0;

            RollingPointPairList BallPointList = new RollingPointPairList(20);
            graph.PointListCreate("lidar1", new PointPairList(), System.Drawing.Color.Red , 2, false, 2);
            graph.PointListCreate("lidar2", new PointPairList(), System.Drawing.Color.Blue, 5, false, 5);
            graph.PointListCreate("balle", BallPointList, System.Drawing.Color.Red, 4, false, 40);
            graph.PointListCreate("objet", new PointPairList(), System.Drawing.Color.Brown, 4, false, 20);
            graph.PointListCreate("robot", new PointPairList(), System.Drawing.Color.Black, 0, false, 5);
            graph.PointListAddSingleData("robot", 0, 0);
            graph.SetFixedXScale(-2, 2);
            graph.SetFixedYScale(-1, 1);

            byte[] msg = new byte[] { (byte)LedID.LedOrange, 1 };
            Message.UartEncodeMessage((byte)CommandID.Led, msg.Length, msg);
            Message.UartEncodeMessage((int)CommandID.SetRobotAutoControl, 1, new byte[] { 0 });    
            Message.UartEncodeSpeedMsg(robotState.VitesseGauche, robotState.VitesseDroite);
        }

        #region Methods
        private void InitTimer(int intervalMilliS, DispatcherTimer timer)
        {
            timer.Interval = new TimeSpan(0, 0, 0, 0, intervalMilliS);
            timer.Start();
        }
        private void SendText()
        {
            //textBoxReception.Text += "Reçu : " + textBoxEmission.Text + "\n";
            Message.UartEncodeMessage((int)CommandID.Text, textBoxEmission.Text.Length, new ASCIIEncoding().GetBytes(textBoxEmission.Text));
            textBoxEmission.Clear();
        }
        private void RobotStop()
        {
            Message.UartEncodeSpeedMsg(0, 0);
        }

        Stopwatch sw = new Stopwatch();
        StateRobot stateRobot;
        BallState headingState = BallState.NotCentered ;
        double seuilCentre = 7.5 * Math.PI / 180;
        double corrI = 0;
        double theta_1 = 0;
        double dt = 1.0 / 15;
        private void HeadToTarget(double theta)
        {
            //if (Math.Abs(theta) > seuilCentre)
            //    headingState = BallState.NotCentered;
            //else
            //    headingState = BallState.Centered;

            //if (previousHeadingState != headingState) 
            //    corrI = 0;

            //Effet P
            double corrP = Toolbox.LimitToInterval(robotState.Kp * theta, -15, 15);

            //Effet I
            corrI = Toolbox.LimitToInterval(corrI + robotState.Ki * theta * dt, -20, 20);

            //Effet D
            double corrD = robotState.Kd * (theta - theta_1) / dt;
                        
            double corr = corrP + corrI + corrD;
            Console.WriteLine("CorrP : " + corrP + " CorrI : " + corrI + "CorrD : " + corrD + "\r\nCorr : " + corr);

            switch(stateRobot)
            {
                case StateRobot.STATE_ATTENTE:
                    RobotStop();
                    sw.Start();

                    stateRobot = StateRobot.STATE_ATTENTE_EN_COURS;
                    break;

                case StateRobot.STATE_ATTENTE_EN_COURS:
                    if (sw.Elapsed.TotalMilliseconds >= 200) //timestamp - previousTimestamp >= 500)
                    {
                        corrI = 0;
                        CheckHeadingState(theta);

                        if (headingState == BallState.NotCentered)
                            stateRobot = StateRobot.STATE_TOURNE_SUR_PLACE_GAUCHE;
                        else
                            stateRobot = StateRobot.STATE_AVANCE;
                        sw.Reset();
                    }
                    break;

                case StateRobot.STATE_AVANCE:
                    Message.UartEncodeSpeedMsg((int)(23 - corr), (int)(22 + corr));
                    CheckHeadingState(theta);
                    if (headingState != BallState.Centered)
                        stateRobot = StateRobot.STATE_ATTENTE;
                    break;

                case StateRobot.STATE_TOURNE_SUR_PLACE_GAUCHE:
                    Message.UartEncodeSpeedMsg((int)-corr, (int)+corr);
                    CheckHeadingState(theta);
                    if (headingState != BallState.NotCentered)
                        stateRobot = StateRobot.STATE_ATTENTE;
                    break;

                default:
                    stateRobot = StateRobot.STATE_ATTENTE;
                    break;
            }

            //if (headingState == BallState.Centered)
            //{
            //    stateRobot = StateRobot.STATE_AVANCE;
            //    Message.UartEncodeSpeedMsg((int)(25-corr), (int)(22 +corr));
            //}
            //else
            //{
            //    stateRobot = StateRobot.STATE_TOURNE_SUR_PLACE_GAUCHE;
            //    Message.UartEncodeSpeedMsg((int)(-corr), (int)(+corr));
            //}

            ////quand balle à gauche, angle est positif, donc on freine a gauche, sinon négatif, on freine a droite
            //byte consigneDroit;
            //byte consigneGauche;

            //double gain = robotState.Kp * theta;
            //consigneDroit = (byte)Toolbox.LimitToInterval(22 + gain, 0, 44);
            //consigneGauche = (byte)Toolbox.LimitToInterval(22 - gain, 0, 44);

            //Message.UartEncodeSpeedMsg(consigneGauche, consigneDroit);
        }

        private void CheckHeadingState(double theta)
        {
            if (Math.Abs(theta) > seuilCentre)
                headingState = BallState.NotCentered;
            else
                headingState = BallState.Centered;
        }

        bool saveNextLidarPoints = false;
        private void SavePictureAndPointPairList(int objectCount, int ballCount, double[] pointArrayX, double[] pointArrayY, double[] filteredPointArrayX, double[] filteredPointArrayY)
        {
            SaveFileDialog pic = new SaveFileDialog();
            pic.FileName = "listePoints";
            pic.AddExtension = true;
            pic.InitialDirectory = @"C:\Users\" + Environment.UserName + @"\Desktop";
            if (pic.ShowDialog() == true)
            {
                string filePath = pic.FileName;
                graph.SavePicture(filePath);

                StringBuilder sb = new StringBuilder();
                sb.AppendLine("Nombre d'objet : " + objectCount + "\r\nNombre de balles : " + ballCount + "\r\n\r\nCoordonnées des points (données brutes) (x, y)  : ");
                for (int i = 0; i < pointArrayX.Length; i++)
                    sb.AppendLine(pointArrayX[i] + " " + pointArrayY[i]);
                sb.AppendLine("\r\nCoordonnées des points (filtrage des pics) (x, y)  : ");
                for (int i = 0; i < filteredPointArrayX.Length; i++)
                    sb.AppendLine(filteredPointArrayX[i] + " " + filteredPointArrayY[i]);

                try
                {
                    using (FileStream fs = File.Create(filePath + ".txt"))
                    {
                        byte[] frame = new UTF8Encoding(true).GetBytes(sb.ToString());
                        fs.Write(frame, 0, frame.Length);
                    }
                }
                catch (Exception ex)
                {
                    Console.WriteLine(ex.ToString());
                }
            }
        }
        #endregion
        
        #region MessageDecoder
        StateReception rcvState = StateReception.Waiting;
        int msgDecodedFunction = 0;
        int msgDecodedPayloadLength = 0;
        byte[] msgDecodedPayload;
        int msgDecodedpayloadIndex = 0;

        private void DecodedMessage(byte c)
        {
            switch(rcvState)
            {
                case StateReception.Waiting:
                    if (c == 0xFE)
                        rcvState = StateReception.FunctionMSB;
                    break;

                case StateReception.FunctionMSB:
                    msgDecodedFunction = c << 8;
                    rcvState = StateReception.FunctionLSB;
                    break;

                case StateReception.FunctionLSB:
                    msgDecodedFunction += c;
                    rcvState = StateReception.PayloadLengthMSB;
                    break;

                case StateReception.PayloadLengthMSB:
                    msgDecodedPayloadLength = c << 8;
                    rcvState = StateReception.PayloadLengthLSB;
                    break;

                case StateReception.PayloadLengthLSB:
                    msgDecodedPayloadLength += c;
                    msgDecodedPayload = new byte[msgDecodedPayloadLength];
                    rcvState = StateReception.Payload;
                    break;

                case StateReception.Payload:
                    if (msgDecodedPayloadLength >= BufferSize)
                    {
                        msgDecodedFunction = 0;
                        msgDecodedPayloadLength = 0;
                        msgDecodedPayload = null;
                        rcvState = StateReception.Waiting;
                    }
                    else if (msgDecodedPayloadLength > 0)
                    {
                        msgDecodedPayload[msgDecodedpayloadIndex++] = c;
                        if (msgDecodedpayloadIndex >= msgDecodedPayloadLength)
                        {
                            msgDecodedpayloadIndex = 0;
                            rcvState = StateReception.CheckSum;
                        }
                    }
                    else
                        rcvState = StateReception.CheckSum;
                    break;

                case StateReception.CheckSum:
                    byte calculatedChecksum = Message.CalculateChecksum(msgDecodedFunction, msgDecodedPayloadLength, msgDecodedPayload);
                    byte receivedChecksum = c;
                    if (calculatedChecksum == receivedChecksum)
                    {
                        //Success
                        //textBoxReception.Text += "\nTrame ok : ";
                        ProcessDecodedMessage(msgDecodedFunction, msgDecodedPayloadLength, msgDecodedPayload);
                    }
                    else
                    {
                        //textBoxReception.Text += "0\n";
                    }

                    rcvState = StateReception.Waiting;
                    break;

                default:
                    rcvState = StateReception.Waiting;
                    break;
            }
        }

        private void ProcessDecodedMessage(int msgFunction, int msgPayloadLength, byte[] msgPayload)
        {
            switch(msgFunction)
            {
                case (int)CommandID.Led:
                    bool ledState;
                    if (msgPayload[1] == 1)
                        ledState = true;
                    else
                        ledState = false;

                    switch(msgPayload[0])
                    {
                        case (byte)LedID.LedOrange:
                            checkBoxLedOra.IsChecked = ledState;
                            break;

                        case (byte)LedID.LedBleue:
                            checkBoxLedBle.IsChecked = ledState;
                            break;

                        case (byte)LedID.LedBlanche:
                            checkBoxLedBla.IsChecked = ledState;
                            break;
                    }
                    break;

                case (int)CommandID.Ir:
                    labelGauche.Content = msgPayload[0].ToString() + "cm";
                    labelCentreGauche.Content = msgPayload[1].ToString() + "cm";
                    labelCentre.Content = msgPayload[2].ToString() + "cm";
                    labelCentreDroit.Content = msgPayload[3].ToString() + "cm";
                    labelDroit.Content = msgPayload[4].ToString() + "cm";
                    break;

                case (int)CommandID.PWM:
                    labelPWMGauche.Content = msgPayload[0].ToString() + "%";
                    labelPWMDroit.Content = msgPayload[1].ToString() + "%";
                    break;

                case (int)CommandID.Text:
                    foreach (byte b in msgPayload)
                    {
                        ReceivedData = b.ToString("X2");
                        textBoxReception.Text += "0x" + ReceivedData + " ";
                    }
                    break;

                case (int)CommandID.State:
                    int instant = (msgPayload[1] << 24) + (msgPayload[2] << 16) + (msgPayload[3] << 8) + msgPayload[4];
                    textBoxReception.Text += "RobotState = " + ((StateRobot)(msgPayload[0])).ToString() + " _ " + instant.ToString() + " ms" + "\n";
                    break;

                case (int)CommandID.Jack:
                    if (msgPayload[0] == 1)
                        robotState.Jack = true;
                    else
                        robotState.Jack = false;
                    break;
            }
        }
        #endregion

        #region Enums
        public enum LedID
        {
            LedOrange = 0,
            LedBleue = 1,
            LedBlanche = 2
        }

        public enum StateRobot
        {
            STATE_ATTENTE = 0,
            STATE_ATTENTE_EN_COURS = 1,
            STATE_AVANCE = 2,
            STATE_AVANCE_EN_COURS = 3,
            STATE_TOURNE_GAUCHE_FAIBLE = 4,
            STATE_TOURNE_GAUCHE_FAIBLE_EN_COURS = 5,
            STATE_TOURNE_DROITE_FAIBLE = 6,
            STATE_TOURNE_DROITE_FAIBLE_EN_COURS = 7,
            STATE_TOURNE_SUR_PLACE_GAUCHE = 8,
            STATE_TOURNE_SUR_PLACE_GAUCHE_EN_COURS = 9,
            STATE_TOURNE_SUR_PLACE_DROITE = 10,
            STATE_TOURNE_SUR_PLACE_DROITE_EN_COURS = 11,
            STATE_ARRET = 12,
            STATE_ARRET_EN_COURS = 13,
            STATE_RECULE = 14,
            STATE_RECULE_EN_COURS = 15,
            STATE_TOURNE_GAUCHE_FORT = 16,
            STATE_TOURNE_GAUCHE_FORT_EN_COURS = 17,
            STATE_TOURNE_DROITE_FORT = 18,
            STATE_TOURNE_DROITE_FORT_EN_COURS = 19
        }

        public enum StateReception
        {
            Waiting,
            FunctionMSB,
            FunctionLSB,
            PayloadLengthMSB,
            PayloadLengthLSB,
            Payload,
            CheckSum
        }

        public enum BallState
        {
            Centered,
            NotCentered
        }
        #endregion

        #region EventSubs
        private void Window_Loaded(object sender, RoutedEventArgs e)
        {
            // Create the interop host control.
            System.Windows.Forms.Integration.WindowsFormsHost host =
                new System.Windows.Forms.Integration.WindowsFormsHost();

            // Assign the MaskedTextBox control as the host control's child.
            host.Child = graph;
            
            // Add the interop host control to the Grid
            // control's collection of child controls.
            this.groupBoxGraph.Content = host;
        }
        private void Window_Closed(object sender, EventArgs e)
        {
            frontLidar.Stop();
            byte[] msg = new byte[] { (byte)LedID.LedOrange, 0 };
            Message.UartEncodeMessage((byte)CommandID.Led, msg.Length, msg);
        }
        private void CheckBoxLedOra_Click(object sender, RoutedEventArgs e)
        {
            byte[] msg = new byte[] { (byte)LedID.LedOrange, 0 };
            if ((bool)checkBoxLedOra.IsChecked)
                msg[1] = 1;
            else
                msg[1] = 0;
            Message.UartEncodeMessage((byte)CommandID.Led, msg.Length, msg);    
        }
        private void button_Click(object sender, RoutedEventArgs e)
        {
            if(buttonEnvoyer.Background != System.Windows.Media.Brushes.RoyalBlue)
            buttonEnvoyer.Background = System.Windows.Media.Brushes.RoyalBlue;

            else
                buttonEnvoyer.Background = System.Windows.Media.Brushes.Beige;

            SendText();
        }
        private void buttonTest_Click(object sender, RoutedEventArgs e)
        {
            //Test Trame textuelle
            //string s = "Bonjour";
            //byte[] array = Encoding.ASCII.GetBytes(s);

            //Test Trame LED
            //byte[] array = { (byte)LedID.LedOrange, 1 };

            //Test Trame IR
            //byte[] array = { 30, 31, 32, 33, 34 };

            //Test Trame PWM
            //byte[] array = { 50, 30 };

            //Test State
            byte[] array = { 0 };
            Message.UartEncodeMessage((int)CommandID.SetRobotAutoControl, array.Length, array);
            array = new byte[] { (byte)StateRobot.STATE_AVANCE };
            Message.UartEncodeMessage((int)CommandID.SetRobotState, array.Length, array);    
        }
        private void buttonClear_Click(object sender, RoutedEventArgs e)
        {
            textBoxReception.Clear();
            textBoxEmission.Clear();
        }
        private void buttonSaveCurrentGraph_Click(object sender, RoutedEventArgs e)
        {
            saveNextLidarPoints = true;
        }
        private void textBoxEmission_KeyUp(object sender, KeyEventArgs e)
        {
            if(e.Key == Key.Enter)
            {
                SendText();
            }
        }
        private void TimerByteReceived_Tick(object sender, EventArgs e)
        {
            while (byteListReceived.Count() > 0)
            {
                //ReceivedData = byteListReceived.Dequeue().ToString("X2");
                //textBoxReception.Text += "0x" + ReceivedData + " ";
                DecodedMessage(byteListReceived.Dequeue());
            }
            ReceivedData = null;
            timestamp++;
        }
        private void TimerStrat_Tick(object sender, EventArgs e)
        {
            if (robotState.Jack)
            {
                if (frontLidar.IsConnected)
                {
                    frontLidar.Start();
                }
            }
            else
            {
                RobotStop();
                corrI = 0;
                timestamp = 0;
                stateRobot = StateRobot.STATE_ATTENTE;
            }
        }
        private void TimerDisplay_Tick(object sender, EventArgs e)
        {
            lock (robotState)
            {
                graph.PointListClear("lidar1");
                graph.PointListClear("lidar2");
                graph.PointListClear("objet");
                graph.PointListClear("balle");

                List<double> listXpos = new List<double>();
                List<double> listYpos = new List<double>();
                if (robotState.coherentObjectList != null)
                {
                    foreach (CoherentObject co in robotState.coherentObjectList)
                    {
                        listXpos.Add(co.xPos);
                        listYpos.Add(co.yPos);
                    }
                }

                //graph.PointListAddMultipleData("lidar1", robotState.xRawArray.ToList(), robotState.yRawArray.ToList());
                graph.PointListAddMultipleData("lidar2", robotState.xProcessedArray.ToList(), robotState.yProcessedArray.ToList());
                graph.PointListAddMultipleData("objet", listXpos, listYpos);

                if (robotState.ballList != null)
                {
                    foreach (CoherentObject balle in robotState.ballList)
                    {
                        graph.PointListAddSingleData("balle", balle.xPos, balle.yPos);
                    }
                }
            }
        }
        private void Sp1_DataReceived(object sender, DataReceivedArgs e)
        {
            foreach (byte b in e.Data)
            {
                byteListReceived.Enqueue(b);
            }
        }
        private void SendEncodedMessage(object sender, MessageReadyToSendEventArgs e)
        {
            Sp1.Write(e.msg, 0, e.msg.Length);
        }
        private void OnProcessedDataReceived(object sender, ProcessedLidarDataEventArgs e)
        {
            lock (robotState)
            {
                robotState.xRawArray = e.xArrayRaw;
                robotState.yRawArray = e.yArrayRaw;
                robotState.xProcessedArray = e.xArrayProcessed;
                robotState.yProcessedArray = e.yArrayProcessed;
                robotState.coherentObjectList = e.coherentObjects;
            }

            //Filtrage des objets pour trouver les balles
            //Taille d'une balle : 6.5 cm
            double tailleMax = 0.08;
            double tailleMin = 0.03;
            if (robotState.coherentObjectList != null)
            {
                robotState.ballList = robotState.coherentObjectList.Where(r => r.size < tailleMax && r.size > tailleMin).ToList();
                

                CoherentObject closestBall = robotState.ballList.Min();
                if (robotState.Jack)
                {
                    if (closestBall == null)
                        RobotStop();
                    else
                        HeadToTarget(closestBall.angle);
                }
                else
                    RobotStop();

                if (closestBall != null)
                    theta_1 = closestBall.angle;
            }
            //Sauvegarde du graphique courant + données textuelles
            if (saveNextLidarPoints)
            {
                saveNextLidarPoints = false;
                SavePictureAndPointPairList(robotState.coherentObjectList.Count, robotState.ballList.Count, robotState.xRawArray, robotState.yRawArray, robotState.xProcessedArray, robotState.yProcessedArray);
            }
        }
        #endregion
    }
}
