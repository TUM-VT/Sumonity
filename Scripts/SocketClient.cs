using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using System.Threading;
using System.Net;
using System.Net.Sockets;
using System.Text;

namespace tumvt.sumounity
{
    public class SocketClient
    {

        public string connectionIP = "127.0.0.1";
        public int connectionPort = 25001;

        public string messageReceived { get; set; }
        public string messageToSend { get; set; }


        Thread _socketThread;
        IPAddress localAdd;
        TcpListener listener;
        TcpClient client;
        NetworkStream networkStream;
        bool isRunning;

        public SocketClient()
        {
            connectionIP = "127.0.0.1";
            connectionPort = 25001;
        }

        public SocketClient(string ipAdress, int port)
        {
            connectionIP = ipAdress;
            connectionPort = port;
        }


        public void Start()
        {
            // Initialize Thread

            ThreadStart threadStart = new ThreadStart(GetInfo);
            _socketThread = new Thread(threadStart);
            _socketThread.Start();

        }

        public void Close()
        {
            _socketThread.Abort();
        }

        void GetInfo()
        {
            Thread.Sleep(1000); 
            client = new TcpClient();
            client.Connect(connectionIP, connectionPort);

            isRunning = true;
            while (isRunning)
            {
                SendAndReceiveData();
            }
            listener.Stop();
        }

        void SendAndReceiveData()
        {
            networkStream = client.GetStream();
            byte[] buffer = new byte[client.ReceiveBufferSize];

            // ======================
            //      Receive Data
            // ====================== 
            int bytesRead = networkStream.Read(buffer, 0, client.ReceiveBufferSize); //Getting data in Bytes from Python
            string dataReceived = Encoding.UTF8.GetString(buffer, 0, bytesRead); //Converting byte data to string

            if (dataReceived != null)
            {
                ProcessReceivedData();
                messageReceived = dataReceived;
            }

            // ======================
            //      Send Data
            // ======================
            messageToSend = string.IsNullOrEmpty(messageToSend) ? "Empty Message" : messageToSend;
            byte[] myWriteBuffer = Encoding.ASCII.GetBytes(messageToSend); //Converting string to byte data
            networkStream.Write(myWriteBuffer, 0, myWriteBuffer.Length); //Sending the data in Bytes to Python
        }


        void ProcessReceivedData()
        {

        }
    }
}
