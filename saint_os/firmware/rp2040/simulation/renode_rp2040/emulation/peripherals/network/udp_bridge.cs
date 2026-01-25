/**
 * UDP Bridge Peripheral for Renode
 *
 * Provides a simple memory-mapped interface for UDP communication
 * that bridges to the host network. This allows simulated firmware
 * to send/receive UDP packets without full ethernet emulation.
 *
 * Memory Map (base address 0x50300000):
 *   0x00: Control register (W: send, R: status)
 *   0x04: TX length (W)
 *   0x08: RX length (R)
 *   0x0C: Remote IP (W/R, 4 bytes)
 *   0x10: Remote Port (W/R, 2 bytes)
 *   0x14: Local Port (W)
 *   0x100-0x5FF: TX buffer (1280 bytes)
 *   0x600-0xAFF: RX buffer (1280 bytes)
 */

using System;
using System.Net;
using System.Net.Sockets;
using Antmicro.Renode.Core;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals.Bus;

namespace Antmicro.Renode.Peripherals.Network
{
    public class UDPBridge : IDoubleWordPeripheral, IKnownSize
    {
        public UDPBridge(IMachine machine, ushort localPort = 9999)
        {
            this.machine = machine;
            this.defaultLocalPort = localPort;
            this.localPort = localPort;

            txBuffer = new byte[1280];
            rxBuffer = new byte[1280];

            Reset();
        }

        /// <summary>
        /// Local UDP port property - can be set after construction
        /// </summary>
        public ushort LocalPort
        {
            get { return localPort; }
            set
            {
                if (socketOpen)
                {
                    CloseSocket();
                }
                localPort = value;
                defaultLocalPort = value;
                this.Log(LogLevel.Info, "UDP bridge configured for port: {0}", localPort);
            }
        }

        public void Reset()
        {
            CloseSocket();
            txLength = 0;
            rxLength = 0;
            rxAvailable = false;
            remoteIP = 0;
            remotePort = 0;
            localPort = defaultLocalPort;
            Array.Clear(txBuffer, 0, txBuffer.Length);
            Array.Clear(rxBuffer, 0, rxBuffer.Length);
        }

        public uint ReadDoubleWord(long offset)
        {
            if (offset >= 0x100 && offset < 0x600)
            {
                // TX buffer read (for debugging)
                int idx = (int)(offset - 0x100);
                if (idx + 4 <= txBuffer.Length)
                {
                    return (uint)(txBuffer[idx] | (txBuffer[idx+1] << 8) |
                                  (txBuffer[idx+2] << 16) | (txBuffer[idx+3] << 24));
                }
                return 0;
            }
            else if (offset >= 0x600 && offset < 0xB00)
            {
                // RX buffer read
                int idx = (int)(offset - 0x600);
                if (idx + 4 <= rxBuffer.Length)
                {
                    return (uint)(rxBuffer[idx] | (rxBuffer[idx+1] << 8) |
                                  (rxBuffer[idx+2] << 16) | (rxBuffer[idx+3] << 24));
                }
                return 0;
            }

            switch (offset)
            {
                case 0x00: // Control
                    uint status = 0;
                    if (socketOpen) status |= (1 << 8);
                    if (rxAvailable) status |= (1 << 9);
                    return status;

                case 0x04: // TX Length
                    return (uint)txLength;

                case 0x08: // RX Length
                    return (uint)rxLength;

                case 0x0C: // Remote IP
                    return remoteIP;

                case 0x10: // Remote Port
                    return remotePort;

                case 0x14: // Local Port
                    return localPort;

                default:
                    this.Log(LogLevel.Warning, "Read from unknown offset 0x{0:X}", offset);
                    return 0;
            }
        }

        public void WriteDoubleWord(long offset, uint value)
        {
            if (offset >= 0x100 && offset < 0x600)
            {
                // TX buffer write
                int idx = (int)(offset - 0x100);
                if (idx + 4 <= txBuffer.Length)
                {
                    txBuffer[idx] = (byte)(value & 0xFF);
                    txBuffer[idx+1] = (byte)((value >> 8) & 0xFF);
                    txBuffer[idx+2] = (byte)((value >> 16) & 0xFF);
                    txBuffer[idx+3] = (byte)((value >> 24) & 0xFF);
                }
                return;
            }
            else if (offset >= 0x600 && offset < 0xB00)
            {
                // RX buffer is read-only
                this.Log(LogLevel.Warning, "Attempt to write to RX buffer at offset 0x{0:X}", offset);
                return;
            }

            switch (offset)
            {
                case 0x00: // Control
                    if ((value & 0x01) != 0) SendPacket();  // SEND
                    if ((value & 0x02) != 0) OpenSocket();  // OPEN
                    if ((value & 0x04) != 0) CloseSocket(); // CLOSE
                    if ((value & 0x08) != 0) ReceivePacket(); // RECV
                    break;

                case 0x04: // TX Length
                    txLength = (int)(value & 0xFFFF);
                    break;

                case 0x0C: // Remote IP
                    remoteIP = value;
                    break;

                case 0x10: // Remote Port
                    remotePort = (ushort)(value & 0xFFFF);
                    break;

                case 0x14: // Local Port
                    localPort = (ushort)(value & 0xFFFF);
                    if (socketOpen)
                    {
                        CloseSocket();
                        OpenSocket();
                    }
                    break;

                default:
                    this.Log(LogLevel.Warning, "Write to unknown offset 0x{0:X}, value 0x{1:X}", offset, value);
                    break;
            }
        }

        public long Size => 0x1000;

        private void OpenSocket()
        {
            if (socketOpen) return;

            try
            {
                socket = new UdpClient(localPort);
                socket.Client.ReceiveTimeout = 1; // Non-blocking
                socketOpen = true;
                this.Log(LogLevel.Info, "UDP socket opened on port {0}", localPort);
            }
            catch (Exception e)
            {
                this.Log(LogLevel.Error, "Failed to open UDP socket: {0}", e.Message);
            }
        }

        private void CloseSocket()
        {
            if (!socketOpen) return;

            try
            {
                socket?.Close();
                socket = null;
                socketOpen = false;
                this.Log(LogLevel.Info, "UDP socket closed");
            }
            catch (Exception e)
            {
                this.Log(LogLevel.Error, "Error closing socket: {0}", e.Message);
            }
        }

        private void SendPacket()
        {
            if (!socketOpen || txLength == 0) return;

            try
            {
                byte[] ipBytes = new byte[4];
                ipBytes[0] = (byte)(remoteIP & 0xFF);
                ipBytes[1] = (byte)((remoteIP >> 8) & 0xFF);
                ipBytes[2] = (byte)((remoteIP >> 16) & 0xFF);
                ipBytes[3] = (byte)((remoteIP >> 24) & 0xFF);

                IPEndPoint endpoint = new IPEndPoint(new IPAddress(ipBytes), remotePort);

                byte[] data = new byte[txLength];
                Array.Copy(txBuffer, data, txLength);

                socket.Send(data, txLength, endpoint);
                this.Log(LogLevel.Debug, "Sent {0} bytes to {1}:{2}", txLength, endpoint.Address, remotePort);
            }
            catch (Exception e)
            {
                this.Log(LogLevel.Error, "Send failed: {0}", e.Message);
            }
        }

        private void ReceivePacket()
        {
            if (!socketOpen) return;

            try
            {
                IPEndPoint remoteEP = new IPEndPoint(IPAddress.Any, 0);
                byte[] data = socket.Receive(ref remoteEP);

                rxLength = Math.Min(data.Length, rxBuffer.Length);
                Array.Copy(data, rxBuffer, rxLength);

                // Store sender info
                byte[] ipBytes = remoteEP.Address.GetAddressBytes();
                remoteIP = (uint)(ipBytes[0] | (ipBytes[1] << 8) | (ipBytes[2] << 16) | (ipBytes[3] << 24));
                remotePort = (ushort)remoteEP.Port;

                rxAvailable = true;
                this.Log(LogLevel.Debug, "Received {0} bytes from {1}:{2}", rxLength, remoteEP.Address, remoteEP.Port);
            }
            catch (SocketException)
            {
                // Timeout - no data available
                rxAvailable = false;
                rxLength = 0;
            }
            catch (Exception e)
            {
                this.Log(LogLevel.Error, "Receive failed: {0}", e.Message);
                rxAvailable = false;
                rxLength = 0;
            }
        }

        private readonly IMachine machine;
        private readonly byte[] txBuffer;
        private readonly byte[] rxBuffer;
        private ushort defaultLocalPort;

        private UdpClient socket;
        private bool socketOpen;
        private bool rxAvailable;
        private int txLength;
        private int rxLength;
        private uint remoteIP;
        private ushort remotePort;
        private ushort localPort;
    }
}
