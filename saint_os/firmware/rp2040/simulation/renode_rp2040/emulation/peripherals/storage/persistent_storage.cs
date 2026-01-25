/**
 * Persistent Storage Peripheral for Renode
 *
 * Provides file-backed persistent storage for simulated nodes.
 * Each node can have its own storage file, allowing configuration
 * to survive across simulation restarts.
 *
 * Memory Map (base address 0x50400000):
 *   0x00: Control register (W: save/load/erase)
 *   0x04: Status register (R: valid/busy flags)
 *   0x100-0xFFF: Data buffer (4KB)
 */

using System;
using System.IO;
using Antmicro.Renode.Core;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals.Bus;

namespace Antmicro.Renode.Peripherals.Storage
{
    public class PersistentStorage : IDoubleWordPeripheral, IKnownSize
    {
        public PersistentStorage(IMachine machine, string storagePath = null, string nodeId = "default")
        {
            this.machine = machine;

            // Set up storage directory
            if (string.IsNullOrEmpty(storagePath))
            {
                storagePath = Path.Combine(
                    Environment.GetFolderPath(Environment.SpecialFolder.UserProfile),
                    ".saint_os", "node_storage"
                );
            }

            this.storageDirectory = storagePath;
            dataBuffer = new byte[DATA_SIZE];

            // Create directory if needed
            try
            {
                Directory.CreateDirectory(storageDirectory);
            }
            catch (Exception e)
            {
                this.Log(LogLevel.Error, "Failed to create storage directory: {0}", e.Message);
            }

            // Set node ID (this also sets up the storage file path)
            NodeId = nodeId;
        }

        /// <summary>
        /// Storage directory property - can be set after construction
        /// </summary>
        public string StoragePath
        {
            get { return storageDirectory; }
            set
            {
                storageDirectory = value;
                try
                {
                    Directory.CreateDirectory(storageDirectory);
                }
                catch (Exception e)
                {
                    this.Log(LogLevel.Error, "Failed to create storage directory: {0}", e.Message);
                }
                // Update storage file path
                storageFile = Path.Combine(storageDirectory, $"{nodeId}.bin");
                this.Log(LogLevel.Info, "Storage directory set to: {0}", storageDirectory);
            }
        }

        /// <summary>
        /// Node ID property - can be set after construction to change storage file
        /// </summary>
        public string NodeId
        {
            get { return nodeId; }
            set
            {
                nodeId = value;
                storageFile = Path.Combine(storageDirectory, $"{nodeId}.bin");
                this.Log(LogLevel.Info, "Storage configured for node: {0} -> {1}", nodeId, storageFile);
                // Reload data from new file
                Reset();
            }
        }

        public void Reset()
        {
            Array.Clear(dataBuffer, 0, dataBuffer.Length);
            hasValidData = false;
            isBusy = false;

            // Try to load existing data
            LoadFromFile();
        }

        public uint ReadDoubleWord(long offset)
        {
            // Data buffer read
            if (offset >= DATA_OFFSET && offset < DATA_OFFSET + DATA_SIZE)
            {
                int idx = (int)(offset - DATA_OFFSET);
                if (idx + 4 <= dataBuffer.Length)
                {
                    return (uint)(dataBuffer[idx] | (dataBuffer[idx+1] << 8) |
                                  (dataBuffer[idx+2] << 16) | (dataBuffer[idx+3] << 24));
                }
                return 0;
            }

            switch (offset)
            {
                case REG_CONTROL:
                    return 0; // Write-only

                case REG_STATUS:
                    uint status = 0;
                    if (hasValidData) status |= STATUS_VALID;
                    if (isBusy) status |= STATUS_BUSY;
                    return status;

                default:
                    this.Log(LogLevel.Warning, "Read from unknown offset 0x{0:X}", offset);
                    return 0;
            }
        }

        public void WriteDoubleWord(long offset, uint value)
        {
            // Data buffer write
            if (offset >= DATA_OFFSET && offset < DATA_OFFSET + DATA_SIZE)
            {
                int idx = (int)(offset - DATA_OFFSET);
                if (idx + 4 <= dataBuffer.Length)
                {
                    dataBuffer[idx] = (byte)(value & 0xFF);
                    dataBuffer[idx+1] = (byte)((value >> 8) & 0xFF);
                    dataBuffer[idx+2] = (byte)((value >> 16) & 0xFF);
                    dataBuffer[idx+3] = (byte)((value >> 24) & 0xFF);
                }
                return;
            }

            switch (offset)
            {
                case REG_CONTROL:
                    if ((value & CTRL_SAVE) != 0)
                    {
                        SaveToFile();
                    }
                    if ((value & CTRL_LOAD) != 0)
                    {
                        LoadFromFile();
                    }
                    if ((value & CTRL_ERASE) != 0)
                    {
                        EraseStorage();
                    }
                    break;

                default:
                    this.Log(LogLevel.Warning, "Write to unknown offset 0x{0:X}", offset);
                    break;
            }
        }

        public long Size => 0x1000;

        private void SaveToFile()
        {
            isBusy = true;
            try
            {
                File.WriteAllBytes(storageFile, dataBuffer);
                hasValidData = true;
                this.Log(LogLevel.Info, "Saved {0} bytes to {1}", dataBuffer.Length, storageFile);
            }
            catch (Exception e)
            {
                this.Log(LogLevel.Error, "Failed to save storage: {0}", e.Message);
            }
            isBusy = false;
        }

        private void LoadFromFile()
        {
            isBusy = true;
            try
            {
                if (File.Exists(storageFile))
                {
                    byte[] data = File.ReadAllBytes(storageFile);
                    int copyLen = Math.Min(data.Length, dataBuffer.Length);
                    Array.Copy(data, dataBuffer, copyLen);

                    // Check for valid magic number (SANT = 0x544E4153 in little-endian)
                    uint magic = (uint)(dataBuffer[0] | (dataBuffer[1] << 8) |
                                       (dataBuffer[2] << 16) | (dataBuffer[3] << 24));
                    hasValidData = (magic == 0x53414E54); // "SANT"

                    if (hasValidData)
                    {
                        this.Log(LogLevel.Info, "Loaded {0} bytes from {1}", copyLen, storageFile);
                    }
                    else
                    {
                        this.Log(LogLevel.Warning, "Storage file exists but has invalid magic: 0x{0:X}", magic);
                    }
                }
                else
                {
                    hasValidData = false;
                    this.Log(LogLevel.Debug, "No storage file found at {0}", storageFile);
                }
            }
            catch (Exception e)
            {
                hasValidData = false;
                this.Log(LogLevel.Error, "Failed to load storage: {0}", e.Message);
            }
            isBusy = false;
        }

        private void EraseStorage()
        {
            isBusy = true;
            try
            {
                Array.Clear(dataBuffer, 0, dataBuffer.Length);
                if (File.Exists(storageFile))
                {
                    File.Delete(storageFile);
                }
                hasValidData = false;
                this.Log(LogLevel.Info, "Storage erased");
            }
            catch (Exception e)
            {
                this.Log(LogLevel.Error, "Failed to erase storage: {0}", e.Message);
            }
            isBusy = false;
        }

        // Register offsets
        private const long REG_CONTROL = 0x00;
        private const long REG_STATUS = 0x04;
        private const long DATA_OFFSET = 0x100;
        private const int DATA_SIZE = 0xF00; // ~4KB data area

        // Control bits
        private const uint CTRL_SAVE = (1 << 0);
        private const uint CTRL_LOAD = (1 << 1);
        private const uint CTRL_ERASE = (1 << 2);

        // Status bits
        private const uint STATUS_VALID = (1 << 0);
        private const uint STATUS_BUSY = (1 << 1);

        private readonly IMachine machine;
        private string nodeId;
        private string storageDirectory;
        private string storageFile;
        private readonly byte[] dataBuffer;

        private bool hasValidData;
        private bool isBusy;
    }
}
