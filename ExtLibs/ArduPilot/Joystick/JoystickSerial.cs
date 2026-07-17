using System;
using System.Globalization;
using System.IO.Ports;
using System.Text;
using System.Threading;

namespace MissionPlanner.Joystick
{
    /// <summary>
    /// Reads 18 RC channels from a serial-connected transmitter (e.g. an EdgeTX radio
    /// running a Lua script that streams channel values over a USB virtual COM port)
    /// and feeds them into the existing joystick RC override pipeline 1:1 - channel N
    /// on the wire always drives RC override channel N, no per-channel remap/expo/reverse.
    /// </summary>
    public class JoystickSerial : JoystickBase
    {
        public const string DevicePrefix = "Serial: ";
        private const int StaleTimeoutMs = 1000;

        private SerialChannelReader reader;
        private volatile bool acquired;
        private bool channelsActive;

        public bool IsConnected => reader != null && reader.IsOpen;
        public bool IsStale { get; private set; } = true;

        public JoystickSerial(Func<MAVLinkInterface> func) : base(func)
        {
            for (int ch = 1; ch <= 18; ch++)
                setAxis(ch, joystickaxis.None);
        }

        public override bool AcquireJoystick(string name)
        {
            var portName = name != null && name.StartsWith(DevicePrefix)
                ? name.Substring(DevicePrefix.Length)
                : name;

            try
            {
                reader = new SerialChannelReader(portName, 115200);
                reader.Start();
                acquired = true;
                return true;
            }
            catch (Exception ex)
            {
                log.Error(ex);
                acquired = false;
                return false;
            }
        }

        public override void UnAcquireJoyStick()
        {
            acquired = false;
            reader?.Stop();
            reader = null;
        }

        public override bool IsJoystickValid()
        {
            return acquired;
        }

        public override int getNumberPOV()
        {
            return 0;
        }

        public override int getNumButtons()
        {
            return 0;
        }

        public override IMyJoystickState GetCurrentState()
        {
            return NullJoystickState.Instance;
        }

        public override void Dispose()
        {
            UnAcquireJoyStick();
        }

        protected override void mainloop()
        {
            while (enabled && IsJoystickValid())
            {
                try
                {
                    Thread.Sleep(20);

                    var currentReader = reader;
                    bool stale = currentReader == null || !currentReader.IsOpen;
                    ushort[] values = null;

                    if (!stale)
                    {
                        var snapshot = currentReader.GetSnapshot();
                        values = snapshot.values;
                        stale = (DateTime.UtcNow - snapshot.lastLineUtc).TotalMilliseconds > StaleTimeoutMs;
                    }

                    IsStale = stale;

                    if (stale)
                    {
                        if (channelsActive)
                        {
                            SetChannelsActive(false);
                            channelsActive = false;
                        }

                        continue;
                    }

                    if (!channelsActive)
                    {
                        SetChannelsActive(true);
                        channelsActive = true;
                    }

                    Interface.MAV.cs.rcoverridech1 = (short)values[0];
                    Interface.MAV.cs.rcoverridech2 = (short)values[1];
                    Interface.MAV.cs.rcoverridech3 = (short)values[2];
                    Interface.MAV.cs.rcoverridech4 = (short)values[3];
                    Interface.MAV.cs.rcoverridech5 = (short)values[4];
                    Interface.MAV.cs.rcoverridech6 = (short)values[5];
                    Interface.MAV.cs.rcoverridech7 = (short)values[6];
                    Interface.MAV.cs.rcoverridech8 = (short)values[7];
                    Interface.MAV.cs.rcoverridech9 = (short)values[8];
                    Interface.MAV.cs.rcoverridech10 = (short)values[9];
                    Interface.MAV.cs.rcoverridech11 = (short)values[10];
                    Interface.MAV.cs.rcoverridech12 = (short)values[11];
                    Interface.MAV.cs.rcoverridech13 = (short)values[12];
                    Interface.MAV.cs.rcoverridech14 = (short)values[13];
                    Interface.MAV.cs.rcoverridech15 = (short)values[14];
                    Interface.MAV.cs.rcoverridech16 = (short)values[15];
                    Interface.MAV.cs.rcoverridech17 = (short)values[16];
                    Interface.MAV.cs.rcoverridech18 = (short)values[17];
                }
                catch (Exception ex)
                {
                    log.Info("JoystickSerial mainloop error " + ex);
                }
            }
        }

        private void SetChannelsActive(bool active)
        {
            for (int ch = 1; ch <= 18; ch++)
                setAxis(ch, active ? joystickaxis.SerialRC : joystickaxis.None);
        }

        /// <summary>
        /// No-op joystick state - JoystickSerial overrides mainloop() entirely and never
        /// consults axis/button state, but the abstract base still requires an instance.
        /// </summary>
        private class NullJoystickState : IMyJoystickState
        {
            public static readonly NullJoystickState Instance = new NullJoystickState();

            public int[] GetSlider() => new[] { 0, 0 };
            public int[] GetPointOfView() => new[] { -1 };
            public bool[] GetButtons() => new bool[128];
            public int AZ => 0;
            public int AY => 0;
            public int AX => 0;
            public int ARz => 0;
            public int ARy => 0;
            public int ARx => 0;
            public int FRx => 0;
            public int FRy => 0;
            public int FRz => 0;
            public int FX => 0;
            public int FY => 0;
            public int FZ => 0;
            public int Rx => 0;
            public int Ry => 0;
            public int Rz => 0;
            public int VRx => 0;
            public int VRy => 0;
            public int VRz => 0;
            public int VX => 0;
            public int VY => 0;
            public int VZ => 0;
            public int X => 0;
            public int Y => 0;
            public int Z => 0;
        }

        /// <summary>
        /// Owns the serial port: opens/reconnects it, assembles newline-delimited lines,
        /// parses the 18-value EdgeTX channel protocol, and hands off the latest converted
        /// PWM values via a thread-safe snapshot.
        /// </summary>
        private class SerialChannelReader
        {
            private readonly SerialPort port;
            private readonly Thread thread;
            private readonly object sync = new object();
            private readonly ushort[] latest = new ushort[18];
            private volatile bool run;
            private volatile bool isOpen;
            private DateTime lastLineUtc = DateTime.MinValue;

            public bool IsOpen => isOpen;

            public SerialChannelReader(string portName, int baud)
            {
                port = new SerialPort(portName, baud, Parity.None, 8, StopBits.One);
                thread = new Thread(mainloop) { IsBackground = true, Name = "JoystickSerial reader" };
            }

            public void Start()
            {
                run = true;
                thread.Start();
            }

            public void Stop()
            {
                run = false;
                try { thread.Join(500); } catch { }
                try { if (port.IsOpen) port.Close(); } catch { }
                isOpen = false;
            }

            public (ushort[] values, DateTime lastLineUtc) GetSnapshot()
            {
                lock (sync)
                {
                    return ((ushort[])latest.Clone(), lastLineUtc);
                }
            }

            private void mainloop()
            {
                var buffer = "";
                var nextRetryUtc = DateTime.MinValue;

                while (run)
                {
                    try
                    {
                        if (!port.IsOpen)
                        {
                            isOpen = false;

                            if (DateTime.UtcNow < nextRetryUtc)
                            {
                                Thread.Sleep(20);
                                continue;
                            }

                            try
                            {
                                port.Open();
                                isOpen = true;
                                buffer = "";
                            }
                            catch
                            {
                                nextRetryUtc = DateTime.UtcNow.AddSeconds(1);
                                Thread.Sleep(20);
                            }

                            continue;
                        }

                        if (port.BytesToRead > 0)
                        {
                            var chunk = new byte[port.BytesToRead];
                            int read = port.Read(chunk, 0, chunk.Length);
                            buffer += Encoding.ASCII.GetString(chunk, 0, read);

                            int newlineIndex;
                            while ((newlineIndex = buffer.IndexOf('\n')) >= 0)
                            {
                                var line = buffer.Substring(0, newlineIndex).TrimEnd('\r');
                                buffer = buffer.Substring(newlineIndex + 1);
                                ParseAndStore(line);
                            }
                        }
                        else
                        {
                            Thread.Sleep(2);
                        }
                    }
                    catch
                    {
                        try { if (port.IsOpen) port.Close(); } catch { }
                        isOpen = false;
                        nextRetryUtc = DateTime.UtcNow.AddSeconds(1);
                    }
                }

                try { if (port.IsOpen) port.Close(); } catch { }
                isOpen = false;
            }

            private void ParseAndStore(string line)
            {
                if (string.IsNullOrWhiteSpace(line))
                    return;

                var tokens = line.Split(',');
                if (tokens.Length != 18)
                    return;

                var values = new ushort[18];
                for (int i = 0; i < 18; i++)
                {
                    if (!int.TryParse(tokens[i], NumberStyles.Integer, CultureInfo.InvariantCulture, out int raw))
                        return;

                    long us = 1500 + (long)raw * 500 / 1024;
                    if (us < 1000) us = 1000;
                    if (us > 2000) us = 2000;
                    values[i] = (ushort)us;
                }

                lock (sync)
                {
                    Array.Copy(values, latest, 18);
                    lastLineUtc = DateTime.UtcNow;
                }
            }
        }
    }
}
