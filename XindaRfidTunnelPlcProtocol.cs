using System;
using System.Collections.Generic;
using System.Globalization;
using System.Linq;
using System.Text;

namespace RfidTunnelPlcController;

/// <summary>
/// 信达标准款 RFID 隧道机 PLC 通讯协议工具类。
/// 协议为 Modbus ASCII 风格：: + 地址/功能码/数据十六进制 ASCII + LRC + CRLF。
/// 串口参数：9600bps、7 数据位、偶校验、1 停止位。
/// </summary>
public static class XindaRfidTunnelPlcProtocol
{
    public const byte DefaultPlcAddress = 0x01;

    public const ushort DoorControlRegister = 0x1064;  // 文档 D100 = H1064
    public const ushort ResetRegister = 0x1010;        // 文档 D16 = H1010
    public const ushort BarcodeSuccessRegister = 0x1009;
    public const ushort SensorStartAddress = 0x0400;
    public const ushort SensorPointCount = 0x0010;

    public const int BaudRate = 9600;
    public const int DataBits = 7;
    public const string Parity = "Even";
    public const string StopBits = "One";

    private const byte ReadDiscreteInputs = 0x02;
    private const byte WriteSingleRegister = 0x06;

    /// <summary>
    /// 正常开门。协议指令：:01 06 10 64 00 10 75 CR LF。
    /// </summary>
    public static string BuildNormalOpenDoorCommand(byte plcAddress = DefaultPlcAddress)
        => BuildOpenDoorCommand(0x0010, plcAddress);

    /// <summary>
    /// 异常开门。协议指令：:01 06 10 64 00 11 74 CR LF。
    /// </summary>
    public static string BuildAbnormalOpenDoorCommand(byte plcAddress = DefaultPlcAddress)
        => BuildOpenDoorCommand(0x0011, plcAddress);

    public static string BuildOpenDoorCommand(ushort value, byte plcAddress = DefaultPlcAddress)
        => BuildWriteSingleRegisterCommand(DoorControlRegister, value, plcAddress);

    /// <summary>
    /// 复位。默认写入 D16(H1010)=1；如现场 PLC 值定义不同，请调用重载指定 value。
    /// </summary>
    public static string BuildResetCommand(byte plcAddress = DefaultPlcAddress)
        => BuildResetCommand(0x0001, plcAddress);

    public static string BuildResetCommand(ushort value, byte plcAddress = DefaultPlcAddress)
        => BuildWriteSingleRegisterCommand(ResetRegister, value, plcAddress);

    /// <summary>
    /// 条码扫码成功：文档示例 :01 06 10 09 00 01 DF CR LF。
    /// </summary>
    public static string BuildBarcodeScanSuccessCommand(byte plcAddress = DefaultPlcAddress)
        => BuildWriteSingleRegisterCommand(BarcodeSuccessRegister, 0x0001, plcAddress);

    /// <summary>
    /// X0-X7 / X10-X17 传感器状态查询：
    /// 文档示例 :01 02 04 00 00 10 E9 CR LF。
    /// </summary>
    public static string BuildSensorStatusQueryCommand(byte plcAddress = DefaultPlcAddress)
        => BuildReadDiscreteInputsCommand(SensorStartAddress, SensorPointCount, plcAddress);

    public static string BuildReadDiscreteInputsCommand(
        ushort startAddress,
        ushort pointCount,
        byte plcAddress = DefaultPlcAddress)
    {
        return BuildAsciiFrame(new[]
        {
            plcAddress,
            ReadDiscreteInputs,
            HighByte(startAddress),
            LowByte(startAddress),
            HighByte(pointCount),
            LowByte(pointCount)
        });
    }

    public static string BuildWriteSingleRegisterCommand(
        ushort registerAddress,
        ushort value,
        byte plcAddress = DefaultPlcAddress)
    {
        return BuildAsciiFrame(new[]
        {
            plcAddress,
            WriteSingleRegister,
            HighByte(registerAddress),
            LowByte(registerAddress),
            HighByte(value),
            LowByte(value)
        });
    }

    public static string BuildAsciiFrame(IReadOnlyList<byte> bytesWithoutLrc)
    {
        if (bytesWithoutLrc == null || bytesWithoutLrc.Count == 0)
        {
            throw new ArgumentException("报文内容不能为空。", nameof(bytesWithoutLrc));
        }

        var builder = new StringBuilder(bytesWithoutLrc.Count * 2 + 5);
        builder.Append(':');
        foreach (var b in bytesWithoutLrc)
        {
            builder.Append(b.ToString("X2", CultureInfo.InvariantCulture));
        }

        builder.Append(CalculateLrc(bytesWithoutLrc).ToString("X2", CultureInfo.InvariantCulture));
        builder.Append("\r\n");
        return builder.ToString();
    }

    public static byte CalculateLrc(IReadOnlyList<byte> bytesWithoutLrc)
    {
        if (bytesWithoutLrc == null)
        {
            throw new ArgumentNullException(nameof(bytesWithoutLrc));
        }

        var sum = 0;
        foreach (var b in bytesWithoutLrc)
        {
            sum = (sum + b) & 0xFF;
        }

        return (byte)((0x100 - sum) & 0xFF);
    }

    public static PlcFrame ParseFrame(string frame)
    {
        if (string.IsNullOrWhiteSpace(frame))
        {
            throw new ArgumentException("PLC 返回报文为空。", nameof(frame));
        }

        var normalized = NormalizeFrame(frame);
        if (normalized.Length < 6 || normalized.Length % 2 != 0)
        {
            throw new FormatException("PLC 返回报文长度不正确。");
        }

        var allBytes = HexToBytes(normalized);
        var payload = allBytes.Take(allBytes.Length - 1).ToArray();
        var receivedLrc = allBytes[^1];
        var expectedLrc = CalculateLrc(payload);
        if (receivedLrc != expectedLrc)
        {
            throw new FormatException($"PLC 返回报文 LRC 校验失败，应为 {expectedLrc:X2}，实际为 {receivedLrc:X2}。");
        }

        return new PlcFrame(payload[0], payload[1], payload.Skip(2).ToArray());
    }

    public static bool IsWriteSingleRegisterSuccess(
        string frame,
        ushort expectedRegisterAddress,
        ushort expectedValue,
        byte expectedPlcAddress = DefaultPlcAddress)
    {
        var parsed = ParseFrame(frame);
        if (parsed.IsException || parsed.Address != expectedPlcAddress || parsed.FunctionCode != WriteSingleRegister)
        {
            return false;
        }

        if (parsed.Data.Length != 4)
        {
            return false;
        }

        var registerAddress = ToUInt16(parsed.Data[0], parsed.Data[1]);
        var value = ToUInt16(parsed.Data[2], parsed.Data[3]);
        return registerAddress == expectedRegisterAddress && value == expectedValue;
    }

    public static SensorStatus ParseSensorStatusResponse(string frame, byte expectedPlcAddress = DefaultPlcAddress)
    {
        var parsed = ParseFrame(frame);
        if (parsed.IsException)
        {
            throw new InvalidOperationException($"PLC 异常响应，功能码 0x{parsed.FunctionCode:X2}，异常码 0x{parsed.ExceptionCode:X2}。");
        }

        if (parsed.Address != expectedPlcAddress || parsed.FunctionCode != ReadDiscreteInputs)
        {
            throw new FormatException("PLC 返回的地址或功能码与传感器查询指令不匹配。");
        }

        if (parsed.Data.Length < 3 || parsed.Data[0] != 0x02)
        {
            throw new FormatException("PLC 传感器状态返回格式不正确。");
        }

        return new SensorStatus(parsed.Data[1], parsed.Data[2]);
    }

    private static string NormalizeFrame(string frame)
    {
        var text = frame.Trim();
        if (text.StartsWith(":", StringComparison.Ordinal))
        {
            text = text[1..];
        }

        text = text
            .Replace(" ", string.Empty, StringComparison.Ordinal)
            .Replace("\r", string.Empty, StringComparison.Ordinal)
            .Replace("\n", string.Empty, StringComparison.Ordinal)
            .Replace("\t", string.Empty, StringComparison.Ordinal);

        if (text.StartsWith("0x", StringComparison.OrdinalIgnoreCase))
        {
            text = text[2..];
        }

        return text;
    }

    private static byte[] HexToBytes(string hex)
    {
        var bytes = new byte[hex.Length / 2];
        for (var i = 0; i < bytes.Length; i++)
        {
            bytes[i] = byte.Parse(hex.Substring(i * 2, 2), NumberStyles.HexNumber, CultureInfo.InvariantCulture);
        }

        return bytes;
    }

    private static byte HighByte(ushort value) => (byte)(value >> 8);

    private static byte LowByte(ushort value) => (byte)(value & 0xFF);

    private static ushort ToUInt16(byte high, byte low) => (ushort)((high << 8) | low);
}

public sealed class PlcFrame
{
    public PlcFrame(byte address, byte functionCode, byte[] data)
    {
        Address = address;
        FunctionCode = functionCode;
        Data = data ?? Array.Empty<byte>();
    }

    public byte Address { get; }

    public byte FunctionCode { get; }

    public byte[] Data { get; }

    public bool IsException => (FunctionCode & 0x80) == 0x80;

    public byte ExceptionCode => IsException && Data.Length > 0 ? Data[0] : (byte)0x00;
}

public sealed class SensorStatus
{
    private readonly byte _x0ToX7;
    private readonly byte _x10ToX17;

    public SensorStatus(byte x0ToX7, byte x10ToX17)
    {
        _x0ToX7 = x0ToX7;
        _x10ToX17 = x10ToX17;
    }

    public byte X0ToX7Raw => _x0ToX7;

    public byte X10ToX17Raw => _x10ToX17;

    public bool X0 => GetX(0);

    public bool X1 => GetX(1);

    public bool FrontCargoDetected => GetX(2);

    public bool EntranceDoorOpenInPlace => GetX(3);

    public bool EntranceDoorClosedInPlace => GetX(4);

    public bool ScanFailedCargoStopDetected => GetX(5);

    public bool MiddleTunnelCargoDetected => GetX(6);

    public bool X7 => GetX(7);

    public bool ExitDoorOpenInPlace => GetX(10);

    public bool ExitDoorClosedInPlace => GetX(11);

    /// <summary>文档说明：1 有效未按下，0 按下。</summary>
    public bool EmergencyStopReleased => GetX(12);

    /// <summary>文档说明：1 手动，0 自动。</summary>
    public bool ManualMode => GetX(13);

    public bool X14 => GetX(14);

    public bool X15 => GetX(15);

    public bool X16 => GetX(16);

    public bool ScanTriggerDetected => GetX(17);

    public bool GetX(int point)
    {
        if (point is >= 0 and <= 7)
        {
            return (_x0ToX7 & (1 << point)) != 0;
        }

        if (point is >= 10 and <= 17)
        {
            return (_x10ToX17 & (1 << (point - 10))) != 0;
        }

        throw new ArgumentOutOfRangeException(nameof(point), "仅支持 X0-X7 / X10-X17。");
    }

    public IReadOnlyDictionary<string, bool> ToDictionary()
    {
        return new Dictionary<string, bool>
        {
            ["X0 未定义"] = GetX(0),
            ["X1 未定义"] = GetX(1),
            ["X2 前段货物检测传感器"] = GetX(2),
            ["X3 入口开门到位检测传感器"] = GetX(3),
            ["X4 入口关门到位检测传感器"] = GetX(4),
            ["X5 扫描失败货物停止检测传感器"] = GetX(5),
            ["X6 中段隧道机货物检测传感器"] = GetX(6),
            ["X7 未定义"] = GetX(7),
            ["X10 出口开门到位检测传感器"] = GetX(10),
            ["X11 出口关门到位检测传感器"] = GetX(11),
            ["X12 急停开关状态检测"] = GetX(12),
            ["X13 模式自动手动开关状态检测"] = GetX(13),
            ["X14 未定义"] = GetX(14),
            ["X15 未定义"] = GetX(15),
            ["X16 未定义"] = GetX(16),
            ["X17 扫描触发传感器"] = GetX(17)
        };
    }
}
