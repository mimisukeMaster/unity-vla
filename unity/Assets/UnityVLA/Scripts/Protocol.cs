// Wire format shared with bridge/unity_vla/protocol.py (protocol version 1).
//   uint32 LE frame_len | uint32 LE header_len | header JSON (UTF-8) | blob section
using System;
using System.Collections.Generic;
using System.IO;
using System.Text;

namespace UnityVLA
{
    public sealed class Blob
    {
        public string Name;
        public string DType;   // uint8 | float32 | int32
        public int[] Shape;
        public byte[] Data;
    }

    public sealed class Message
    {
        public Dictionary<string, object> Header = new Dictionary<string, object>();
        public List<Blob> Blobs = new List<Blob>();

        public static Message Ok() => new Message { Header = { ["ok"] = true } };

        public static Message Error(string error) =>
            new Message { Header = { ["ok"] = false, ["error"] = error } };

        public Message Set(string key, object value)
        {
            Header[key] = value;
            return this;
        }

        public Message AddImage(string name, byte[] rgb, int width, int height)
        {
            Blobs.Add(new Blob { Name = name, DType = "uint8", Shape = new[] { height, width, 3 }, Data = rgb });
            return this;
        }
    }

    public static class Protocol
    {
        public const int Version = 1;
        public const int MaxFrame = 64 * 1024 * 1024;

        public static Message Read(Stream s)
        {
            int frameLen = ReadInt(s);
            if (frameLen < 4 || frameLen > MaxFrame) throw new IOException($"bad frame length {frameLen}");
            var body = ReadExact(s, frameLen);
            int headerLen = BitConverter.ToInt32(body, 0);
            var header = Json.ParseObject(Encoding.UTF8.GetString(body, 4, headerLen));
            var msg = new Message { Header = header };
            if (header.TryGetValue("blobs", out var bl) && bl is List<object> descs)
            {
                int baseOff = 4 + headerLen;
                foreach (var o in descs)
                {
                    var d = (Dictionary<string, object>)o;
                    int off = d.I("offset"), len = d.I("length");
                    var data = new byte[len];
                    Buffer.BlockCopy(body, baseOff + off, data, 0, len);
                    var shp = d.Arr("shape");
                    var shape = new int[shp.Count];
                    for (int k = 0; k < shape.Length; k++) shape[k] = Convert.ToInt32(shp[k]);
                    msg.Blobs.Add(new Blob { Name = d.Str("name"), DType = d.Str("dtype"), Shape = shape, Data = data });
                }
                header.Remove("blobs");
            }
            return msg;
        }

        public static void Write(Stream s, Message msg)
        {
            var header = new Dictionary<string, object>(msg.Header);
            int offset = 0;
            if (msg.Blobs.Count > 0)
            {
                var descs = new List<object>();
                foreach (var b in msg.Blobs)
                {
                    var shape = new List<object>();
                    foreach (var x in b.Shape) shape.Add(x);
                    descs.Add(new Dictionary<string, object>
                    {
                        ["name"] = b.Name, ["offset"] = offset, ["length"] = b.Data.Length,
                        ["dtype"] = b.DType, ["shape"] = shape,
                    });
                    offset += b.Data.Length;
                }
                header["blobs"] = descs;
            }
            var hbytes = Encoding.UTF8.GetBytes(Json.Serialize(header));
            int bodyLen = 4 + hbytes.Length + offset;
            var buf = new byte[4 + bodyLen];
            WriteInt(buf, 0, bodyLen);
            WriteInt(buf, 4, hbytes.Length);
            Buffer.BlockCopy(hbytes, 0, buf, 8, hbytes.Length);
            int p = 8 + hbytes.Length;
            foreach (var b in msg.Blobs)
            {
                Buffer.BlockCopy(b.Data, 0, buf, p, b.Data.Length);
                p += b.Data.Length;
            }
            s.Write(buf, 0, buf.Length);
            s.Flush();
        }

        static void WriteInt(byte[] buf, int at, int v)
        {
            buf[at] = (byte)v;
            buf[at + 1] = (byte)(v >> 8);
            buf[at + 2] = (byte)(v >> 16);
            buf[at + 3] = (byte)(v >> 24);
        }

        static int ReadInt(Stream s)
        {
            var b = ReadExact(s, 4);
            return b[0] | (b[1] << 8) | (b[2] << 16) | (b[3] << 24);
        }

        static byte[] ReadExact(Stream s, int n)
        {
            var buf = new byte[n];
            int got = 0;
            while (got < n)
            {
                int r = s.Read(buf, got, n - got);
                if (r <= 0) throw new EndOfStreamException("connection closed");
                got += r;
            }
            return buf;
        }
    }
}
