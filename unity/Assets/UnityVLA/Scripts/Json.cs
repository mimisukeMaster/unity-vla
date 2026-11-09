// Minimal JSON reader/writer (no external packages needed).
// Objects -> Dictionary<string, object>, arrays -> List<object>, numbers -> double.
using System;
using System.Collections;
using System.Collections.Generic;
using System.Globalization;
using System.Text;
using UnityEngine;

namespace UnityVLA
{
    public static class Json
    {
        public static object Parse(string text)
        {
            var p = new Parser(text);
            var v = p.ParseValue();
            p.SkipWs();
            if (!p.End) throw new FormatException("trailing characters in JSON");
            return v;
        }

        public static Dictionary<string, object> ParseObject(string text) =>
            Parse(text) as Dictionary<string, object> ?? throw new FormatException("JSON root is not an object");

        public static string Serialize(object value)
        {
            var sb = new StringBuilder(256);
            Write(sb, value);
            return sb.ToString();
        }

        // ---------------------------------------------------------------- typed getters
        public static Dictionary<string, object> Obj(this Dictionary<string, object> d, string key) =>
            d != null && d.TryGetValue(key, out var v) ? v as Dictionary<string, object> : null;

        public static List<object> Arr(this Dictionary<string, object> d, string key) =>
            d != null && d.TryGetValue(key, out var v) ? v as List<object> : null;

        public static double Num(this Dictionary<string, object> d, string key, double def = 0) =>
            d != null && d.TryGetValue(key, out var v) && v != null ? Convert.ToDouble(v, CultureInfo.InvariantCulture) : def;

        public static float F(this Dictionary<string, object> d, string key, float def = 0) => (float)Num(d, key, def);

        public static int I(this Dictionary<string, object> d, string key, int def = 0) => (int)Math.Round(Num(d, key, def));

        public static string Str(this Dictionary<string, object> d, string key, string def = null) =>
            d != null && d.TryGetValue(key, out var v) && v != null ? v.ToString() : def;

        public static bool Bool(this Dictionary<string, object> d, string key, bool def = false) =>
            d != null && d.TryGetValue(key, out var v) && v is bool b ? b : def;

        public static bool Has(this Dictionary<string, object> d, string key) =>
            d != null && d.ContainsKey(key) && d[key] != null;

        public static float[] Floats(this List<object> a)
        {
            if (a == null) return null;
            var r = new float[a.Count];
            for (int i = 0; i < a.Count; i++) r[i] = Convert.ToSingle(a[i], CultureInfo.InvariantCulture);
            return r;
        }

        public static float[] Floats(this Dictionary<string, object> d, string key) => Arr(d, key).Floats();

        public static Vector3 V3(this Dictionary<string, object> d, string key, Vector3 def = default)
        {
            var f = Floats(d, key);
            return f != null && f.Length >= 3 ? new Vector3(f[0], f[1], f[2]) : def;
        }

        public static Quaternion Q(this Dictionary<string, object> d, string key)
        {
            var f = Floats(d, key);
            return f != null && f.Length >= 4 ? new Quaternion(f[0], f[1], f[2], f[3]).normalized : Quaternion.identity;
        }

        public static Color Col(this Dictionary<string, object> d, string key, Color def)
        {
            var f = Floats(d, key);
            if (f == null || f.Length < 3) return def;
            return new Color(f[0], f[1], f[2], f.Length > 3 ? f[3] : 1f);
        }

        public static List<object> ToList(IEnumerable<float> xs)
        {
            var l = new List<object>();
            foreach (var x in xs) l.Add((double)x);
            return l;
        }

        public static List<object> ToList(Vector3 v) => new List<object> { (double)v.x, (double)v.y, (double)v.z };

        public static List<object> ToList(Quaternion q) =>
            new List<object> { (double)q.x, (double)q.y, (double)q.z, (double)q.w };

        // ---------------------------------------------------------------- writer
        static void Write(StringBuilder sb, object v)
        {
            switch (v)
            {
                case null: sb.Append("null"); break;
                case string s: WriteString(sb, s); break;
                case bool b: sb.Append(b ? "true" : "false"); break;
                case float f: WriteNumber(sb, f); break;
                case double d: WriteNumber(sb, d); break;
                case int i: sb.Append(i.ToString(CultureInfo.InvariantCulture)); break;
                case long l: sb.Append(l.ToString(CultureInfo.InvariantCulture)); break;
                case IDictionary dict:
                    sb.Append('{');
                    bool first = true;
                    foreach (DictionaryEntry e in dict)
                    {
                        if (!first) sb.Append(',');
                        first = false;
                        WriteString(sb, e.Key.ToString());
                        sb.Append(':');
                        Write(sb, e.Value);
                    }
                    sb.Append('}');
                    break;
                case IEnumerable list:
                    sb.Append('[');
                    bool f0 = true;
                    foreach (var item in list)
                    {
                        if (!f0) sb.Append(',');
                        f0 = false;
                        Write(sb, item);
                    }
                    sb.Append(']');
                    break;
                default:
                    WriteString(sb, v.ToString());
                    break;
            }
        }

        static void WriteNumber(StringBuilder sb, double d)
        {
            if (double.IsNaN(d) || double.IsInfinity(d)) { sb.Append("null"); return; }
            sb.Append(d.ToString("R", CultureInfo.InvariantCulture));
        }

        static void WriteString(StringBuilder sb, string s)
        {
            sb.Append('"');
            foreach (var c in s)
            {
                switch (c)
                {
                    case '"': sb.Append("\\\""); break;
                    case '\\': sb.Append("\\\\"); break;
                    case '\n': sb.Append("\\n"); break;
                    case '\r': sb.Append("\\r"); break;
                    case '\t': sb.Append("\\t"); break;
                    default:
                        if (c < 0x20) sb.Append("\\u").Append(((int)c).ToString("x4"));
                        else sb.Append(c);
                        break;
                }
            }
            sb.Append('"');
        }

        // ---------------------------------------------------------------- parser
        class Parser
        {
            readonly string s;
            int i;
            public Parser(string text) { s = text; }
            public bool End => i >= s.Length;

            public void SkipWs()
            {
                while (i < s.Length && char.IsWhiteSpace(s[i])) i++;
            }

            public object ParseValue()
            {
                SkipWs();
                if (End) throw new FormatException("unexpected end of JSON");
                char c = s[i];
                if (c == '{') return ParseObj();
                if (c == '[') return ParseArr();
                if (c == '"') return ParseStr();
                if (c == 't') { Expect("true"); return true; }
                if (c == 'f') { Expect("false"); return false; }
                if (c == 'n') { Expect("null"); return null; }
                return ParseNum();
            }

            void Expect(string w)
            {
                if (string.CompareOrdinal(s, i, w, 0, w.Length) != 0) throw new FormatException($"expected {w} at {i}");
                i += w.Length;
            }

            Dictionary<string, object> ParseObj()
            {
                var d = new Dictionary<string, object>();
                i++;
                SkipWs();
                if (s[i] == '}') { i++; return d; }
                while (true)
                {
                    SkipWs();
                    var k = ParseStr();
                    SkipWs();
                    if (s[i] != ':') throw new FormatException($"expected ':' at {i}");
                    i++;
                    d[k] = ParseValue();
                    SkipWs();
                    if (s[i] == ',') { i++; continue; }
                    if (s[i] == '}') { i++; return d; }
                    throw new FormatException($"expected ',' or '}}' at {i}");
                }
            }

            List<object> ParseArr()
            {
                var l = new List<object>();
                i++;
                SkipWs();
                if (s[i] == ']') { i++; return l; }
                while (true)
                {
                    l.Add(ParseValue());
                    SkipWs();
                    if (s[i] == ',') { i++; continue; }
                    if (s[i] == ']') { i++; return l; }
                    throw new FormatException($"expected ',' or ']' at {i}");
                }
            }

            string ParseStr()
            {
                if (s[i] != '"') throw new FormatException($"expected string at {i}");
                i++;
                var sb = new StringBuilder();
                while (true)
                {
                    char c = s[i++];
                    if (c == '"') return sb.ToString();
                    if (c != '\\') { sb.Append(c); continue; }
                    char e = s[i++];
                    switch (e)
                    {
                        case '"': sb.Append('"'); break;
                        case '\\': sb.Append('\\'); break;
                        case '/': sb.Append('/'); break;
                        case 'b': sb.Append('\b'); break;
                        case 'f': sb.Append('\f'); break;
                        case 'n': sb.Append('\n'); break;
                        case 'r': sb.Append('\r'); break;
                        case 't': sb.Append('\t'); break;
                        case 'u':
                            sb.Append((char)Convert.ToInt32(s.Substring(i, 4), 16));
                            i += 4;
                            break;
                        default: throw new FormatException($"bad escape at {i}");
                    }
                }
            }

            object ParseNum()
            {
                int start = i;
                while (i < s.Length && "+-0123456789.eE".IndexOf(s[i]) >= 0) i++;
                if (start == i) throw new FormatException($"unexpected character '{s[i]}' at {i}");
                return double.Parse(s.Substring(start, i - start), NumberStyles.Float, CultureInfo.InvariantCulture);
            }
        }
    }
}
