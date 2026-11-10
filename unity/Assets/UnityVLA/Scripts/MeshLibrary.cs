// Loads the binary meshes written by tools/build_robot_assets.py (format 'UVM1').
using System;
using System.Collections.Generic;
using System.IO;
using UnityEngine;
using UnityEngine.Rendering;

namespace UnityVLA
{
    public static class MeshLibrary
    {
        const string ResourceDir = "UnityVLA/Meshes/";
        static readonly Dictionary<string, Mesh> Cache = new Dictionary<string, Mesh>();
        static readonly Dictionary<Color, Material> Materials = new Dictionary<Color, Material>();
        static int unityWindingSign;   // +1 or -1, learned from a Unity primitive

        public static Mesh Get(string name)
        {
            if (Cache.TryGetValue(name, out var m) && m != null) return m;
            var ta = Resources.Load<TextAsset>(ResourceDir + name);
            if (ta == null) throw new FileNotFoundException($"mesh not found in Resources: {ResourceDir}{name}");
            m = Decode(ta.bytes, name);
            Cache[name] = m;
            return m;
        }

        public static Mesh Decode(byte[] bytes, string name)
        {
            using var br = new BinaryReader(new MemoryStream(bytes));
            var magic = new string(br.ReadChars(4));
            if (magic != "UVM1") throw new InvalidDataException($"{name}: bad magic {magic}");
            int nv = br.ReadInt32();
            int ni = br.ReadInt32();
            var v = new Vector3[nv];
            var n = new Vector3[nv];
            for (int i = 0; i < nv; i++) v[i] = new Vector3(br.ReadSingle(), br.ReadSingle(), br.ReadSingle());
            for (int i = 0; i < nv; i++) n[i] = new Vector3(br.ReadSingle(), br.ReadSingle(), br.ReadSingle());
            var idx = new int[ni];
            for (int i = 0; i < ni; i++) idx[i] = br.ReadInt32();

            if (WindingSign(v, n, idx) != UnityWindingSign())
            {
                for (int t = 0; t < ni; t += 3) (idx[t + 1], idx[t + 2]) = (idx[t + 2], idx[t + 1]);
            }

            var mesh = new Mesh { name = name };
            if (nv > 65535) mesh.indexFormat = IndexFormat.UInt32;
            mesh.vertices = v;
            mesh.normals = n;
            mesh.triangles = idx;
            mesh.RecalculateBounds();
            return mesh;
        }

        /// Sign of (geometric triangle normal . supplied vertex normal), summed over the mesh.
        static int WindingSign(Vector3[] v, Vector3[] n, int[] idx)
        {
            double acc = 0;
            for (int t = 0; t < idx.Length; t += 3)
            {
                Vector3 a = v[idx[t]], b = v[idx[t + 1]], c = v[idx[t + 2]];
                var g = Vector3.Cross(b - a, c - a);
                acc += Vector3.Dot(g, n[idx[t]] + n[idx[t + 1]] + n[idx[t + 2]]);
            }
            return acc >= 0 ? 1 : -1;
        }

        static int UnityWindingSign()
        {
            if (unityWindingSign != 0) return unityWindingSign;
            // The temporary primitive must disappear immediately: a deferred Destroy would leave
            // a 1 m box collider at the origin for the rest of the frame and fling the robot.
            var go = GameObject.CreatePrimitive(PrimitiveType.Cube);
            go.GetComponent<Collider>().enabled = false;
            var m = go.GetComponent<MeshFilter>().sharedMesh;
            unityWindingSign = WindingSign(m.vertices, m.normals, m.triangles);
            UnityEngine.Object.DestroyImmediate(go);
            return unityWindingSign;
        }

        public static Material ColorMaterial(Color c, float smoothness = 0.3f)
        {
            if (Materials.TryGetValue(c, out var mat) && mat != null) return mat;
            mat = new Material(LitShader()) { color = c, name = $"uv_{ColorUtility.ToHtmlStringRGBA(c)}" };
            if (mat.HasProperty("_Glossiness")) mat.SetFloat("_Glossiness", smoothness);
            if (mat.HasProperty("_Smoothness")) mat.SetFloat("_Smoothness", smoothness);
            if (mat.HasProperty("_BaseColor")) mat.SetColor("_BaseColor", c);
            Materials[c] = mat;
            return mat;
        }

        public static Shader LitShader()
        {
            bool srp = GraphicsSettings.currentRenderPipeline != null;
            var s = srp ? Shader.Find("Universal Render Pipeline/Lit") : Shader.Find("Standard");
            return s != null ? s : Shader.Find("Diffuse");
        }

        public static void DestroyAny(UnityEngine.Object o)
        {
            if (o == null) return;
            if (Application.isPlaying) UnityEngine.Object.Destroy(o);
            else UnityEngine.Object.DestroyImmediate(o);
        }
    }
}
