// Synchronous RGB capture (row 0 = top), deterministic and independent of the game view.
using System.Collections.Generic;
using UnityEngine;

namespace UnityVLA
{
    public class CaptureCamera
    {
        public readonly Camera Cam;
        public readonly int Width, Height;
        readonly RenderTexture rt;
        readonly Texture2D tex;
        readonly byte[] flipped;

        public CaptureCamera(string name, Transform parent, int width, int height, float vfov)
        {
            var go = new GameObject(name);
            go.transform.SetParent(parent, false);
            Cam = go.AddComponent<Camera>();
            Cam.fieldOfView = vfov;
            Cam.nearClipPlane = 0.01f;
            Cam.farClipPlane = 20f;
            Cam.enabled = false;   // rendered on demand only
            Width = width;
            Height = height;
            rt = new RenderTexture(width, height, 24, RenderTextureFormat.ARGB32, RenderTextureReadWrite.sRGB)
            {
                antiAliasing = 4, name = name + "_rt",
            };
            tex = new Texture2D(width, height, TextureFormat.RGB24, false, false);
            flipped = new byte[width * height * 3];
            Cam.targetTexture = rt;
        }

        public byte[] Capture()
        {
            var prev = RenderTexture.active;
            Cam.Render();
            RenderTexture.active = rt;
            tex.ReadPixels(new Rect(0, 0, Width, Height), 0, 0, false);
            tex.Apply(false);
            RenderTexture.active = prev;
            var raw = tex.GetRawTextureData<byte>();
            int row = Width * 3;
            for (int y = 0; y < Height; y++)   // Unity: row 0 = bottom -> flip
                Unity.Collections.NativeArray<byte>.Copy(raw, (Height - 1 - y) * row, flipped, y * row, row);
            return (byte[])flipped.Clone();
        }

        public void Destroy()
        {
            if (Cam != null) MeshLibrary.DestroyAny(Cam.gameObject);
            if (rt != null) { rt.Release(); MeshLibrary.DestroyAny(rt); }
            MeshLibrary.DestroyAny(tex);
        }

        public static void Place(Transform t, Vector3 position, Vector3 lookAt, Vector3 up)
        {
            t.localPosition = position;
            var fwd = lookAt - position;
            t.localRotation = fwd.sqrMagnitude > 1e-9f ? Quaternion.LookRotation(fwd.normalized, up) : Quaternion.identity;
        }
    }

    public class CameraRig
    {
        public CaptureCamera Exterior, Wrist;

        public static CameraRig Build(Transform world, Transform hand, Dictionary<string, object> cfg)
        {
            var ex = cfg.Obj("exterior") ?? new Dictionary<string, object>();
            var wr = cfg.Obj("wrist") ?? new Dictionary<string, object>();
            var rig = new CameraRig
            {
                Exterior = new CaptureCamera("exterior_camera", world, ex.I("width", 320), ex.I("height", 180), ex.F("vfov", 62)),
                Wrist = new CaptureCamera("wrist_camera", hand, wr.I("width", 320), wr.I("height", 180), wr.F("vfov", 58)),
            };
            CaptureCamera.Place(rig.Exterior.Cam.transform, ex.V3("position", new Vector3(-0.55f, 0.42f, 0.25f)),
                ex.V3("look_at", new Vector3(0, 0.05f, 0.5f)), Vector3.up);
            // Wrist camera in the gripper frame: y = along the fingers, x = closing axis.
            CaptureCamera.Place(rig.Wrist.Cam.transform, wr.V3("position", new Vector3(0f, 0.035f, -0.075f)),
                wr.V3("look_at", new Vector3(0f, 0.16f, 0.0f)), wr.V3("up", new Vector3(0f, 0f, -1f)));
            return rig;
        }

        public void Destroy()
        {
            Exterior?.Destroy();
            Wrist?.Destroy();
        }
    }
}
