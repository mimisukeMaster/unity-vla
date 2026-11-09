// Table-top scene: static environment + per-episode objects placed from a seed, and success checks.
using System;
using System.Collections.Generic;
using UnityEngine;

namespace UnityVLA
{
    public class SceneObject
    {
        public string Name;
        public string Shape;
        public float Size;
        public Rigidbody Body;
        public Vector3 SpawnPosition;
    }

    public class TaskScene
    {
        public Transform Root;
        public readonly Dictionary<string, SceneObject> Objects = new Dictionary<string, SceneObject>();
        public Dictionary<string, object> Goal;
        public float HoldS;
        float heldFor;
        public bool Success { get; private set; }
        public Light Sun;

        // ------------------------------------------------------------------ build
        public static TaskScene Build(Transform world, Dictionary<string, object> cfg, int seed)
        {
            cfg ??= new Dictionary<string, object>();
            var s = new TaskScene { Root = new GameObject("Scene").transform };
            s.Root.SetParent(world, false);
            s.Goal = cfg.Obj("goal") ?? new Dictionary<string, object> { ["type"] = "place", ["target"] = "red_cube", ["container"] = "bowl" };
            s.HoldS = cfg.F("hold_s", 0.5f);
            var rng = new System.Random(seed);

            BuildEnvironment(s, cfg);

            var region = cfg.Obj("region");
            var xr = region?.Floats("x") ?? new[] { -0.22f, 0.22f };
            var zr = region?.Floats("z") ?? new[] { 0.38f, 0.62f };
            float spacing = cfg.F("min_spacing", 0.11f);
            var placed = new List<Vector2>();
            var objs = cfg.Arr("objects") ?? DefaultObjects();
            foreach (var o in objs)
            {
                var O = (Dictionary<string, object>)o;
                Vector2 p = Vector2.zero;
                for (int tries = 0; tries < 200; tries++)
                {
                    p = new Vector2(Lerp(xr, rng.NextDouble()), Lerp(zr, rng.NextDouble()));
                    bool ok = true;
                    foreach (var q in placed) ok &= (q - p).magnitude >= spacing;
                    if (ok) break;
                }
                placed.Add(p);
                float yaw = (float)(rng.NextDouble() * 360.0);
                var so = SpawnObject(s.Root, O, new Vector3(p.x, 0f, p.y), yaw,
                    cfg.Bool("randomize_colors") ? RandomTint(rng) : (Color?)null);
                s.Objects[so.Name] = so;
            }
            return s;
        }

        static float Lerp(float[] r, double u) => (float)(r[0] + (r[1] - r[0]) * u);

        static Color RandomTint(System.Random rng) =>
            new Color(0.85f + 0.15f * (float)rng.NextDouble(), 0.85f + 0.15f * (float)rng.NextDouble(), 0.85f + 0.15f * (float)rng.NextDouble());

        static List<object> DefaultObjects() => new List<object>
        {
            new Dictionary<string, object> { ["name"] = "red_cube", ["shape"] = "cube", ["size"] = 0.05, ["color"] = Json.ToList(new[] { 0.85f, 0.1f, 0.1f }) },
            new Dictionary<string, object> { ["name"] = "bowl", ["shape"] = "bowl", ["size"] = 0.16, ["color"] = Json.ToList(new[] { 0.9f, 0.9f, 0.88f }) },
        };

        static void BuildEnvironment(TaskScene s, Dictionary<string, object> cfg)
        {
            var t = cfg.Obj("table") ?? new Dictionary<string, object>();
            var table = GameObject.CreatePrimitive(PrimitiveType.Cube);
            table.name = "table";
            table.transform.SetParent(s.Root, false);
            table.transform.localPosition = t.V3("center", new Vector3(0, -0.02f, 0.45f));
            table.transform.localScale = t.V3("size", new Vector3(1.2f, 0.04f, 0.8f));
            table.GetComponent<Renderer>().sharedMaterial = MeshLibrary.ColorMaterial(t.Col("color", new Color(0.55f, 0.42f, 0.3f)), 0.2f);

            var floor = GameObject.CreatePrimitive(PrimitiveType.Plane);
            floor.name = "floor";
            floor.transform.SetParent(s.Root, false);
            floor.transform.localPosition = new Vector3(0, -0.75f, 0.3f);
            floor.transform.localScale = new Vector3(1, 1, 1);
            floor.GetComponent<Renderer>().sharedMaterial = MeshLibrary.ColorMaterial(new Color(0.45f, 0.45f, 0.47f), 0.1f);

            foreach (var (pos, scale) in new[]
            {
                (new Vector3(0, 0.5f, 1.6f), new Vector3(4, 2.5f, 0.05f)),
                (new Vector3(-1.8f, 0.5f, 0.3f), new Vector3(0.05f, 2.5f, 3.0f)),
                (new Vector3(1.8f, 0.5f, 0.3f), new Vector3(0.05f, 2.5f, 3.0f)),
            })
            {
                var w = GameObject.CreatePrimitive(PrimitiveType.Cube);
                w.name = "wall";
                w.transform.SetParent(s.Root, false);
                w.transform.localPosition = pos;
                w.transform.localScale = scale;
                w.GetComponent<Renderer>().sharedMaterial = MeshLibrary.ColorMaterial(new Color(0.82f, 0.82f, 0.8f), 0.05f);
                MeshLibrary.DestroyAny(w.GetComponent<Collider>());
            }

            var L = cfg.Obj("lighting") ?? new Dictionary<string, object>();
            var lg = new GameObject("sun");
            lg.transform.SetParent(s.Root, false);
            lg.transform.localRotation = Quaternion.Euler(55f, -30f, 0f);
            s.Sun = lg.AddComponent<Light>();
            s.Sun.type = LightType.Directional;
            s.Sun.intensity = L.F("intensity", 1.1f);
            s.Sun.shadows = L.Bool("shadows", true) ? LightShadows.Soft : LightShadows.None;
            RenderSettings.ambientMode = UnityEngine.Rendering.AmbientMode.Flat;
            RenderSettings.ambientLight = new Color(0.45f, 0.45f, 0.48f);
        }

        static SceneObject SpawnObject(Transform parent, Dictionary<string, object> O, Vector3 floorPos, float yaw, Color? tint)
        {
            string shape = O.Str("shape", "cube");
            float size = O.F("size", 0.05f);
            var color = O.Col("color", Color.white);
            if (tint.HasValue) color *= tint.Value;
            GameObject g;
            float halfHeight;
            if (shape == "bowl")
            {
                g = BuildBowl(size, color);
                halfHeight = 0f;           // bowl origin is at its bottom
            }
            else
            {
                g = GameObject.CreatePrimitive(shape == "sphere" ? PrimitiveType.Sphere : PrimitiveType.Cube);
                g.transform.localScale = Vector3.one * size;
                g.GetComponent<Renderer>().sharedMaterial = MeshLibrary.ColorMaterial(color, 0.35f);
                halfHeight = size * 0.5f;
            }
            g.name = O.Str("name", shape);
            g.transform.SetParent(parent, false);
            g.transform.localPosition = floorPos + Vector3.up * (halfHeight + 0.001f);
            g.transform.localRotation = Quaternion.Euler(0, yaw, 0);
#if UNITY_6000_0_OR_NEWER
            var mat = new PhysicsMaterial("obj") { staticFriction = 0.8f, dynamicFriction = 0.7f };
#else
            var mat = new PhysicMaterial("obj") { staticFriction = 0.8f, dynamicFriction = 0.7f };
#endif
            foreach (var c in g.GetComponentsInChildren<Collider>()) c.sharedMaterial = mat;
            var rb = g.AddComponent<Rigidbody>();
            rb.mass = O.F("mass", 0.08f);
            rb.collisionDetectionMode = CollisionDetectionMode.ContinuousSpeculative;
            rb.interpolation = RigidbodyInterpolation.None;
            return new SceneObject { Name = g.name, Shape = shape, Size = size, Body = rb, SpawnPosition = g.transform.position };
        }

        /// Open bowl: lathe mesh for looks, compound box colliders (base + 16 wall segments).
        static GameObject BuildBowl(float diameter, Color color)
        {
            var g = new GameObject("bowl");
            float R = diameter * 0.5f, H = diameter * 0.35f, wall = 0.006f, rb = R * 0.55f;
            var vis = new GameObject("visual");
            vis.transform.SetParent(g.transform, false);
            vis.AddComponent<MeshFilter>().sharedMesh = LatheBowl(rb, R, H, wall, 32);
            vis.AddComponent<MeshRenderer>().sharedMaterial = MeshLibrary.ColorMaterial(color, 0.5f);

            var bottom = g.AddComponent<BoxCollider>();
            bottom.center = new Vector3(0, wall * 0.5f, 0);
            bottom.size = new Vector3(rb * 1.5f, wall, rb * 1.5f);
            int n = 16;
            float slant = Mathf.Atan2(R - rb, H) * Mathf.Rad2Deg;
            float len = Mathf.Sqrt((R - rb) * (R - rb) + H * H);
            float seg = 2 * Mathf.PI * R / n * 1.05f;
            for (int i = 0; i < n; i++)
            {
                float a = 360f * i / n;
                var w = new GameObject("wall");
                w.transform.SetParent(g.transform, false);
                var dir = Quaternion.Euler(0, a, 0);
                float rm = (R + rb) * 0.5f;
                w.transform.localPosition = dir * new Vector3(0, H * 0.5f, rm);
                w.transform.localRotation = dir * Quaternion.Euler(slant, 0, 0);   // top leans outward
                var bc = w.AddComponent<BoxCollider>();
                bc.size = new Vector3(seg, len, wall);
            }
            return g;
        }

        static Mesh LatheBowl(float rBottom, float rTop, float h, float wall, int seg)
        {
            // profile from outer bottom centre -> outer rim -> inner rim -> inner bottom centre
            var prof = new List<Vector2>
            {
                new Vector2(0, 0), new Vector2(rBottom, 0), new Vector2(rTop, h),
                new Vector2(rTop - wall, h), new Vector2(rBottom - wall, wall), new Vector2(0, wall),
            };
            var v = new List<Vector3>();
            var tri = new List<int>();
            int pc = prof.Count;
            for (int i = 0; i <= seg; i++)
            {
                float a = 2 * Mathf.PI * i / seg;
                float c = Mathf.Cos(a), s = Mathf.Sin(a);
                foreach (var p in prof) v.Add(new Vector3(p.x * c, p.y, p.x * s));
            }
            for (int i = 0; i < seg; i++)
                for (int j = 0; j < pc - 1; j++)
                {
                    int a = i * pc + j, b = a + 1, c = a + pc, d = c + 1;
                    tri.AddRange(new[] { a, b, c, b, d, c });
                }
            var m = new Mesh { name = "bowl" };
            m.SetVertices(v);
            m.SetTriangles(tri, 0);
            m.RecalculateNormals();
            m.RecalculateBounds();
            return m;
        }

        // ------------------------------------------------------------------ success
        public void UpdateSuccess(float dt, Transform pinch, float gripper)
        {
            bool cond = Evaluate(pinch, gripper);
            heldFor = cond ? heldFor + dt : 0f;
            Success = heldFor >= HoldS;
        }

        bool Evaluate(Transform pinch, float gripper)
        {
            string type = Goal.Str("type", "place");
            if (!Objects.TryGetValue(Goal.Str("target", ""), out var target)) return false;
            var p = target.Body.position;
            if (type == "lift")
                return p.y - target.SpawnPosition.y >= Goal.F("height", 0.08f);
            if (type == "place")
            {
                if (!Objects.TryGetValue(Goal.Str("container", "bowl"), out var bowl)) return false;
                var b = bowl.Body.position;
                var dxz = new Vector2(p.x - b.x, p.z - b.z).magnitude;
                bool inside = dxz < bowl.Size * 0.35f && p.y - b.y < bowl.Size * 0.35f && p.y > b.y;
                bool released = gripper < 0.5f || Vector3.Distance(pinch.position, p) > 0.05f;
                #if UNITY_6000_0_OR_NEWER
                bool still = target.Body.linearVelocity.magnitude < 0.05f;
#else
                bool still = target.Body.velocity.magnitude < 0.05f;
#endif
                return inside && released && still;
            }
            return false;
        }

        public Dictionary<string, object> Describe()
        {
            var objs = new Dictionary<string, object>();
            foreach (var kv in Objects)
                objs[kv.Key] = new Dictionary<string, object>
                {
                    ["pos"] = Json.ToList(kv.Value.Body.position),
                    ["rot"] = Json.ToList(kv.Value.Body.rotation),
                };
            return new Dictionary<string, object> { ["goal"] = Goal, ["objects"] = objs, ["held_for"] = (double)heldFor };
        }

        public void Destroy()
        {
            if (Root == null) return;
            Root.gameObject.SetActive(false);   // Destroy is deferred; remove from physics now
            MeshLibrary.DestroyAny(Root.gameObject);
        }
    }
}
