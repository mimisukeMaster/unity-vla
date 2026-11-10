// Lockstep simulation server.  The bridge drives time: physics advances only on "step".
// Requests are received on a background thread and executed on Unity's main thread.
using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Diagnostics;
using System.Net;
using System.Net.Sockets;
using System.Threading;
using UnityEngine;
using Debug = UnityEngine.Debug;

namespace UnityVLA
{
    public class SimServer : MonoBehaviour
    {
        [Header("Network")]
        public int port = 5555;
        public bool listenOnAllInterfaces = false;

        [Header("Robot")]
        public RobotSettings robotSettings = new RobotSettings();

        [Header("Idle preview (no bridge connected)")]
        [Tooltip("Run physics in real time while no bridge is connected")]
        public bool freeRunWhenIdle = true;
        [Tooltip("Drive the arm from the sliders below while idle")]
        public bool manualControl = false;
        [Range(-2.9f, 2.9f)] public float[] manualJoints = { 0f, -0.628f, 0f, -2.513f, 0f, 1.885f, 0f };
        [Range(0f, 1f)] public float manualGripper = 0f;

        [Header("State (read-only)")]
        [NonSerialized] public volatile bool clientConnected;
        public float simTime;
        public int stepsThisEpisode;
        public string lastCommand;

        public RobotModel Robot { get; private set; }
        public TaskScene Scene { get; private set; }
        public CameraRig Cameras { get; private set; }

        float controlDt = 1f / 15f, physicsDt = 0.002f;
        Transform world;

        class Pending
        {
            public Message Request, Reply;
            public readonly ManualResetEventSlim Done = new ManualResetEventSlim(false);
        }

        readonly ConcurrentQueue<Pending> queue = new ConcurrentQueue<Pending>();
        readonly AutoResetEvent queued = new AutoResetEvent(false);
        Thread netThread;
        TcpListener listener;
        volatile bool running;

        // ------------------------------------------------------------------ lifecycle
        void Awake()
        {
            Application.runInBackground = true;
            QualitySettings.vSyncCount = 0;
            Application.targetFrameRate = 200;
            Physics.simulationMode = SimulationMode.Script;
            ParseCommandLine();
            world = new GameObject("World").transform;
            world.SetParent(transform, false);
            BuildEpisode(0, null, null);
        }

        void OnEnable()
        {
            running = true;
            netThread = new Thread(NetLoop) { IsBackground = true, Name = "UnityVLA.SimServer" };
            netThread.Start();
        }

        void OnDisable()
        {
            running = false;
            try { listener?.Stop(); } catch { /* ignore */ }
            queued.Set();
        }

        void ParseCommandLine()
        {
            var args = Environment.GetCommandLineArgs();
            for (int i = 0; i < args.Length - 1; i++)
                if (args[i] == "-unityvla-port" && int.TryParse(args[i + 1], out var p)) port = p;
        }

        void Update()
        {
            // Serve requests for up to ~12 ms per frame (a request usually follows the
            // previous reply within a millisecond, so this batches many steps per frame).
            var sw = Stopwatch.StartNew();
            bool served = false;
            while (sw.ElapsedMilliseconds < 12)
            {
                if (queue.TryDequeue(out var p))
                {
                    p.Reply = SafeHandle(p.Request);
                    p.Done.Set();
                    served = true;
                    continue;
                }
                if (!clientConnected) break;
                int left = 12 - (int)sw.ElapsedMilliseconds;
                if (left <= 0 || !queued.WaitOne(left)) break;
            }

            if (!served && !clientConnected && freeRunWhenIdle && Robot != null)
            {
                if (manualControl)
                {
                    Robot.SetArmTargets(manualJoints);
                    Robot.gripper.SetCommand(manualGripper);
                }
                float dt = Mathf.Min(Time.deltaTime, 0.05f);
                int n = Mathf.Max(1, Mathf.RoundToInt(dt / physicsDt));
                for (int i = 0; i < n; i++) Physics.Simulate(physicsDt);
                Robot.gripper.SetVisual(Robot.gripper.Measured());
            }
        }

        // ------------------------------------------------------------------ network thread
        void NetLoop()
        {
            try
            {
                listener = new TcpListener(listenOnAllInterfaces ? IPAddress.Any : IPAddress.Loopback, port);
                listener.Start();
                Debug.Log($"[UnityVLA] SimServer listening on port {port}");
            }
            catch (Exception e)
            {
                Debug.LogError($"[UnityVLA] cannot listen on port {port}: {e.Message}");
                return;
            }
            while (running)
            {
                TcpClient client;
                try { client = listener.AcceptTcpClient(); }
                catch { break; }
                client.NoDelay = true;
                clientConnected = true;
                Debug.Log("[UnityVLA] bridge connected");
                try
                {
                    using var stream = client.GetStream();
                    while (running)
                    {
                        var req = Protocol.Read(stream);
                        var p = new Pending { Request = req };
                        queue.Enqueue(p);
                        queued.Set();
                        p.Done.Wait();
                        Protocol.Write(stream, p.Reply);
                        if (req.Header.Str("cmd") == "close") break;
                    }
                }
                catch (Exception e)
                {
                    if (running) Debug.Log($"[UnityVLA] bridge disconnected ({e.GetType().Name})");
                }
                finally
                {
                    client.Close();
                    clientConnected = false;
                }
            }
        }

        // ------------------------------------------------------------------ commands
        Message SafeHandle(Message req)
        {
            string cmd = req.Header.Str("cmd", "");
            lastCommand = cmd;
            try
            {
                switch (cmd)
                {
                    case "hello": return Hello();
                    case "reset": return Reset(req.Header);
                    case "step": return Step(req.Header);
                    case "observe": return Observe();
                    case "set_joints": return SetJoints(req.Header);
                    case "link_poses": return Message.Ok().Set("links", Robot.LinkPoses());
                    case "close": return Message.Ok();
                    default: return Message.Error($"unknown command '{cmd}'");
                }
            }
            catch (Exception e)
            {
                Debug.LogException(e);
                return Message.Error($"{e.GetType().Name}: {e.Message}");
            }
        }

        Message Hello()
        {
            var lo = new List<object>();
            var hi = new List<object>();
            for (int i = 0; i < 7; i++) { lo.Add((double)Robot.lower[i]); hi.Add((double)Robot.upper[i]); }
            return Message.Ok()
                .Set("protocol", Protocol.Version)
                .Set("unity_version", Application.unityVersion)
                .Set("joint_names", new List<object>(Robot.jointNames))
                .Set("joint_lower", lo)
                .Set("joint_upper", hi)
                .Set("phys_sign", Json.ToList(Robot.physSign))
                .Set("gripper_stroke_m", (double)Robot.gripper.strokeM)
                .Set("cameras", new Dictionary<string, object>
                {
                    ["exterior"] = new List<object> { Cameras.Exterior.Width, Cameras.Exterior.Height },
                    ["wrist"] = new List<object> { Cameras.Wrist.Width, Cameras.Wrist.Height },
                });
        }

        Message Reset(Dictionary<string, object> h)
        {
            controlDt = h.F("control_dt", 1f / 15f);
            physicsDt = h.F("physics_dt", 0.002f);
            var home = h.Floats("home_qpos");
            BuildEpisode(h.I("seed"), h.Obj("scene"), home);
            return Message.Ok();
        }

        void BuildEpisode(int seed, Dictionary<string, object> scene, float[] home)
        {
            Cameras?.Destroy();
            Scene?.Destroy();
            if (Robot != null)
            {
                Robot.gameObject.SetActive(false);   // Destroy is deferred; leave physics now
                MeshLibrary.DestroyAny(Robot.gameObject);
            }

            ApplyRobotOverrides(scene?.Obj("robot"));
            Robot = RobotFactory.Build(world, robotSettings, calibrate: true);   // before objects exist
            var q = home ?? Robot.homeQpos;
            Robot.TeleportArm(q);
            Robot.gripper.Teleport(0f);
            Scene = TaskScene.Build(world, scene, seed);
            Cameras = CameraRig.Build(world, Robot.hand, scene?.Obj("cameras") ?? new Dictionary<string, object>());

            string Positions()
            {
                var sb = new System.Text.StringBuilder();
                foreach (var kv in Scene.Objects) sb.Append($" {kv.Key}={kv.Value.Body.position.ToString("F3")}");
                return sb.ToString();
            }
            Debug.Log($"[UnityVLA] reset seed={seed} spawned:{Positions()}");
            foreach (var kv in Scene.Objects)
            {
                var b = new Bounds(kv.Value.Body.position, Vector3.zero);
                foreach (var c in kv.Value.Body.GetComponentsInChildren<Collider>()) b.Encapsulate(c.bounds);
                var hits = Physics.OverlapBox(b.center, b.extents * 0.98f);
                var names = new List<string>();
                foreach (var hcol in hits)
                    if (hcol.attachedRigidbody != kv.Value.Body) names.Add($"{hcol.transform.parent?.name}/{hcol.name}");
                Debug.Log($"[UnityVLA] overlap {kv.Key} bounds={b.center.ToString("F3")}+-{b.extents.ToString("F3")}: {string.Join(", ", names)}");
            }
            // let objects settle with the arm held at home
            for (int i = 0; i < Mathf.RoundToInt(0.5f / physicsDt); i++)
            {
                Physics.Simulate(physicsDt);
                if (i < 3 || i % 50 == 0) Debug.Log($"[UnityVLA] settle {i}:{Positions()} pinch={Robot.gripper.pinch.position.ToString("F3")}");
            }
            Debug.Log($"[UnityVLA] reset seed={seed} settled:{Positions()}");
            Robot.gripper.SetVisual(Robot.gripper.Measured());
            simTime = 0f;
            stepsThisEpisode = 0;
        }

        void ApplyRobotOverrides(Dictionary<string, object> r)
        {
            if (r == null) return;
            if (r.Has("stiffness")) robotSettings.stiffness = r.Floats("stiffness");
            if (r.Has("damping")) robotSettings.damping = r.Floats("damping");
            if (r.Has("force_limit")) robotSettings.forceLimit = r.Floats("force_limit");
            if (r.Has("grip_force")) robotSettings.gripForce = r.F("grip_force");
            if (r.Has("pad_friction")) robotSettings.padFriction = r.F("pad_friction");
            if (r.Has("arm_gravity")) robotSettings.armGravity = r.Bool("arm_gravity");
        }

        Message Step(Dictionary<string, object> h)
        {
            var a = h.Floats("action");
            if (a == null || a.Length != 8) return Message.Error("action must have 8 values");
            var q = new float[7];
            Array.Copy(a, q, 7);
            Robot.SetArmTargets(q);
            Robot.gripper.SetCommand(a[7]);
            int n = Mathf.Max(1, Mathf.RoundToInt(controlDt / physicsDt));
            for (int i = 0; i < n; i++) Physics.Simulate(physicsDt);
            float g = Robot.gripper.Measured();
            Robot.gripper.SetVisual(g);
            simTime += n * physicsDt;
            stepsThisEpisode++;
            Scene.UpdateSuccess(n * physicsDt, Robot.gripper.pinch, g);
            return Message.Ok().Set("sim_time", (double)simTime);
        }

        Message Observe()
        {
            var q = Robot.GetArmPositions();
            return Message.Ok()
                .Set("joint_position", Json.ToList(q))
                .Set("gripper_position", (double)Robot.gripper.Measured())
                .Set("success", Scene.Success)
                .Set("sim_time", (double)simTime)
                .Set("task", Scene.Describe())
                .AddImage("exterior", Cameras.Exterior.Capture(), Cameras.Exterior.Width, Cameras.Exterior.Height)
                .AddImage("wrist", Cameras.Wrist.Capture(), Cameras.Wrist.Width, Cameras.Wrist.Height);
        }

        Message SetJoints(Dictionary<string, object> h)
        {
            var q = h.Floats("qpos");
            if (q == null || q.Length != 7) return Message.Error("qpos must have 7 values");
            Robot.TeleportArm(q);
            Robot.gripper.Teleport(h.F("gripper"));
            Physics.Simulate(1e-4f);   // pushes reduced coordinates into the transforms
            return Message.Ok();
        }
    }
}
