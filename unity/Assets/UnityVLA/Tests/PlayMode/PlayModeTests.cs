using System.Collections;
using System.Collections.Generic;
using System.IO;
using NUnit.Framework;
using UnityEngine;
using UnityEngine.TestTools;

namespace UnityVLA.Tests
{
    /// Physics-level checks (M1 tracking, gripper, M2 determinism).  They run without the bridge.
    public class PlayModeTests
    {
        GameObject host;
        SimulationMode prevMode;

        [SetUp]
        public void SetUp()
        {
            prevMode = Physics.simulationMode;
            Physics.simulationMode = SimulationMode.Script;
            host = new GameObject("playmode_test");
        }

        [TearDown]
        public void TearDown()
        {
            Object.Destroy(host);
            Physics.simulationMode = prevMode;
        }

        static void Simulate(float seconds, float dt = 0.002f)
        {
            int n = Mathf.RoundToInt(seconds / dt);
            for (int i = 0; i < n; i++) Physics.Simulate(dt);
        }

        static List<object> ReferenceCases()
        {
            var path = Path.Combine(Application.dataPath, "UnityVLA/Tests/EditMode/fk_reference.json");
            return Json.ParseObject(File.ReadAllText(path)).Arr("cases");
        }

        /// M1: drive every joint to reference poses; link origins must land within 5 mm.
        [UnityTest]
        public IEnumerator Arm_TracksTargets_Within5mm()
        {
            var robot = RobotFactory.Build(host.transform);
            Debug.Log("[UnityVLA] physSign = " + string.Join(",", robot.physSign));
            var cases = ReferenceCases();
            double worst = 0;
            string worstName = "";
            for (int k = 0; k < 8; k++)
            {
                var c = (Dictionary<string, object>)cases[k];
                var q = c.Floats("q");
                robot.SetArmTargets(q);
                Simulate(3f);
                var got = robot.LinkPoses();
                foreach (var kv in c.Obj("links"))
                {
                    var e = (((Dictionary<string, object>)kv.Value).V3("pos") -
                             ((Dictionary<string, object>)got[kv.Key]).V3("pos")).magnitude;
                    if (e > worst) { worst = e; worstName = kv.Key; }
                }
                var qa = robot.GetArmPositions();
                for (int j = 0; j < 7; j++)
                    Assert.AreEqual(Mathf.Clamp(q[j], robot.lower[j], robot.upper[j]), qa[j], 0.01f, $"case {k} joint {j + 1}");
                yield return null;
            }
            Debug.Log($"[UnityVLA] tracking worst error {worst * 1000:F2} mm ({worstName})");
            Assert.Less(worst, 0.005, $"tracking error {worst} m at {worstName}");
        }

        /// Teleport + reported joint positions agree with the reference FK (sign convention).
        [UnityTest]
        public IEnumerator Arm_Teleport_MatchesReference()
        {
            var robot = RobotFactory.Build(host.transform);
            var cases = ReferenceCases();
            for (int k = 0; k < 10; k++)
            {
                var c = (Dictionary<string, object>)cases[k];
                robot.TeleportArm(c.Floats("q"));
                Physics.Simulate(1e-4f);
                var got = robot.LinkPoses();
                foreach (var kv in c.Obj("links"))
                {
                    var e = (((Dictionary<string, object>)kv.Value).V3("pos") -
                             ((Dictionary<string, object>)got[kv.Key]).V3("pos")).magnitude;
                    Assert.Less(e, 0.002, $"case {k} link {kv.Key}");
                }
            }
            yield return null;
        }

        [UnityTest]
        public IEnumerator Gripper_OpensAndCloses()
        {
            var robot = RobotFactory.Build(host.transform);
            robot.gripper.SetCommand(1f);
            Simulate(1f);
            float closed = robot.gripper.Measured();
            robot.gripper.SetCommand(0f);
            Simulate(1f);
            float open = robot.gripper.Measured();
            Debug.Log($"[UnityVLA] gripper closed={closed:F3} open={open:F3}");
            Assert.Greater(closed, 0.95f);
            Assert.Less(open, 0.05f);
            yield return null;
        }

        /// M2: same commands twice from a fresh build -> identical trajectories.
        [UnityTest]
        public IEnumerator Arm_Deterministic()
        {
            float[] Run()
            {
                foreach (Transform t in host.transform) { t.gameObject.SetActive(false); Object.Destroy(t.gameObject); }
                var robot = RobotFactory.Build(host.transform);
                var home = robot.homeQpos;
                var q = new float[7];
                float worst = 0;
                for (int s = 0; s < 60; s++)
                {
                    for (int j = 0; j < 7; j++) q[j] = home[j] + 0.3f * Mathf.Sin(0.2f * s + j);
                    robot.SetArmTargets(q);
                    robot.gripper.SetCommand(s % 30 < 15 ? 1f : 0f);
                    Simulate(1f / 15f);
                    var qa = robot.GetArmPositions();
                    for (int j = 0; j < 7; j++) worst = Mathf.Max(worst, Mathf.Abs(qa[j] - q[j]));
                }
                Debug.Log($"[UnityVLA] determinism run: worst |q - target| = {worst:F4} rad");
                Assert.Less(worst, 0.5f, "arm diverged from its targets (unstable physics)");
                return robot.GetArmPositions();
            }
            var a = Run();
            yield return null;
            var b = Run();
            for (int j = 0; j < 7; j++) Assert.AreEqual(a[j], b[j], 1e-4f, $"joint {j + 1}");
        }

        /// Diagnostics: tracking lag of a 0.3 rad sine for several drive gain sets.
        [UnityTest]
        public IEnumerator Diag_GainSweep()
        {
            var sets = new (float k, float d, float kw, float dw)[]
            {
                (35, 1.75f, 9, 0.45f), (200, 10, 50, 2.5f), (1000, 50, 250, 12.5f),
                (5000, 250, 1250, 62.5f), (20000, 1000, 5000, 250),
            };
            foreach (var g in sets)
            {
                foreach (Transform t in host.transform) { t.gameObject.SetActive(false); Object.Destroy(t.gameObject); }
                var st = new RobotSettings
                {
                    stiffness = new[] { g.k, g.k, g.k, g.k, g.kw, g.kw, g.kw },
                    damping = new[] { g.d, g.d, g.d, g.d, g.dw, g.dw, g.dw },
                    forceLimit = new float[] { 1000, 1000, 1000, 1000, 300, 300, 300 },
                };
                var robot = RobotFactory.Build(host.transform, st);
                var home = robot.homeQpos;
                var q = (float[])home.Clone();
                float worst = 0, settle = 0;
                for (int s = 0; s < 90; s++)
                {
                    if (s < 60) for (int j = 0; j < 7; j++) q[j] = home[j] + 0.3f * Mathf.Sin(0.2f * s + j);
                    robot.SetArmTargets(q);
                    robot.gripper.SetCommand(s % 30 < 15 ? 1f : 0f);
                    Simulate(1f / 15f);
                    var qa = robot.GetArmPositions();
                    for (int j = 0; j < 7; j++)
                    {
                        float e = Mathf.Abs(qa[j] - q[j]);
                        if (s < 60) worst = Mathf.Max(worst, e); else if (s == 89) settle = Mathf.Max(settle, e);
                    }
                }
                Debug.Log($"[UnityVLA] gains k={g.k} d={g.d} kw={g.kw} dw={g.dw}: worst lag {worst:F4} rad, settled err {settle:F5} rad");
                yield return null;
            }
        }
    }
}
