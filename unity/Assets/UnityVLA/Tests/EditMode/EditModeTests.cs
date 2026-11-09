using System.Collections.Generic;
using System.IO;
using NUnit.Framework;
using UnityEngine;

namespace UnityVLA.Tests
{
    public class EditModeTests
    {
        static Dictionary<string, object> LoadReference()
        {
            var path = Path.Combine(Application.dataPath, "UnityVLA/Tests/EditMode/fk_reference.json");
            return Json.ParseObject(File.ReadAllText(path));
        }

        [Test]
        public void Json_RoundTrip()
        {
            var d = new Dictionary<string, object>
            {
                ["a"] = 1.5, ["b"] = new List<object> { 1.0, "x", true, null },
                ["c"] = new Dictionary<string, object> { ["s"] = "q\"\n" },
            };
            var back = Json.ParseObject(Json.Serialize(d));
            Assert.AreEqual(1.5, back.Num("a"), 1e-12);
            Assert.AreEqual("q\"\n", back.Obj("c").Str("s"));
            Assert.AreEqual(4, back.Arr("b").Count);
        }

        [Test]
        public void Protocol_RoundTrip()
        {
            var msg = Message.Ok().Set("x", 3.0).AddImage("img", new byte[] { 1, 2, 3, 4, 5, 6 }, 2, 1);
            var ms = new MemoryStream();
            Protocol.Write(ms, msg);
            ms.Position = 0;
            var back = Protocol.Read(ms);
            Assert.AreEqual(3.0, back.Header.Num("x"));
            Assert.AreEqual(1, back.Blobs.Count);
            CollectionAssert.AreEqual(new[] { 1, 2, 3 }, back.Blobs[0].Shape);
            Assert.AreEqual(6, back.Blobs[0].Data[5]);
        }

        [Test]
        public void Gripper_InverseInterp()
        {
            var t = new[] { 0f, -0.01f, -0.03f, -0.044f };
            foreach (var g in new[] { 0f, 0.2f, 0.5f, 0.9f, 1f })
                Assert.AreEqual(g, GripperModel.InverseInterp(t, GripperModel.Interp(t, g)), 1e-5);
        }

        [Test]
        public void Meshes_AllLoad()
        {
            var man = RobotFactory.LoadManifest();
            int n = 0;
            foreach (var o in man.Arr("links"))
                foreach (var v in ((Dictionary<string, object>)o).Arr("visuals"))
                {
                    var m = MeshLibrary.Get(((Dictionary<string, object>)v).Str("mesh"));
                    Assert.Greater(m.vertexCount, 0);
                    n++;
                }
            Assert.Greater(n, 30);
        }

        /// M1 (geometry): the Unity hierarchy built from the manifest reproduces the
        /// MuJoCo forward kinematics of the Panda, converted to Unity coordinates.
        [Test]
        public void Kinematics_MatchReference()
        {
            var host = new GameObject("fk_test");
            try
            {
                var robot = RobotFactory.Build(host.transform, null, calibrate: false);
                var cases = LoadReference().Arr("cases");
                double worst = 0;
                string worstName = "";
                foreach (var co in cases)
                {
                    var c = (Dictionary<string, object>)co;
                    robot.PoseKinematic(c.Floats("q"));
                    var got = robot.LinkPoses();
                    foreach (var kv in c.Obj("links"))
                    {
                        var exp = ((Dictionary<string, object>)kv.Value).V3("pos");
                        var act = ((Dictionary<string, object>)got[kv.Key]).V3("pos");
                        double e = (exp - act).magnitude;
                        if (e > worst) { worst = e; worstName = kv.Key; }
                    }
                }
                Debug.Log($"[UnityVLA] kinematics worst error {worst * 1000:F4} mm ({worstName}) over {cases.Count} cases");
                Assert.Less(worst, 1e-4, $"worst link error {worst} m at {worstName}");
            }
            finally
            {
                Object.DestroyImmediate(host);
            }
        }
    }
}
