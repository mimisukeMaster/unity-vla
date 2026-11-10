// Franka Panda + Robotiq 2F-85 built at runtime from Resources/UnityVLA/robot_manifest.json.
// Joint convention (see docs/robot_model.md):
//   DROID joint angle q (rad, right-handed, ROS)  ->  Unity kinematics AngleAxis(jointSign * q, axis)
//   ArticulationBody reduced coordinate p = q * jointSign * physSign, where physSign (+1/-1) is
//   measured once at build time, so no PhysX sign convention is assumed.
using System;
using System.Collections.Generic;
using UnityEngine;

namespace UnityVLA
{
    [Serializable]
    public class RobotSettings
    {
        // Chosen with the Diag_GainSweep PlayMode test: settled error 0, no oscillation,
        // tracks a 0.3 rad / 0.7 Hz sine at 15 Hz commands.  (Revolute targets are in degrees.)
        [Tooltip("Drive stiffness per arm joint")]
        public float[] stiffness = { 5000, 5000, 5000, 5000, 1250, 1250, 1250 };
        [Tooltip("Drive damping per arm joint")]
        public float[] damping = { 250, 250, 250, 250, 62.5f, 62.5f, 62.5f };
        [Tooltip("Torque limit per arm joint (N·m). Above the Franka spec so the sim tracks like DROID's controller.")]
        public float[] forceLimit = { 1000, 1000, 1000, 1000, 300, 300, 300 };
        public bool armGravity = false;
        [Tooltip("Closing force limit per finger (N)")]
        public float gripForce = 40f;
        public float padStiffness = 3000f;
        public float padDamping = 100f;
        public float padFriction = 1.0f;
        public int solverIterations = 32;
        public int solverVelocityIterations = 8;
    }

    public class RobotModel : MonoBehaviour
    {
        public string[] jointNames;
        public ArticulationBody root;
        public ArticulationBody[] joints = new ArticulationBody[7];
        public Vector3[] jointAxes = new Vector3[7];
        public float[] jointSign = new float[7];
        public float[] physSign = { 1, 1, 1, 1, 1, 1, 1 };
        public float[] lower = new float[7], upper = new float[7];
        public Quaternion[] restLocalRot = new Quaternion[7];
        public Dictionary<string, Transform> links = new Dictionary<string, Transform>();
        public GripperModel gripper;
        public Transform hand;
        public RobotSettings settings;
        public List<Collider> colliders = new List<Collider>();
        public float[] homeQpos = new float[7];

        // ------------------------------------------------------------------ arm
        public float ToDrive(int i, float q) => q * jointSign[i] * physSign[i];

        public void SetArmTargets(float[] q)
        {
            for (int i = 0; i < 7; i++)
            {
                float qi = Mathf.Clamp(q[i], lower[i], upper[i]);
                var d = joints[i].xDrive;
                d.target = ToDrive(i, qi) * Mathf.Rad2Deg;
                joints[i].xDrive = d;
            }
        }

        public float[] GetArmPositions()
        {
            var q = new float[7];
            for (int i = 0; i < 7; i++) q[i] = joints[i].jointPosition[0] * jointSign[i] * physSign[i];
            return q;
        }

        /// Teleport (no dynamics).  Transforms update after the next Physics.Simulate.
        public void TeleportArm(float[] q)
        {
            for (int i = 0; i < 7; i++)
            {
                float qi = Mathf.Clamp(q[i], lower[i], upper[i]);
                joints[i].jointPosition = new ArticulationReducedSpace(ToDrive(i, qi));
                joints[i].jointVelocity = new ArticulationReducedSpace(0f);
            }
            SetArmTargets(q);
        }

        /// Kinematic pose of the transforms only (edit-mode tests, previews).
        public void PoseKinematic(float[] q)
        {
            for (int i = 0; i < 7; i++)
                joints[i].transform.localRotation =
                    restLocalRot[i] * Quaternion.AngleAxis(jointSign[i] * q[i] * Mathf.Rad2Deg, jointAxes[i]);
        }

        public Dictionary<string, object> LinkPoses()
        {
            var d = new Dictionary<string, object>();
            foreach (var kv in links)
            {
                var t = kv.Value;
                d[kv.Key] = new Dictionary<string, object>
                {
                    ["pos"] = Json.ToList(transform.InverseTransformPoint(t.position)),
                    ["rot"] = Json.ToList(Quaternion.Inverse(transform.rotation) * t.rotation),
                };
            }
            return d;
        }
    }

    public static class RobotFactory
    {
        public const string ManifestResource = "UnityVLA/robot_manifest";

        public static Dictionary<string, object> LoadManifest()
        {
            var ta = Resources.Load<TextAsset>(ManifestResource);
            if (ta == null) throw new Exception("robot_manifest.json not found in Resources/UnityVLA");
            return Json.ParseObject(ta.text);
        }

        public static RobotModel Build(Transform parent, RobotSettings settings = null, bool calibrate = true)
        {
            settings ??= new RobotSettings();
            var man = LoadManifest();
            var go = new GameObject("Robot");
            go.transform.SetParent(parent, false);
            var model = go.AddComponent<RobotModel>();
            model.settings = settings;
            var home = man.Floats("home_qpos");
            Array.Copy(home, model.homeQpos, 7);
            var names = man.Arr("joint_names");
            model.jointNames = new string[names.Count];
            for (int i = 0; i < names.Count; i++) model.jointNames[i] = names[i].ToString();

            var byName = new Dictionary<string, Transform>();
            int ji = 0;
            foreach (var o in man.Arr("links"))
            {
                var L = (Dictionary<string, object>)o;
                string name = L.Str("name");
                string parentName = L.Str("parent");
                var parentT = parentName == null ? go.transform : byName[parentName];
                var lt = new GameObject(name).transform;
                lt.SetParent(parentT, false);
                lt.localPosition = L.V3("pos");
                lt.localRotation = L.Q("rot");
                byName[name] = lt;
                model.links[name] = lt;
                if (name == "attachment") continue;   // pose marker only

                foreach (var vo in L.Arr("visuals")) AddVisual(lt, (Dictionary<string, object>)vo, Vector3.zero, Quaternion.identity);
                foreach (var co in L.Arr("colliders"))
                {
                    var cgo = new GameObject("collider");
                    cgo.transform.SetParent(lt, false);
                    var mc = cgo.AddComponent<MeshCollider>();
                    mc.sharedMesh = MeshLibrary.Get(((Dictionary<string, object>)co).Str("mesh"));
                    mc.convex = true;
                    model.colliders.Add(mc);
                }

                var ab = lt.gameObject.AddComponent<ArticulationBody>();
                ab.mass = L.F("mass", 0.1f);
                ab.automaticCenterOfMass = false;
                ab.centerOfMass = L.V3("com");
                ab.useGravity = settings.armGravity;
                ab.linearDamping = 0.05f;
                ab.angularDamping = 0.05f;
                ab.jointFriction = 0f;
                var J = L.Obj("joint");
                if (J == null)
                {
                    model.root = ab;
                    ab.immovable = true;
                    ab.solverIterations = settings.solverIterations;
                    ab.solverVelocityIterations = settings.solverVelocityIterations;
                    continue;
                }
                var axis = J.V3("axis").normalized;
                ab.jointType = ArticulationJointType.RevoluteJoint;
                ab.anchorPosition = Vector3.zero;
                ab.anchorRotation = Quaternion.FromToRotation(Vector3.right, axis);
                model.joints[ji] = ab;
                model.jointAxes[ji] = axis;
                model.jointSign[ji] = J.F("sign", -1f);
                model.lower[ji] = J.F("lower");
                model.upper[ji] = J.F("upper");
                model.restLocalRot[ji] = lt.localRotation;
                ji++;
            }
            if (ji != 7) throw new Exception($"expected 7 joints in manifest, found {ji}");

            model.gripper = GripperModel.Build(model, man.Obj("gripper"), settings);
            model.hand = model.gripper.transform;

            ConfigureDrives(model);
            IgnoreSelfCollisions(model);
            if (calibrate && Application.isPlaying) CalibrateSigns(model);
            if (Application.isPlaying) model.TeleportArm(model.homeQpos);
            else model.PoseKinematic(model.homeQpos);
            return model;
        }

        static void ConfigureDrives(RobotModel m)
        {
            var s = m.settings;
            for (int i = 0; i < 7; i++)
            {
                var ab = m.joints[i];
                ab.twistLock = ArticulationDofLock.LimitedMotion;
                var d = ab.xDrive;
                float a = m.ToDrive(i, m.lower[i]) * Mathf.Rad2Deg;
                float b = m.ToDrive(i, m.upper[i]) * Mathf.Rad2Deg;
                d.lowerLimit = Mathf.Min(a, b);
                d.upperLimit = Mathf.Max(a, b);
                d.stiffness = s.stiffness[i];
                d.damping = s.damping[i];
                d.forceLimit = s.forceLimit[i];
                d.driveType = ArticulationDriveType.Force;
                ab.xDrive = d;
            }
        }

        static void IgnoreSelfCollisions(RobotModel m)
        {
            var cs = m.colliders;
            for (int a = 0; a < cs.Count; a++)
                for (int b = a + 1; b < cs.Count; b++)
                    Physics.IgnoreCollision(cs[a], cs[b], true);
        }

        /// Measures the sign PhysX uses for positive reduced coordinates on each joint.
        /// Must run before any scene objects exist (it advances physics by a tiny step).
        public static void CalibrateSigns(RobotModel m)
        {
            const float probe = 0.25f;  // rad
            for (int i = 0; i < 7; i++)
            {
                var ab = m.joints[i];
                var rest = ab.transform.localRotation;
                ab.twistLock = ArticulationDofLock.FreeMotion;   // limits would clamp the probe
                ab.jointPosition = new ArticulationReducedSpace(probe);
                var d = ab.xDrive; d.target = probe * Mathf.Rad2Deg; ab.xDrive = d;
                Physics.Simulate(1e-4f);
                var delta = Quaternion.Inverse(rest) * ab.transform.localRotation;
                delta.ToAngleAxis(out float ang, out Vector3 ax);
                float s = Vector3.Dot(ax, m.jointAxes[i]) * Mathf.Sign(ang) >= 0 ? 1f : -1f;
                m.physSign[i] = s;
                ab.jointPosition = new ArticulationReducedSpace(0f);
                d.target = 0; ab.xDrive = d;
                Physics.Simulate(1e-4f);
            }
            m.gripper.CalibrateSigns();
            ConfigureDrives(m);   // limits depend on the sign
        }

        public static GameObject AddVisual(Transform parent, Dictionary<string, object> v, Vector3 pos, Quaternion rot)
        {
            var g = new GameObject(v.Str("mesh"));
            g.transform.SetParent(parent, false);
            g.transform.localPosition = v.Has("pos") ? v.V3("pos") : pos;
            g.transform.localRotation = v.Has("rot") ? v.Q("rot") : rot;
            g.AddComponent<MeshFilter>().sharedMesh = MeshLibrary.Get(v.Str("mesh"));
            g.AddComponent<MeshRenderer>().sharedMaterial =
                MeshLibrary.ColorMaterial(v.Col("color", new Color(0.8f, 0.8f, 0.8f)));
            return g;
        }
    }
}
