// Robotiq 2F-85: visual linkage animated from tables, physics via prismatic pad joints.
// g = 0 open, 1 closed (DROID convention).
using System;
using System.Collections.Generic;
using UnityEngine;

namespace UnityVLA
{
    public class GripperModel : MonoBehaviour
    {
        [Serializable]
        public class Side
        {
            public string name;
            public Transform driver, coupler, spring, follower;
            public Pose[] driverT, couplerT, springT, followerT;
            public ArticulationBody lat, vert;
            public Vector3 latAxisHand, vertAxisLocal;
            public float latSign = 1, vertSign = 1;
            public float[] latTable, vertTable;
            public Vector3 latRestPos, vertRestPos;
        }

        public Side[] sides;
        public ArticulationBody body;
        public Transform pinch;
        public float command;
        public float strokeM;

        public static GripperModel Build(RobotModel robot, Dictionary<string, object> G, RobotSettings s)
        {
            var attach = robot.links[G.Str("attach_to")];
            var link7 = attach.parent;
            var bp = G.Obj("base_pose");
            var go = new GameObject("gripper");
            go.transform.SetParent(link7, false);
            go.transform.localPosition = attach.localPosition + attach.localRotation * bp.V3("pos");
            go.transform.localRotation = attach.localRotation * bp.Q("rot");
            var gm = go.AddComponent<GripperModel>();
            gm.strokeM = G.F("stroke_m");

            foreach (var v in G.Arr("static_visuals"))
                RobotFactory.AddVisual(go.transform, (Dictionary<string, object>)v, Vector3.zero, Quaternion.identity);
            var cgo = new GameObject("collider");
            cgo.transform.SetParent(go.transform, false);
            var mc = cgo.AddComponent<MeshCollider>();
            mc.sharedMesh = MeshLibrary.Get(G.Obj("collider").Str("mesh"));
            mc.convex = true;
            robot.colliders.Add(mc);

            gm.body = go.AddComponent<ArticulationBody>();
            gm.body.jointType = ArticulationJointType.FixedJoint;
            gm.body.mass = G.F("mass", 0.9f);
            gm.body.useGravity = s.armGravity;

            gm.pinch = new GameObject("pinch").transform;
            gm.pinch.SetParent(go.transform, false);
            gm.pinch.localPosition = G.V3("pinch");
            robot.links["pinch"] = gm.pinch;

#if UNITY_6000_0_OR_NEWER
            var padMat = new PhysicsMaterial("pad")
#else
            var padMat = new PhysicMaterial("pad")
#endif
            {
                staticFriction = s.padFriction, dynamicFriction = s.padFriction,
#if UNITY_6000_0_OR_NEWER
                frictionCombine = PhysicsMaterialCombine.Maximum,
#else
                frictionCombine = PhysicMaterialCombine.Maximum,
#endif
            };

            var sideList = new List<Side>();
            foreach (var so in G.Arr("sides"))
            {
                var S = (Dictionary<string, object>)so;
                var side = new Side { name = S.Str("side") };
                side.driver = Part(go.transform, S.Obj("driver"), out side.driverT);
                side.coupler = Part(side.driver, S.Obj("coupler"), out side.couplerT);
                side.spring = Part(go.transform, S.Obj("spring"), out side.springT);
                side.follower = Part(side.spring, S.Obj("follower"), out side.followerT);
                foreach (var pv in S.Arr("pad_visuals"))
                    RobotFactory.AddVisual(side.follower, (Dictionary<string, object>)pv, Vector3.zero, Quaternion.identity);

                // physical pad: hand -> lat (prismatic) -> vert (prismatic) -> boxes
                var rest = S.Obj("pad_rest");
                var restRot = rest.Q("rot");
                side.latAxisHand = S.V3("pad_lateral_axis").normalized;
                var latLocal = Quaternion.Inverse(restRot) * side.latAxisHand;
                side.vertAxisLocal = (Quaternion.Inverse(restRot) * S.V3("pad_vertical_axis")).normalized;
                side.latTable = S.Floats("pad_lateral");
                side.vertTable = S.Floats("pad_vertical");

                var lat = new GameObject($"{side.name}_pad_lat");
                lat.transform.SetParent(go.transform, false);
                lat.transform.localPosition = rest.V3("pos");
                lat.transform.localRotation = restRot;
                side.latRestPos = lat.transform.localPosition;
                side.lat = Prismatic(lat, latLocal, 0.1f, s);
                Limits(side.lat, side.latTable);

                var vert = new GameObject($"{side.name}_pad_vert");
                vert.transform.SetParent(lat.transform, false);
                side.vertRestPos = Vector3.zero;
                foreach (var bo in S.Arr("pad_boxes"))
                {
                    var B = (Dictionary<string, object>)bo;
                    var bg = new GameObject("pad_box");
                    bg.transform.SetParent(vert.transform, false);
                    bg.transform.localPosition = B.V3("pos");
                    var bc = bg.AddComponent<BoxCollider>();
                    bc.size = B.V3("size");
                    bc.sharedMaterial = padMat;
                    robot.colliders.Add(bc);
                }
                side.vert = Prismatic(vert, side.vertAxisLocal, 0.1f, s);
                Limits(side.vert, side.vertTable);
                sideList.Add(side);
            }
            gm.sides = sideList.ToArray();
            gm.ConfigureDrives(s);
            gm.SetVisual(0f);
            return gm;
        }

        static Transform Part(Transform parent, Dictionary<string, object> P, out Pose[] table)
        {
            var t = new GameObject(P.Str("mesh") + "_joint").transform;
            t.SetParent(parent, false);
            var rows = P.Arr("table");
            table = new Pose[rows.Count];
            for (int i = 0; i < rows.Count; i++)
            {
                var r = (Dictionary<string, object>)rows[i];
                table[i] = new Pose(r.V3("pos"), r.Q("rot"));
            }
            t.localPosition = table[0].position;
            t.localRotation = table[0].rotation;
            RobotFactory.AddVisual(t, P, Vector3.zero, Quaternion.identity).transform.localPosition = Vector3.zero;
            return t;
        }

        static ArticulationBody Prismatic(GameObject g, Vector3 localAxis, float mass, RobotSettings s)
        {
            var ab = g.AddComponent<ArticulationBody>();
            ab.jointType = ArticulationJointType.PrismaticJoint;
            ab.anchorPosition = Vector3.zero;
            ab.anchorRotation = Quaternion.FromToRotation(Vector3.right, localAxis.normalized);
            ab.mass = mass;
            ab.useGravity = false;
            ab.linearDamping = 0.05f;
            ab.angularDamping = 0.05f;
            return ab;
        }

        static void Limits(ArticulationBody ab, float[] table)
        {
            float lo = Mathf.Min(table), hi = Mathf.Max(table);
            ab.linearLockX = ArticulationDofLock.LimitedMotion;
            var d = ab.xDrive;
            d.lowerLimit = lo - 0.002f;
            d.upperLimit = hi + 0.002f;
            ab.xDrive = d;
        }

        public void ConfigureDrives(RobotSettings s)
        {
            foreach (var side in sides)
            {
                var d = side.lat.xDrive;
                d.stiffness = s.padStiffness; d.damping = s.padDamping; d.forceLimit = s.gripForce;
                d.driveType = ArticulationDriveType.Force;
                Flip(ref d, side.latSign, side.latTable);
                side.lat.xDrive = d;
                var v = side.vert.xDrive;
                v.stiffness = s.padStiffness * 2; v.damping = s.padDamping; v.forceLimit = 200f;
                v.driveType = ArticulationDriveType.Force;
                Flip(ref v, side.vertSign, side.vertTable);
                side.vert.xDrive = v;
            }
        }

        static void Flip(ref ArticulationDrive d, float sign, float[] table)
        {
            float lo = Mathf.Min(table) - 0.002f, hi = Mathf.Max(table) + 0.002f;
            d.lowerLimit = sign > 0 ? lo : -hi;
            d.upperLimit = sign > 0 ? hi : -lo;
        }

        // ------------------------------------------------------------------ control
        public void SetCommand(float g)
        {
            command = Mathf.Clamp01(g);
            foreach (var side in sides)
            {
                SetTarget(side.lat, side.latSign * Interp(side.latTable, command));
                SetTarget(side.vert, side.vertSign * Interp(side.vertTable, command));
            }
        }

        public void Teleport(float g)
        {
            SetCommand(g);
            foreach (var side in sides)
            {
                side.lat.jointPosition = new ArticulationReducedSpace(side.latSign * Interp(side.latTable, command));
                side.vert.jointPosition = new ArticulationReducedSpace(side.vertSign * Interp(side.vertTable, command));
                side.lat.jointVelocity = new ArticulationReducedSpace(0f);
                side.vert.jointVelocity = new ArticulationReducedSpace(0f);
            }
            SetVisual(command);
        }

        /// Measured opening in [0, 1] from the lateral pad joints (mean of both fingers).
        public float Measured()
        {
            float acc = 0;
            foreach (var side in sides)
                acc += InverseInterp(side.latTable, side.lat.jointPosition[0] * side.latSign);
            return Mathf.Clamp01(acc / sides.Length);
        }

        public void SetVisual(float g)
        {
            foreach (var side in sides)
            {
                Apply(side.driver, side.driverT, g);
                Apply(side.coupler, side.couplerT, g);
                Apply(side.spring, side.springT, g);
                Apply(side.follower, side.followerT, g);
            }
        }

        public void CalibrateSigns()
        {
            foreach (var side in sides)
            {
                side.latSign = ProbeSign(side.lat, side.latAxisHand, side.lat.transform);
                side.vertSign = ProbeSign(side.vert, side.vertAxisLocal, side.vert.transform);
            }
            ConfigureDrives(GetComponentInParent<RobotModel>().settings);
        }

        static float ProbeSign(ArticulationBody ab, Vector3 axisInParent, Transform t)
        {
            var rest = t.localPosition;
            ab.linearLockX = ArticulationDofLock.FreeMotion;
            ab.jointPosition = new ArticulationReducedSpace(0.01f);
            var d = ab.xDrive; d.target = 0.01f; ab.xDrive = d;
            Physics.Simulate(1e-4f);
            float s = Vector3.Dot(t.localPosition - rest, axisInParent) >= 0 ? 1f : -1f;
            ab.jointPosition = new ArticulationReducedSpace(0f);
            d.target = 0; ab.xDrive = d;
            ab.linearLockX = ArticulationDofLock.LimitedMotion;
            Physics.Simulate(1e-4f);
            return s;
        }

        static void SetTarget(ArticulationBody ab, float x)
        {
            var d = ab.xDrive;
            d.target = x;
            ab.xDrive = d;
        }

        static void Apply(Transform t, Pose[] table, float g)
        {
            float f = Mathf.Clamp01(g) * (table.Length - 1);
            int i = Mathf.Min((int)f, table.Length - 2);
            float u = f - i;
            t.localPosition = Vector3.Lerp(table[i].position, table[i + 1].position, u);
            t.localRotation = Quaternion.Slerp(table[i].rotation, table[i + 1].rotation, u);
        }

        public static float Interp(float[] table, float g)
        {
            float f = Mathf.Clamp01(g) * (table.Length - 1);
            int i = Mathf.Min((int)f, table.Length - 2);
            return Mathf.Lerp(table[i], table[i + 1], f - i);
        }

        /// Inverse of Interp for a monotonic table.
        public static float InverseInterp(float[] table, float x)
        {
            int n = table.Length;
            bool inc = table[n - 1] > table[0];
            for (int i = 0; i < n - 1; i++)
            {
                float a = table[i], b = table[i + 1];
                bool inside = inc ? x <= b : x >= b;
                if (inside || i == n - 2)
                {
                    float u = Mathf.Approximately(a, b) ? 0 : Mathf.Clamp01((x - a) / (b - a));
                    return (i + u) / (n - 1);
                }
            }
            return 0;
        }
    }
}
