// Menu: UnityVLA/Create Tabletop Scene — one click to get a runnable scene.
using System.IO;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;

namespace UnityVLA.Editor
{
    public static class UnityVLAMenu
    {
        const string ScenePath = "Assets/UnityVLA/Scenes/Tabletop.unity";

        [MenuItem("UnityVLA/Create Tabletop Scene", priority = 1)]
        public static void CreateScene()
        {
            if (!EditorSceneManager.SaveCurrentModifiedScenesIfUserWantsTo()) return;
            var scene = EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);

            var server = new GameObject("UnityVLA");
            server.AddComponent<SimServer>();

            // Game-view camera for watching; the policy cameras are created at runtime.
            var cam = new GameObject("Viewer Camera").AddComponent<Camera>();
            cam.transform.position = new Vector3(0.9f, 0.75f, -0.35f);
            cam.transform.LookAt(new Vector3(0f, 0.15f, 0.45f));
            cam.fieldOfView = 50f;
            cam.tag = "MainCamera";

            Directory.CreateDirectory(Path.GetDirectoryName(ScenePath));
            EditorSceneManager.SaveScene(scene, ScenePath);
            EditorBuildSettings.scenes = new[] { new EditorBuildSettingsScene(ScenePath, true) };
            Debug.Log($"[UnityVLA] created {ScenePath}. Press Play, then run the bridge.");
        }

        [MenuItem("UnityVLA/Open Tabletop Scene", priority = 2)]
        public static void OpenScene()
        {
            if (!File.Exists(ScenePath)) { CreateScene(); return; }
            if (EditorSceneManager.SaveCurrentModifiedScenesIfUserWantsTo()) EditorSceneManager.OpenScene(ScenePath);
        }

        [MenuItem("UnityVLA/Build Standalone Player", priority = 20)]
        public static void BuildPlayer()
        {
            if (!File.Exists(ScenePath)) CreateScene();
            // Built-in RP needs the Standard shader in the build; it is referenced only from code.
            var standard = Shader.Find("Standard");
            if (standard != null) AlwaysInclude(standard);
            string exe = EditorUserBuildSettings.activeBuildTarget == BuildTarget.StandaloneWindows64
                ? "Builds/UnityVLA/UnityVLA.exe" : "Builds/UnityVLA/UnityVLA";
            var report = BuildPipeline.BuildPlayer(new[] { ScenePath }, exe,
                EditorUserBuildSettings.activeBuildTarget, BuildOptions.None);
            Debug.Log($"[UnityVLA] build: {report.summary.result} -> {exe}");
        }

        static void AlwaysInclude(Shader shader)
        {
            var gs = AssetDatabase.LoadAllAssetsAtPath("ProjectSettings/GraphicsSettings.asset")[0];
            var so = new SerializedObject(gs);
            var arr = so.FindProperty("m_AlwaysIncludedShaders");
            for (int i = 0; i < arr.arraySize; i++)
                if (arr.GetArrayElementAtIndex(i).objectReferenceValue == shader) return;
            arr.InsertArrayElementAtIndex(arr.arraySize);
            arr.GetArrayElementAtIndex(arr.arraySize - 1).objectReferenceValue = shader;
            so.ApplyModifiedProperties();
        }
    }
}
