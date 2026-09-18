using System;
using System.Collections.Generic;
using UnityEngine;

namespace RobotRuntime
{
    public enum RobotMode
    {
        Game,
        Mirror,
        Simulation,
        Visualisation
    }

    public enum OverrideValueType
    {
        Boolean,
        Integer,
        Float,
        String,
        Vector3,
        Color
    }

    [Serializable]
    public sealed class ComponentToggle
    {
        [Tooltip("The existing scene component that should be enabled in this mode.")]
        public Behaviour component;
        public bool enabled = true;
    }

    [Serializable]
    public sealed class MemberOverride
    {
        [Tooltip("Component containing the public field or writable property.")]
        public Component target;
        [Tooltip("Case-sensitive public field/property name.")]
        public string memberName;
        public OverrideValueType valueType = OverrideValueType.Float;

        public bool boolValue;
        public int intValue;
        public float floatValue;
        public string stringValue;
        public Vector3 vector3Value;
        public Color colorValue = Color.white;

        [Tooltip("Show this value as an editable setting in the pause HUD.")]
        public bool showInHud = true;
        [Tooltip("Optional friendly label. The member name is used when empty.")]
        public string displayName;
    }

    [Serializable]
    public sealed class ModeProfile
    {
        public RobotMode mode;
        public string title;
        [TextArea(2, 4)] public string description;
        public Color accent = new Color(0.16f, 0.72f, 1f, 1f);

        [Tooltip("Existing scene scripts enabled for this mode. Scripts referenced in any profile are disabled first.")]
        public List<ComponentToggle> sceneComponents = new List<ComponentToggle>();

        [Tooltip("Public fields/properties assigned before scripts are enabled.")]
        public List<MemberOverride> settings = new List<MemberOverride>();

        [Tooltip("Prefabs instantiated on Apply. Put publisher scripts on these so their Start method runs each time.")]
        public List<GameObject> startupPrefabs = new List<GameObject>();
    }

    /// <summary>Optional hook for components created by a mode startup prefab.</summary>
    public interface IModeInitializable
    {
        void InitializeForMode(RobotMode mode, GameManager manager);
    }
}
