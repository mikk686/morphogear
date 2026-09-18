// DroneSettings.cs
// Parameter registry for the runtime settings menu (ArticulationBody drone project).
// Namespace DroneLab keeps it independent from any other settings system in the project.
//
// Add a parameter from ANY script in one line:
//     DroneSettings.Float("Camera", "FOV", () => cam.fieldOfView, v => cam.fieldOfView = v, 30f, 120f);
//     DroneSettings.Vec3 ("PID", "Pos X", () => c.PID_pos_x, v => c.PID_pos_x = v, "P,I,D");
//     DroneSettings.Bool ("Rig", "Immovable", () => root.immovable, v => root.immovable = v);
//     DroneSettings.Readout("Rig", "Total mass", () => TotalMass().ToString("F2") + " kg");
//     DroneSettings.Button("Rig", "Reset pose", () => rig.ResetPose());
//
// ...or decorate a field and let the menu find it:
//     [DroneSetting("Wind", "Speed", Min = 0, Max = 20)] public float windSpeed = 3f;
using System;
using System.Collections.Generic;
using System.Globalization;
using System.Reflection;
using UnityEngine;

namespace DroneLab
{
    /// <summary>Marks a field for the settings menu (works on public and [SerializeField] private fields).</summary>
    [AttributeUsage(AttributeTargets.Field, Inherited = true, AllowMultiple = false)]
    public sealed class DroneSettingAttribute : Attribute
    {
        public string Category;
        public string Label;
        public float Min;
        public float Max;
        /// <summary>Sub-labels for Vector3, e.g. "P,I,D".</summary>
        public string Labels;
        public int Order;

        public DroneSettingAttribute(string category = "General", string label = null)
        {
            Category = category;
            Label = label;
        }
    }

    public enum SettingKind { Float, Int, Bool, Vector3, Enum, Text, Button, Header, Readout }

    public class SettingEntry
    {
        public SettingKind kind;
        public string category = "General";
        public string label = "";
        public Type valueType;
        public Func<object> get;
        public Action<object> set;
        public Action click;
        public Func<string> readout;
        public float min, max;
        public string[] subLabels = { "X", "Y", "Z" };
        public int order;
        public object owner;
        public object defaultValue;
        public int insertIndex;

        public bool HasSlider => (kind == SettingKind.Float || kind == SettingKind.Int) && max > min;
        public string Key => category + "/" + label;
    }

    public static class DroneSettings
    {
        private static readonly List<SettingEntry> _entries = new List<SettingEntry>();
        private static int _counter;

        public static IReadOnlyList<SettingEntry> Entries => _entries;

        public static event Action<SettingEntry> Changed;
        public static event Action ListChanged;

        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.SubsystemRegistration)]
        private static void ResetStatics()
        {
            _entries.Clear();
            _counter = 0;
            Changed = null;
            ListChanged = null;
        }

        // ------------------------------------------------------------------ registration
        public static SettingEntry Add(SettingEntry e)
        {
            if (e == null) return null;
            e.insertIndex = _counter++;
            if (e.set != null && e.get != null)
            {
                try { e.defaultValue = e.get(); } catch { e.defaultValue = null; }
            }
            _entries.Add(e);
            ListChanged?.Invoke();
            return e;
        }

        public static SettingEntry Float(string category, string label, Func<float> get, Action<float> set,
                                         float min = 0f, float max = 0f, object owner = null, int order = 0)
            => Add(new SettingEntry
            {
                kind = SettingKind.Float, category = category, label = label, valueType = typeof(float),
                get = () => get(), set = v => set(Convert.ToSingle(v, CultureInfo.InvariantCulture)),
                min = min, max = max, owner = owner, order = order
            });

        public static SettingEntry Int(string category, string label, Func<int> get, Action<int> set,
                                       float min = 0f, float max = 0f, object owner = null, int order = 0)
            => Add(new SettingEntry
            {
                kind = SettingKind.Int, category = category, label = label, valueType = typeof(int),
                get = () => get(), set = v => set(Convert.ToInt32(v, CultureInfo.InvariantCulture)),
                min = min, max = max, owner = owner, order = order
            });

        public static SettingEntry Bool(string category, string label, Func<bool> get, Action<bool> set,
                                        object owner = null, int order = 0)
            => Add(new SettingEntry
            {
                kind = SettingKind.Bool, category = category, label = label, valueType = typeof(bool),
                get = () => get(), set = v => set((bool)v), owner = owner, order = order
            });

        public static SettingEntry Vec3(string category, string label, Func<Vector3> get, Action<Vector3> set,
                                        string subLabels = "X,Y,Z", object owner = null, int order = 0)
            => Add(new SettingEntry
            {
                kind = SettingKind.Vector3, category = category, label = label, valueType = typeof(Vector3),
                get = () => get(), set = v => set((Vector3)v),
                subLabels = SplitLabels(subLabels), owner = owner, order = order
            });

        public static SettingEntry Enum<T>(string category, string label, Func<T> get, Action<T> set,
                                           object owner = null, int order = 0) where T : struct, System.Enum
            => Add(new SettingEntry
            {
                kind = SettingKind.Enum, category = category, label = label, valueType = typeof(T),
                get = () => get(), set = v => set((T)v), owner = owner, order = order
            });

        public static SettingEntry Text(string category, string label, Func<string> get, Action<string> set,
                                        object owner = null, int order = 0)
            => Add(new SettingEntry
            {
                kind = SettingKind.Text, category = category, label = label, valueType = typeof(string),
                get = () => get(), set = v => set(v as string ?? ""), owner = owner, order = order
            });

        public static SettingEntry Button(string category, string label, Action onClick,
                                          object owner = null, int order = 0)
            => Add(new SettingEntry
            {
                kind = SettingKind.Button, category = category, label = label,
                click = onClick, owner = owner, order = order
            });

        /// <summary>Live read-only value (total mass, current speed, body count...).</summary>
        public static SettingEntry Readout(string category, string label, Func<string> value,
                                           object owner = null, int order = 0)
            => Add(new SettingEntry
            {
                kind = SettingKind.Readout, category = category, label = label,
                readout = value, owner = owner, order = order
            });

        /// <summary>Section title inside a category.</summary>
        public static SettingEntry Header(string category, string text, object owner = null, int order = 0)
            => Add(new SettingEntry
            {
                kind = SettingKind.Header, category = category, label = text, owner = owner, order = order
            });

        // ------------------------------------------------------------------ reflection binding
        private const BindingFlags Flags = BindingFlags.Instance | BindingFlags.Public | BindingFlags.NonPublic;

        /// <summary>Registers every field of the target marked with [DroneSetting].</summary>
        public static int Bind(object target)
        {
            if (target == null) return 0;
            int n = 0;
            for (Type t = target.GetType(); t != null && t != typeof(MonoBehaviour) && t != typeof(object); t = t.BaseType)
            {
                foreach (var f in t.GetFields(Flags | BindingFlags.DeclaredOnly))
                {
                    var a = (DroneSettingAttribute)Attribute.GetCustomAttribute(f, typeof(DroneSettingAttribute));
                    if (a == null) continue;
                    if (BindFieldInfo(target, f, a.Category, a.Label ?? Prettify(f.Name), a.Labels, a.Min, a.Max, a.Order) != null)
                        n++;
                }
            }
            return n;
        }

        /// <summary>Registers one field by name - reaches private [SerializeField] fields too.</summary>
        public static SettingEntry BindField(object target, string fieldName, string category, string label = null,
                                             string subLabels = null, float min = 0f, float max = 0f, int order = 0)
        {
            if (target == null) return null;
            FieldInfo f = null;
            for (Type t = target.GetType(); t != null && f == null; t = t.BaseType)
                f = t.GetField(fieldName, Flags);

            if (f == null)
            {
                Debug.LogWarning($"[DroneSettings] Field '{fieldName}' not found on {target.GetType().Name} - skipped.");
                return null;
            }
            return BindFieldInfo(target, f, category, label ?? Prettify(f.Name), subLabels, min, max, order);
        }

        /// <summary>
        /// Registers every public Vector3 field that looks like a PID gain triple
        /// (name contains "pid" or "gain"). Works whatever the controller calls them.
        /// </summary>
        public static int BindPidGains(object target, string category, string subLabels = "P,I,D")
        {
            if (target == null) return 0;
            int n = 0;
            foreach (var f in target.GetType().GetFields(BindingFlags.Instance | BindingFlags.Public))
            {
                if (f.FieldType != typeof(Vector3)) continue;
                string low = f.Name.ToLowerInvariant();
                if (!low.Contains("pid") && !low.Contains("gain")) continue;
                if (BindFieldInfo(target, f, category, Prettify(f.Name), subLabels, 0f, 0f, 0) != null) n++;
            }
            return n;
        }

        private static SettingEntry BindFieldInfo(object target, FieldInfo f, string category, string label,
                                                  string subLabels, float min, float max, int order)
        {
            Type ft = f.FieldType;
            var e = new SettingEntry
            {
                category = string.IsNullOrEmpty(category) ? "General" : category,
                label = label,
                valueType = ft,
                owner = target,
                min = min,
                max = max,
                order = order,
                get = () => f.GetValue(target),
                set = v => f.SetValue(target, ConvertTo(v, ft))
            };

            if (ft == typeof(float) || ft == typeof(double)) e.kind = SettingKind.Float;
            else if (ft == typeof(int)) e.kind = SettingKind.Int;
            else if (ft == typeof(bool)) e.kind = SettingKind.Bool;
            else if (ft == typeof(Vector3)) { e.kind = SettingKind.Vector3; e.subLabels = SplitLabels(subLabels ?? "X,Y,Z"); }
            else if (ft.IsEnum) e.kind = SettingKind.Enum;
            else if (ft == typeof(string)) e.kind = SettingKind.Text;
            else return null;

            return Add(e);
        }

        /// <summary>Scans loaded scenes for [DroneSetting] fields.</summary>
        public static int BindScene()
        {
            int n = 0;
            var all = UnityEngine.Object.FindObjectsByType<MonoBehaviour>(FindObjectsInactive.Include, FindObjectsSortMode.None);
            foreach (var mb in all)
            {
                if (mb == null) continue;
                if (!HasSettingFields(mb.GetType())) continue;
                n += Bind(mb);
            }
            return n;
        }

        private static readonly Dictionary<Type, bool> _hasSettings = new Dictionary<Type, bool>();
        private static bool HasSettingFields(Type t)
        {
            if (_hasSettings.TryGetValue(t, out bool has)) return has;
            has = false;
            for (Type c = t; c != null && c != typeof(MonoBehaviour) && c != typeof(object); c = c.BaseType)
            {
                foreach (var f in c.GetFields(Flags | BindingFlags.DeclaredOnly))
                    if (Attribute.GetCustomAttribute(f, typeof(DroneSettingAttribute)) != null) { has = true; break; }
                if (has) break;
            }
            _hasSettings[t] = has;
            return has;
        }

        public static void Unbind(object owner)
        {
            if (_entries.RemoveAll(e => ReferenceEquals(e.owner, owner)) > 0) ListChanged?.Invoke();
        }

        public static void Clear()
        {
            _entries.Clear();
            ListChanged?.Invoke();
        }

        // ------------------------------------------------------------------ access
        public static void SetValue(SettingEntry e, object value)
        {
            if (e?.set == null) return;
            try { e.set(value); Changed?.Invoke(e); }
            catch (Exception ex) { Debug.LogWarning($"[DroneSettings] '{e.Key}': {ex.Message}"); }
        }

        public static object GetValue(SettingEntry e)
        {
            if (e?.get == null) return null;
            try { return e.get(); } catch { return null; }
        }

        public static List<string> Categories()
        {
            var list = new List<string>();
            foreach (var e in _entries)
                if (!list.Contains(e.category)) list.Add(e.category);
            return list;
        }

        public static List<SettingEntry> InCategory(string category)
        {
            var list = _entries.FindAll(e => e.category == category);
            list.Sort((a, b) => a.order != b.order ? a.order.CompareTo(b.order)
                                                   : a.insertIndex.CompareTo(b.insertIndex));
            return list;
        }

        public static void ResetDefaults(string category = null)
        {
            foreach (var e in _entries)
            {
                if (e.set == null || e.defaultValue == null) continue;
                if (category != null && e.category != category) continue;
                SetValue(e, e.defaultValue);
            }
        }

        // ------------------------------------------------------------------ presets
        [Serializable] private class Kv { public string k; public string v; }
        [Serializable] private class Blob { public List<Kv> items = new List<Kv>(); }

        public static string ToJson()
        {
            var blob = new Blob();
            foreach (var e in _entries)
            {
                if (e.set == null || e.get == null) continue;
                blob.items.Add(new Kv { k = e.Key, v = ToStr(GetValue(e)) });
            }
            return JsonUtility.ToJson(blob, true);
        }

        public static void FromJson(string json)
        {
            if (string.IsNullOrEmpty(json)) return;
            var blob = JsonUtility.FromJson<Blob>(json);
            if (blob?.items == null) return;
            foreach (var kv in blob.items)
            {
                var e = _entries.Find(x => x.Key == kv.k);
                if (e?.set == null) continue;
                object parsed = FromStr(kv.v, e.valueType);
                if (parsed != null) SetValue(e, parsed);
            }
        }

        public static string PresetPath(string name)
        {
            string dir = System.IO.Path.Combine(Application.persistentDataPath, "DronePresets");
            System.IO.Directory.CreateDirectory(dir);
            return System.IO.Path.Combine(dir, name + ".json");
        }

        public static void SavePreset(string name = "default")
        {
            try
            {
                string path = PresetPath(name);
                System.IO.File.WriteAllText(path, ToJson());
                Debug.Log($"[DroneSettings] Preset saved: {path}");
            }
            catch (Exception e) { Debug.LogError($"[DroneSettings] Save failed: {e.Message}"); }
        }

        public static bool LoadPreset(string name = "default")
        {
            try
            {
                string path = PresetPath(name);
                if (!System.IO.File.Exists(path)) { Debug.LogWarning($"[DroneSettings] No preset '{name}'."); return false; }
                FromJson(System.IO.File.ReadAllText(path));
                Debug.Log($"[DroneSettings] Preset loaded: {path}");
                return true;
            }
            catch (Exception e) { Debug.LogError($"[DroneSettings] Load failed: {e.Message}"); return false; }
        }

        // ------------------------------------------------------------------ helpers
        private static readonly CultureInfo C = CultureInfo.InvariantCulture;

        public static string ToStr(object v)
        {
            switch (v)
            {
                case null: return "";
                case float f: return f.ToString("R", C);
                case double d: return d.ToString("R", C);
                case int i: return i.ToString(C);
                case bool b: return b ? "1" : "0";
                case Vector3 v3: return v3.x.ToString("R", C) + ";" + v3.y.ToString("R", C) + ";" + v3.z.ToString("R", C);
                default: return v.ToString();
            }
        }

        public static object FromStr(string s, Type t)
        {
            if (t == null || s == null) return null;
            try
            {
                if (t == typeof(float)) return float.Parse(s, C);
                if (t == typeof(double)) return double.Parse(s, C);
                if (t == typeof(int)) return int.Parse(s, C);
                if (t == typeof(bool)) return s == "1" || s.ToLowerInvariant() == "true";
                if (t == typeof(string)) return s;
                if (t.IsEnum) return System.Enum.Parse(t, s, true);
                if (t == typeof(Vector3))
                {
                    var p = s.Split(';');
                    if (p.Length != 3) return null;
                    return new Vector3(float.Parse(p[0], C), float.Parse(p[1], C), float.Parse(p[2], C));
                }
            }
            catch { }
            return null;
        }

        private static object ConvertTo(object v, Type t)
        {
            if (v == null) return null;
            if (t.IsInstanceOfType(v)) return v;
            if (t == typeof(float)) return Convert.ToSingle(v, C);
            if (t == typeof(double)) return Convert.ToDouble(v, C);
            if (t == typeof(int)) return Convert.ToInt32(v, C);
            if (t == typeof(bool)) return Convert.ToBoolean(v, C);
            if (t.IsEnum) return System.Enum.ToObject(t, Convert.ToInt32(v, C));
            return v;
        }

        private static string[] SplitLabels(string s)
        {
            if (string.IsNullOrEmpty(s)) return new[] { "X", "Y", "Z" };
            var parts = s.Split(',');
            var outp = new string[3];
            for (int i = 0; i < 3; i++) outp[i] = i < parts.Length ? parts[i].Trim() : "";
            return outp;
        }

        /// <summary>"PID_pos_x_gains" -> "Pos X"</summary>
        public static string Prettify(string fieldName)
        {
            if (string.IsNullOrEmpty(fieldName)) return "";
            string s = fieldName;
            s = s.Replace("PID_", "").Replace("pid_", "").Replace("_gains", "").Replace("_gain", "");
            s = s.Replace('_', ' ');
            var sb = new System.Text.StringBuilder(s.Length + 6);
            bool up = true;
            foreach (char ch in s)
            {
                if (ch == ' ') { sb.Append(' '); up = true; continue; }
                if (char.IsUpper(ch) && sb.Length > 0 && !up) sb.Append(' ');
                sb.Append(up ? char.ToUpperInvariant(ch) : ch);
                up = false;
            }
            return sb.ToString().Trim();
        }
    }
}
