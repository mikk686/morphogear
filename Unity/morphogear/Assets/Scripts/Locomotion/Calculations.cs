using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Calculations
{
    public class Calculation : MonoBehaviour
    {
        private Control control;
        private int _L_UA;
        private int _L_FA;
        private int _L_Sh;
        private int _step_shift_forward;
        private int _step_shift_backward;
        private float _robotStandingHeight;
        private float[][] trajectory;
        private float[][] angles;

        private void Start()
        {
            control = GameObject.Find("MorphoGear").GetComponent<Control>();
            _L_UA = control.L_UA;
            _L_FA = control.L_FA;
            _L_Sh= control.L_Sh;
            _robotStandingHeight = Mathf.Sin(control.initialStandingAngle * Mathf.PI / 180) * _L_UA + _L_FA;
            _step_shift_forward = (int)(Mathf.Sqrt((_L_UA + _L_FA) * (_L_UA + _L_FA) - _robotStandingHeight * _robotStandingHeight) * 0.9f);         //Leg is straight
            _step_shift_backward = (int)(Mathf.Sqrt(_L_FA * _L_FA - (_robotStandingHeight - _L_UA) * (_robotStandingHeight - _L_UA)));     //First link -90 degrees (down)

            //trajectory = trajectoryGenerator(100);
            //angles = inverseKinematics(trajectory[0], trajectory[1]);
            Debug.Log("Robot Height (mm):" + _robotStandingHeight);

        }

        public float[][] trajectoryGenerator(int segment_num, float multiplicator = 0.8f, float _archimed_compression = 0.7f)
        {
            _robotStandingHeight = Mathf.Sin(control.initialStandingAngle * Mathf.PI / 180) * _L_UA + _L_FA;
            _step_shift_forward = (int)(Mathf.Sqrt((_L_UA + _L_FA) * (_L_UA + _L_FA) - _robotStandingHeight * _robotStandingHeight) * 0.9f);         //Leg is straight
            //float _archimed_shift = 0.7f;
            //float _archimed_compression = 0.7f;
            List<float> x = new List<float>();
            List<float> y = new List<float>();
            for (int i = 0; i < segment_num; i++) //Straight Line shifted to close archimed spiral
            {
                x.Add((float)(_step_shift_forward - multiplicator * _step_shift_forward * i / segment_num));// _archimed_shift ;
                y.Add((float)-_robotStandingHeight);
            }
            for (int j = segment_num; j > 0; j--)  //Archimed Spiral Compressed by Y
            {
                x.Add((float)-j / segment_num * multiplicator * _step_shift_forward * Mathf.Cos(Mathf.PI - Mathf.PI * j / segment_num) + _step_shift_forward);//_archimed_shift *);
                y.Add((float)j / segment_num * _archimed_compression * _step_shift_forward * Mathf.Sin(Mathf.PI - Mathf.PI * j / segment_num) - _robotStandingHeight);
            }
            float[] x_float = x.ToArray();
            float[] y_float = y.ToArray();
            return new[] { x_float, y_float };
        }

        public float[][] trajectoryGenerator1(int segment_num, float multiplicator = 0.8f)
        {
            //float _archimed_shift = 0.7f;
            float _archimed_compression = 0.5f;
            List<float> x = new List<float>();
            List<float> y = new List<float>();
            for (int i = 0; i < segment_num; i++) //Straight Line shifted to close archimed spiral
            {
                x.Add((float)(_step_shift_forward - multiplicator * _step_shift_forward * i / segment_num));// _archimed_shift ;
                y.Add((float)-_robotStandingHeight);
            }
            for (int j = 0; j < segment_num; j++)  //Archimed Spiral Compressed by Y
            {
                x.Add((float)j / segment_num * multiplicator * _step_shift_forward * Mathf.Cos(Mathf.PI - Mathf.PI * j / segment_num));// + _step_shift_forward);//_archimed_shift *);
                y.Add((float)j / segment_num * _archimed_compression * _step_shift_forward * Mathf.Sin(Mathf.PI - Mathf.PI * j / segment_num) - _robotStandingHeight);
            }
            float[] x_float = x.ToArray();
            float[] y_float = y.ToArray();
            return new[] { x_float, y_float };
        }

        public float[][] trajectorySitDown(int segment_num)
        {
            float x_pose = _L_UA; //Mathf.Cos(control.initialStandingAngle * Mathf.PI / 180) * _L_UA;

            float y_pose = Mathf.Sqrt((_L_UA + _L_FA) * (_L_UA + _L_FA) - x_pose * x_pose);
            float temp = y_pose - _L_FA;
            Debug.Log("Impedance Dump X Domain (mm) x0.9: " + temp * 0.9f);
            List<float> x = new List<float>();
            List<float> y = new List<float>();
            for (int i = 0; i < segment_num; i++)
            {
                x.Add(x_pose);
                y.Add(-y_pose + (y_pose - _L_FA) * i / segment_num); //(-_robotStandingHeight + (_robotStandingHeight - _L_FA) * i / segment_num );
            }

            float[] x_float = x.ToArray();
            float[] y_float = y.ToArray();
            return new[] { x_float, y_float };
        }



        public float[][] inverseKinematics(float[] x, float[] y)
        {
            float[][] theta = new float[2][];
            theta[0] = new float[x.Length];
            theta[1] = new float[y.Length];


            int L1 = _L_UA;
            int L2 = _L_FA;
            for (int i = 0; i < x.Length; i++)
            {
                float c2 = (x[i] * x[i] + y[i] * y[i] - L1 * L1 - L2 * L2) / (2 * L1 * L2);
                float s2 = Mathf.Sqrt(1 - c2 * c2);
                theta[1][i] = -Mathf.Acos(c2) * 180 / Mathf.PI;
                theta[0][i] = (Mathf.Atan2(y[i], x[i]) + Mathf.Atan2(L2 * s2, L1 + L2 * c2)) * 180 / Mathf.PI;
            }

            return (theta);
        }

        public float[] inverseKinematic(float x, float y)
        {
            float[] theta = { 0, 0 };

            int L1 = _L_UA;
            int L2 = _L_FA;
            float c2 = (x * x + y * y - L1 * L1 - L2 * L2) / (2 * L1 * L2);
            float s2 = Mathf.Sqrt(1 - c2 * c2);
            theta[1] = -Mathf.Acos(c2) * 180 / Mathf.PI;
            theta[0] = (Mathf.Atan2(y, x) + Mathf.Atan2(L2 * s2, L1 + L2 * c2)) * 180 / Mathf.PI;

            return (theta);
        }


        public float[][] inverseKinematics3D(Vector3[] targets)
        {
            float[][] theta = new float[3][];
            theta[0] = new float[targets.Length];
            theta[1] = new float[targets.Length];
            theta[2] = new float[targets.Length];

            float L1 = _L_UA;
            float L2 = _L_FA;
            float L3 = _L_Sh;

            for (int i = 0; i < targets.Length; i++)
            {
                Vector3 target = targets[i];

                // 1. Compute yaw (azimuth)
                float dx = target.x - 0; // assuming base at (0, 0, 0)
                float dz = target.z - 0;
                float yaw = Mathf.Atan2(dz, dx);
                theta[0][i] = yaw * Mathf.Rad2Deg;

                // 2. Rotate into yaw-aligned plane
                float x2D = new Vector2(dx, dz).magnitude - L3; // remove shoulder offset along yaw
                
                //x2D = L3 * Mathf.Cos(theta[0][i]);
                float y2D = target.y;

                // 3. Solve 2D IK
                float dist2 = x2D * x2D + y2D * y2D;
                float c2 = (dist2 - L1 * L1 - L2 * L2) / (2 * L1 * L2);
                c2 = Mathf.Clamp(c2, -1f, 1f);
                float s2 = Mathf.Sqrt(1 - c2 * c2);

                theta[2][i] = -Mathf.Acos(c2) * Mathf.Rad2Deg;

                float k1 = L1 + L2 * c2;
                float k2 = L2 * s2;
                float shoulder = Mathf.Atan2(y2D, x2D) - Mathf.Atan2(k2, k1);
                theta[1][i] = shoulder * Mathf.Rad2Deg;
            }

            return theta;
        }


        public Vector3 forwardKinematics(float yawDeg, float shoulderDeg, float elbowDeg)
        {
            float L1 = _L_UA;
            float L2 = _L_FA;
            float L3 = _L_Sh; // Shoulder base offset

            // Convert angles to radians
            float yaw = yawDeg * Mathf.Deg2Rad;
            float shoulder = shoulderDeg * Mathf.Deg2Rad;
            float elbow = elbowDeg * Mathf.Deg2Rad;

            // Compute planar position (2D)
            float x2D = L1 * Mathf.Cos(shoulder) + L2 * Mathf.Cos(shoulder + elbow);
            float y = L1 * Mathf.Sin(shoulder) + L2 * Mathf.Sin(shoulder + elbow);

            // Rotate 2D XZ plane result by yaw
            float x = Mathf.Cos(yaw) * x2D;
            float z = Mathf.Sin(yaw) * x2D;

            return new Vector3(z + Mathf.Sin(yaw) * L3, -y, x+Mathf.Cos(yaw) * L3);

            // Add shoulder offset
            //return new Vector3(x + Mathf.Cos(yaw) * L3, y, z + Mathf.Sin(yaw) * L3);
        }


    }
}

