using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using LidarProcessor;

namespace RobotPWF2
{
    public class Robot
    {
        public bool Jack { get; set; } = false;
        public byte VitesseDroite { get; set; } = 0;
        public byte VitesseGauche { get; set; } = 0;
        public double Kp { get; set; } = 12.5;
        public double Ki { get; set; } = 0;//6.0;
        public double Kd { get; set; } = 0;//5; 
        public double[] xRawArray { get; set; }
        public double[] yRawArray { get; set; }
        public double[] xProcessedArray { get; set; }
        public double[] yProcessedArray { get; set; }
        public List<CoherentObject> coherentObjectList { get; set; }
        public List<CoherentObject> ballList { get; set; }
    }
}
