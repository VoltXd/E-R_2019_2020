using GeometryElements;
using MathNet.Numerics.LinearAlgebra;
using System;

namespace Utilities
{
    /// <summary>Décrit la position en X,Y,Theta d'un objet.</summary>
    public class PositionInfo
    {
        /// <summary>Renvoie un <see cref="string"/> décrivant l'objet.</summary>
        public override string ToString()
            => $"({X:0.##}; {Y:0.##}; {Theta:0.##})";

        /// <summary>Position en X en m.</summary>
        public double X { get; set; }

        /// <summary>Position en Y en m.</summary>
        public double Y { get; set; }

        /// <summary>Angle Theta en rad.</summary>
        public double Theta { get; set; }

        /// <summary>Constructeur par défaut (valeurs à 0).</summary>
        public PositionInfo() { }

        /// <summary>Angle en radians.</summary>
        public PositionInfo(double x, double y, double theta)
        {
            X = x;
            Y = y;
            Theta = theta;
        }

        /// <summary>Renvoie une copie profonde de l'objet.</summary>
        public PositionInfo Copy() => new PositionInfo(X, Y, Theta);

        /// <summary>Remplie cette instance en copiant les valeurs de positionInfo.</summary>
        /// <param name="positionInfo">Valeurs à copier.</param>
        public void FillWith(PositionInfo positionInfo)
        {
            X = positionInfo.X;
            Y = positionInfo.Y;
            Theta = positionInfo.Theta;
        }

        /// <summary>Renvoie le Point3D décrivant les coordonnées cartésiennes du <see cref="PositionInfo"/>.</summary>
        public Point3D ToPoint3D()
            => new Point3D(X, Y);

        /// <summary>Applique une addition membre par membre.</summary>
        public static PositionInfo operator +(PositionInfo left, SpeedInfo right)
            => new PositionInfo(left.X + right.VX, left.Y + right.VY, left.Theta + right.VTheta);

        /// <summary>Modifie les coordonnées du <see cref="PositionInfo"/> actuellement dans un référentiel parent dans un référentiel enfant.</summary>
        /// <param name="ref_x">Position x de l'origine du référentiel enfant dans le référentiel parent.</param>
        /// <param name="ref_y">Position y de l'origine du référentiel enfant dans le référentiel parent.</param>
        /// <param name="ref_theta">Angle theta en radians de l'origine du référentiel enfant dans le référentiel parent.</param>
        public void ToChildRef(double ref_x, double ref_y, double ref_theta)
        {
            // Il faut prendre l'opposé des valeurs dans le cas parent -> enfant
            ref_x = -ref_x;
            ref_y = -ref_y;
            ref_theta = -ref_theta;

            // Effectue la translation du point
            X += ref_x;
            Y += ref_y;
            Theta += ref_theta;

            // Effectue la rotation du point
            Point3D pt = ToPoint3D();
            pt.Rotate2D(ref_theta);

            X = pt.X;
            Y = pt.Y;
        }

        /// <summary>Modifie les coordonnées du <see cref="PositionInfo"/> actuellement dans un référentiel enfant dans un référentiel parent.</summary>
        /// <param name="ref_x">Position x de l'origine du référentiel enfant dans le référentiel parent.</param>
        /// <param name="ref_y">Position y de l'origine du référentiel enfant dans le référentiel parent.</param>
        /// <param name="ref_theta">Angle theta en radians de l'origine du référentiel enfant dans le référentiel parent.</param>
        public void ToParentRef(double ref_x, double ref_y, double ref_theta)
        {
            // Effectue la rotation du point
            Point3D pt = ToPoint3D();
            pt.Rotate2D(ref_theta);

            X = pt.X;
            Y = pt.Y;

            // Effectue la translation du point
            X += ref_x;
            Y += ref_y;
            Theta += ref_theta;
        }
    }

    /// <summary>Décrit les vitesses en VX,VY,VTheta d'un objet.</summary>
    public class SpeedInfo
    {
        /// <summary>Renvoie un <see cref="string"/> décrivant l'objet.</summary>
        public override string ToString()
            => $"({VX:0.##}; {VY:0.##}; {VTheta:0.##})";

        /// <summary>Vitesse en X en m/s.</summary>
        public double VX { get; set; }

        /// <summary>Vitesse en Y en m/s.</summary>
        public double VY { get; set; }

        /// <summary>Vitesse de rotation VTheta en rad/s.</summary>
        public double VTheta { get; set; }

        /// <summary>Constructeur par défaut (valeurs à 0).</summary>
        public SpeedInfo() { }

        /// <summary>Vitesse de rotation en rad/s.</summary>
        public SpeedInfo(double vx, double vy, double vtheta)
        {
            VX = vx;
            VY = vy;
            VTheta = vtheta;
        }

        /// <summary>Renvoie une copie profonde de l'objet.</summary>
        public SpeedInfo Copy() => new SpeedInfo(VX, VY, VTheta);

        /// <summary>Remplie cette instance en copiant les valeurs de speedInfo.</summary>
        /// <param name="speedInfo">Valeurs à copier.</param>
        public void FillWith(SpeedInfo speedInfo)
        {
            VX = speedInfo.VX;
            VY = speedInfo.VY;
            VTheta = speedInfo.VTheta;
        }

        /// <summary>Applique une division membre par membre.</summary>
        public static SpeedInfo operator/(SpeedInfo left, double right)
            => new SpeedInfo(left.VX / right, left.VY / right, left.VTheta / right);

        /// <summary>Passe les vitesses de cet objet de son référentiel au référentiel parent.</summary>
        /// <param name="theta">Angle du référentiel enfant dans le référentiel parent.</param>
        public SpeedInfo ToParentRef(double theta)
        {
            // On prépare la matrice rotation pour passer du ref enfant au ref parent
            Matrix<double> rotationMatrix = Matrix<double>.Build.DenseOfArray(new[,]
            {
                { Math.Cos(-theta), Math.Sin(-theta) },
                { -Math.Sin(-theta), Math.Cos(-theta) }
            });

            // On récupère les vitesses X et Y dans le référentiel parent
            Vector<double> speedRefChild = Vector<double>.Build.DenseOfArray(new[] { VX, VY });
            Vector<double> speedRefParent = rotationMatrix * speedRefChild;

            return new SpeedInfo
            {
                VX = speedRefParent[0],
                VY = speedRefParent[1],
                VTheta = VTheta
            };
        }

        /// <summary>Passe les vitesses de cet objet de son référentiel au référentiel enfant.</summary>
        /// <param name="theta">Angle du référentiel enfant dans le référentiel parent.</param>
        public SpeedInfo ToChildRef(double theta)
        {
            // On prépare la matrice rotation pour passer du ref terrain au ref enfant
            Matrix<double> rotationMatrix = Matrix<double>.Build.DenseOfArray(new[,]
            {
                { Math.Cos(-theta), Math.Sin(-theta) },
                { -Math.Sin(-theta), Math.Cos(-theta) }
            });

            // On récupère les vitesses X et Y dans le référentiel enfant
            Vector<double> speedRefParent = Vector<double>.Build.DenseOfArray(new[] { VX, VY });
            Vector<double> speedRefChild = rotationMatrix.Inverse() * speedRefParent;

            return new SpeedInfo
            {
                VX = speedRefChild[0],
                VY = speedRefChild[1],
                VTheta = VTheta
            };
        }
    }

    /// <summary>Décrit les accélérations en AX,AY,ATheta d'un objet.</summary>
    public class AccelerationInfo
    {
        /// <summary>Renvoie un <see cref="string"/> décrivant l'objet.</summary>
        public override string ToString()
            => $"({AX:0.##}; {AY:0.##}; {ATheta:0.##})";

        /// <summary>Accélération en X en m/s².</summary>
        public double AX { get; set; }

        /// <summary>Accélération en Y en m/s².</summary>
        public double AY { get; set; }

        /// <summary>Accélération de vitesse rotation ATheta en rad/s².</summary>
        public double ATheta { get; set; }

        /// <summary>Constructeur par défaut (valeurs à 0).</summary>
        public AccelerationInfo() { }

        /// <summary>Accélération de vitesse de rotation en rad/s.</summary>
        public AccelerationInfo(double ax, double ay, double atheta)
        {
            AX = ax;
            AY = ay;
            ATheta = atheta;
        }

        /// <summary>Renvoie une copie profonde de l'objet.</summary>
        public AccelerationInfo Copy() => new AccelerationInfo(AX, AY, ATheta);

        /// <summary>Remplie cette instance en copiant les valeurs de accelerationInfo.</summary>
        /// <param name="accelerationInfo">Valeurs à copier.</param>
        public void FillWith(AccelerationInfo accelerationInfo)
        {
            AX = accelerationInfo.AX;
            AY = accelerationInfo.AY;
            ATheta = accelerationInfo.ATheta;
        }

        /// <summary>Applique une division membre par membre.</summary>
        public static AccelerationInfo operator /(AccelerationInfo left, double right)
            => new AccelerationInfo(left.AX / right, left.AY / right, left.ATheta / right);

        /// <summary>Passe les accélérations de cet objet de son référentiel au référentiel parent.</summary>
        /// <param name="theta">Angle du référentiel enfant dans le référentiel parent.</param>
        public AccelerationInfo ToParentRef(double theta)
        {
            // On prépare la matrice rotation pour passer du ref enfant au ref parent
            Matrix<double> rotationMatrix = Matrix<double>.Build.DenseOfArray(new[,]
            {
                { Math.Cos(-theta), Math.Sin(-theta) },
                { -Math.Sin(-theta), Math.Cos(-theta) }
            });

            // On récupère les vitesses X et Y dans le référentiel parent
            Vector<double> accelRefChild = Vector<double>.Build.DenseOfArray(new[] { AX, AY });
            Vector<double> accelRefParent = rotationMatrix * accelRefChild;

            return new AccelerationInfo
            {
                AX = accelRefParent[0],
                AY = accelRefParent[1],
                ATheta = ATheta
            };
        }

        /// <summary>Passe les accélérations de cet objet de son référentiel au référentiel enfant.</summary>
        /// <param name="theta">Angle du référentiel enfant dans le référentiel parent.</param>
        public AccelerationInfo ToChildRef(double theta)
        {
            // On prépare la matrice rotation pour passer du ref terrain au ref enfant
            Matrix<double> rotationMatrix = Matrix<double>.Build.DenseOfArray(new[,]
            {
                { Math.Cos(-theta), Math.Sin(-theta) },
                { -Math.Sin(-theta), Math.Cos(-theta) }
            });

            // On récupère les accélérations X et Y dans le référentiel enfant
            Vector<double> accelRefParent = Vector<double>.Build.DenseOfArray(new[] { AX, AY });
            Vector<double> accelRefChild = rotationMatrix.Inverse() * accelRefParent;

            return new AccelerationInfo
            {
                AX = accelRefChild[0],
                AY = accelRefChild[1],
                ATheta = ATheta
            };
        }
    }

    /// <summary>Décrit la position en X,Y,Theta et la vitesse en VX,VY,VTheta d'un objet.</summary>
    public class MovementInfo
    {
        /// <summary>Renvoie un <see cref="string"/> décrivant l'objet.</summary>
        public override string ToString()
            => $"({X:0.##}; {Y:0.##}; {Theta:0.##}) - ({VX:0.##}; {VY:0.##}; {VTheta:0.##})";

        #region Infos Objects

        /// <summary>Positions du mouvement.</summary>
        public PositionInfo Position { get; }

        /// <summary>Vitesses du mouvement.</summary>
        public SpeedInfo Speed { get; }

        /// <summary>Accélérations du mouvement.</summary>
        public AccelerationInfo Acceleration { get; }

        #endregion
        #region Value Properties

        /// <summary>Position en X en m.</summary>
        public double X
        {
            get => Position.X;
            set => Position.X = value;
        }

        /// <summary>Position en Y en m.</summary>
        public double Y
        {
            get => Position.Y;
            set => Position.Y = value;
        }

        /// <summary>Angle Theta en rad.</summary>
        public double Theta
        {
            get => Position.Theta;
            set => Position.Theta = value;
        }

        /// <summary>Vitesse en X en m/s.</summary>
        public double VX
        {
            get => Speed.VX;
            set => Speed.VX = value;
        }

        /// <summary>Vitesse en Y en m/s.</summary>
        public double VY
        {
            get => Speed.VY;
            set => Speed.VY = value;
        }

        /// <summary>Vitesse de rotation en rad/s.</summary>
        public double VTheta
        {
            get => Speed.VTheta;
            set => Speed.VTheta = value;
        }

        /// <summary>Accélération en X en m/s².</summary>
        public double AX
        {
            get => Acceleration.AX;
            set => Acceleration.AX = value;
        }

        /// <summary>Accélération en Y en m/s².</summary>
        public double AY
        {
            get => Acceleration.AY;
            set => Acceleration.AY = value;
        }

        /// <summary>Accélération de vitesse de rotation en rad/s².</summary>
        public double ATheta
        {
            get => Acceleration.ATheta;
            set => Acceleration.ATheta = value;
        }

        #endregion

        /// <summary>Crée une instance en copiant les valeurs données (met à 0 sinon).</summary>
        public MovementInfo(PositionInfo pos = null, SpeedInfo speed = null, AccelerationInfo accel = null)
        {
            Position = pos?.Copy() ?? new PositionInfo();
            Speed = speed?.Copy() ?? new SpeedInfo();
            Acceleration = accel?.Copy() ?? new AccelerationInfo();
        }

        /// <summary>Crée une instance en copiant les valeurs données (met à 0 sinon).</summary>
        public MovementInfo(double x, double y, double theta,
                            double vx = 0, double vy = 0, double vtheta = 0,
                            double ax = 0, double ay = 0, double atheta = 0)
        {
            Position = new PositionInfo(x, y, theta);
            Speed = new SpeedInfo(vx, vy, vtheta);
            Acceleration = new AccelerationInfo(ax, ay, atheta);
        }

        /// <summary>Renvoie une copie profonde de l'objet.</summary>
        public MovementInfo Copy() => new MovementInfo(Position, Speed, Acceleration);

        /// <summary>Copie les valeurs de movementInfo dans cette instance.</summary>
        public void FillWith(MovementInfo movementInfo)
        {
            Position.FillWith(movementInfo.Position);
            Speed.FillWith(movementInfo.Speed);
            Acceleration.FillWith(movementInfo.Acceleration);
        }

        /// <summary>Modifie les coordonnées du <see cref="MovementInfo"/> actuellement dans un référentiel parent dans un référentiel enfant.</summary>
        /// <param name="ref_x">Position x de l'origine du référentiel enfant dans le référentiel parent.</param>
        /// <param name="ref_y">Position y de l'origine du référentiel enfant dans le référentiel parent.</param>
        /// <param name="ref_theta">Angle theta en radians de l'origine du référentiel enfant dans le référentiel parent.</param>
        public void ToChildRef(double ref_x, double ref_y, double ref_theta)
        {
            Position.ToChildRef(ref_x, ref_y, ref_theta);
            Speed.ToChildRef(ref_theta);
            Acceleration.ToChildRef(ref_theta);
        }

        /// <summary>Modifie les coordonnées du <see cref="PositionInfo"/> actuellement dans un référentiel enfant dans un référentiel parent.</summary>
        /// <param name="ref_x">Position x de l'origine du référentiel enfant dans le référentiel parent.</param>
        /// <param name="ref_y">Position y de l'origine du référentiel enfant dans le référentiel parent.</param>
        /// <param name="ref_theta">Angle theta en radians de l'origine du référentiel enfant dans le référentiel parent.</param>
        public void ToParentRef(double ref_x, double ref_y, double ref_theta)
        {
            Position.ToParentRef(ref_x, ref_y, ref_theta);
            Speed.ToParentRef(ref_theta);
            Acceleration.ToParentRef(ref_theta);
        }
    }
}