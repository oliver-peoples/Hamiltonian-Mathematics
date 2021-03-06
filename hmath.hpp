#pragma once

/* Don't worry, I'm aware just how barren this is compared to what it should be based on this-|
																							  |
This is the hmath Hamiltonian Mathematics Library											  |
																							  |
				  █████          ██															  |
			   ██████  █        ████ █														  |
			  ██   █  █         █████														  |
			 █    █  █          █ █															  |
				 █  █           █															  |
				██ ██           █															  |
				██ ██           █															  |
				██ ██████████████															  |
				██ ██           █															  |
				██ ██           ██															  |
				█  ██           ██															  |
				   █             ██															  |
			   ████              ██															  |
			  █  █████            ██														  |
			 █     ██	       																  |
			 █			       																  |
			  █			       																  |
			   ██		       																  |
																							  |
This library encompasses vectors in 2d and 3d					 <----------------------------|
euclidean spaces, complex numbers, dual 3d vectors,				 <----------------------------|
quaternions, and dual quaternions.								 <----------------------------|
																 <----------------------------|
Some of these algebras are rather niche/complicated				 <----------------------------|
so I suggest you look at the wiki for reference on 				 <----------------------------|
the algebras and their implementation.							 <----------------------------|
																 <----------------------------|
For the dual algebras, I suggest you read Multi-Body			 <----------------------------|
Kinematics and Dynamics with Lie Groups by Chevallier			 <----------------------------|
Lerbet. This is quite a nice read, though quite technical.		 <----------------------------|
																 <----------------------------|
For the quaternions and complex numbers, I suggest Quaternions	 <----------------------------|
and Rotation Sequences by Jack B. Kuipers. This is another		 <----------------------------|
great read.

Please excuse the ascii art...it looks good on Visual Studio but nowhere else :(
*/

#include <cmath>
#include <iostream>

namespace hmath
{
	// constant expression mathematical constants

	constexpr long double H_PI = 3.1415926535897932384626433832795028841971693993751058209749445923078164062862089986280348253421170680;
	constexpr long double H_PI_2 = 1.5707963267948966192313216916397514420985846996875529104874722961539082031431044993140174126710585340;
	constexpr long double H_TAU = 6.2831853071795864769252867665590057683943387987502116419498891846156328125724179972560696506842341360;
	constexpr long double H_PI_squared = 9.8696044010893586188344909998761511353136994072407906264133493762200448224192052430017734037185522318;
	constexpr long double H_root_PI = 1.7724538509055160272981674833411451827975494561223871282138077898529112845910321813749506567385446654;
	constexpr long double H_e = 2.7182818284590452353602874713526624977572470936999595749669676277240766303535475945713821785251664274;
	constexpr long double H_e_2 = 1.3591409142295226176801437356763312488786235468499797874834838138620383151767737972856910892625832137;
	constexpr long double H_2_e = 5.4365636569180904707205749427053249955144941873999191499339352554481532607070951891427643570503328549;
	constexpr long double H_e_squared = 7.3890560989306502272304274605750078131803155705518473240871278225225737960790577633843124850791217948;
	constexpr long double H_root_e = 1.6487212707001281468486507878141635716537761007101480115750793116406610211942156086327765200563666430;

	// constant expressions for use in object construction

	typedef int HMATH_MAKE;

	constexpr HMATH_MAKE ELEMENT_WISE = 0;
	constexpr HMATH_MAKE VECTOR_TO = 1;
	constexpr HMATH_MAKE VECTOR3D_CROSS = 2;
	constexpr HMATH_MAKE NORMED = 3;
	constexpr HMATH_MAKE VECTOR_WISE = 4;
	constexpr HMATH_MAKE RADIANS = 5;
	constexpr HMATH_MAKE DEGREES = 6;
	constexpr HMATH_MAKE LINEAR_COMBINATION = 7;
	constexpr HMATH_MAKE IDENTITY = 8;

	/* ############################## MISC FUNCTIONS ############################## */

	double toRad(double degrees)
	{
		return degrees * (H_PI / 180);
	}

	double toDegree(double rad)
	{
		return rad * (180 / H_PI);
	}

	bool double_equality(double d1, double d2, double epsilon)
	{
		return std::abs(d1 - d2) < epsilon;
	}

	double roundN(double val, int decimal_place)
	{
		val = round( val * (pow(10, decimal_place)) ) / (pow(10, decimal_place));
		return val;
	}

	/* ############################## ALGEBRA CLASSES ############################## */

	//class Complex
	//{
	//public:
	//	double a, b;

	//	Complex();
	//	~Complex();

	//private:

	//};

	/* ############################## VECTOR2 ############################## */

	class Vector2
	{
	public:
		double i, j;

		Vector2() {}

		Vector2(double i, double j, HMATH_MAKE construct = ELEMENT_WISE)
		{
			if (construct == ELEMENT_WISE)
			{
				this->i = i;
				this->j = j;
			}
			else if (construct == NORMED)
			{
				double mag = sqrt(i * i + j * j);
				this->i = i / mag;
				this->j = j / mag;
			}
		}

		Vector2(hmath::Vector2 v1, hmath::Vector2 v2, HMATH_MAKE construct = VECTOR_TO)
		{
			if (construct == VECTOR_TO)
			{
				this->i = v2.i - v1.i;
				this->j = v2.j - v1.j;
			}
		}

		double norm()
		{
			return sqrt((this->i * this->i) + (this->j * this->j));
		}

		Vector2 normed()
		{
			hmath::Vector2 new_vector;
			new_vector.i = this->i / this->norm();
			new_vector.j = this->j / this->norm();
			return new_vector;
		}
	};

	double dot(Vector2 v1, Vector2 v2)
	{
		return v1.i * v2.i + v1.j * v2.j;
	}

	std::ostream& operator<<(std::ostream& os, Vector2 v)
	{
		os << v.i << ", " << v.j;
		return os;
	}

	// Vector2 operator+(Vector2& v1, Vector2& v2)
	// {
	// 	return Vector2(v1.i + v2.i, v1.j + v2.j);
	// }

	Vector2 operator+(Vector2 v1, Vector2 v2)
	{
		return Vector2(v1.i + v2.i, v1.j + v2.j);
	}

	void operator+=(Vector2& v1, Vector2 v2)
	{
		v1.i += v2.i;
		v1.j += v2.j;
	}

	Vector2 operator-(Vector2 v1, Vector2 v2)
	{
		return Vector2(v1.i - v2.i, v1.j - v2.j);
	}

	void operator-=(Vector2& v1, Vector2 v2)
	{
		v1.i -= v2.i;
		v1.j -= v2.j;
	}

	Vector2 operator*(Vector2 v, double d)
	{
		return Vector2(v.i * d, v.j * d);
	}

	/* ############################## VECTOR3 ############################## */

	class Vector3
	{
	public:
		double i, j, k;

		Vector3() {}

		Vector3(double i, double j, double k, HMATH_MAKE construct = ELEMENT_WISE)
		{
			if (construct == ELEMENT_WISE)
			{
				this->i = i;
				this->j = j;
				this->k = k;
			}
			else if (construct == NORMED)
			{
				double mag = sqrt(i * i + j * j + k * k);
				this->i = i / mag;
				this->j = j / mag;
				this->k = k / mag;
			}
		}

		Vector3(hmath::Vector3 v1, hmath::Vector3 v2, HMATH_MAKE construct = VECTOR_TO)
		{
			if (construct == VECTOR_TO)
			{
				this->i = v2.i - v1.i;
				this->j = v2.j - v1.j;
				this->k = v2.k - v1.k;
			}
			else if (construct == VECTOR3D_CROSS)
			{
				this->i = v1.j * v2.k - v1.k * v2.j;
				this->j = v1.k * v2.i - v1.i * v2.k;
				this->k = v1.i * v2.j - v1.j * v2.i;
			}
		}

		Vector3(Vector2 v)
		{
			this->i = v.i;
			this->j = v.j;
			this->k = 0;
		}

		Vector3 normed()
		{
			hmath::Vector3 new_vector;
			new_vector.i = this->i / this->norm();
			new_vector.j = this->j / this->norm();
			new_vector.k = this->k / this->norm();
			return new_vector;
		}

		double norm()
		{
			return sqrt((this->i * this->i) + (this->j * this->j) + (this->k * this->k));
		}

		void normalize()
		{
			double mag = this->norm();

			this->i *= 1 / mag;
			this->j *= 1 / mag;
			this->k *= 1 / mag;
		}

		double dot(Vector3 v1, Vector3 v2)
		{
			return v1.i * v2.i + v1.j * v2.j + v1.k * v2.k;
		}
	};

	std::ostream& operator<<(std::ostream& os, Vector3 v)
	{
		os << v.i << ", " << v.j << ", " << v.k;
		return os;
	}

	Vector3 operator+(Vector3 v1, Vector3 v2)
	{
		return Vector3(v1.i + v2.i, v1.j + v2.j, v1.k + v2.k);
	}

	void operator+=(Vector3& v1, Vector3 v2)
	{
		v1.i += v2.i;
		v1.j += v2.j;
		v1.k += v2.k;
	}

	Vector3 operator-(Vector3 v1, Vector3 v2)
	{
		return Vector3(v1.i - v2.i, v1.j - v2.j, v1.k - v2.k);
	}

	void operator-=(Vector3& v1, Vector3 v2)
	{
		v1.i -= v2.i;
		v1.j -= v2.j;
		v1.k -= v2.k;
	}

	Vector3 operator*(Vector3 v, double scalar)
	{
		return Vector3(v.i * scalar, v.j * scalar, v.k * scalar);
	}

	void operator*=(Vector3& v, double scalar)
	{
		v.i *= scalar;
		v.j *= scalar;
		v.k *= scalar;
	}

	/* ############################## LONG VECTOR3 ############################### */

	class lVector3
	{
	public:
		long double i, j, k;

		lVector3() {}

		lVector3(long double i, long double j, long double k, HMATH_MAKE construct = ELEMENT_WISE)
		{
			if (construct == ELEMENT_WISE)
			{
				this->i = i;
				this->j = j;
				this->k = k;
			}
			else if (construct == NORMED)
			{
				long double mag = sqrt(i * i + j * j + k * k);
				this->i = i / mag;
				this->j = j / mag;
				this->k = k / mag;
			}
		}

		lVector3(hmath::lVector3 v1, hmath::lVector3 v2, HMATH_MAKE construct = VECTOR_TO)
		{
			if (construct == VECTOR_TO)
			{
				this->i = v2.i - v1.i;
				this->j = v2.j - v1.j;
				this->k = v2.k - v1.k;
			}
			else if (construct == VECTOR3D_CROSS)
			{
				this->i = v1.j * v2.k - v1.k * v2.j;
				this->j = v1.k * v2.i - v1.i * v2.k;
				this->k = v1.i * v2.j - v1.j * v2.i;
			}
		}

		lVector3(Vector2 v)
		{
			this->i = v.i;
			this->j = v.j;
			this->k = 0;
		}

		lVector3 normed()
		{
			hmath::lVector3 new_vector;
			new_vector.i = this->i / this->norm();
			new_vector.j = this->j / this->norm();
			new_vector.k = this->k / this->norm();
			return new_vector;
		}

		long double norm()
		{
			return sqrt((this->i * this->i) + (this->j * this->j) + (this->k * this->k));
		}

		void normalize()
		{
			long double mag = this->norm();

			this->i *= 1 / mag;
			this->j *= 1 / mag;
			this->k *= 1 / mag;
		}		
	};

	long double dot(lVector3 v1, lVector3 v2)
	{
		return v1.i * v2.i + v1.j * v2.j + v1.k * v2.k;
	}

	std::ostream& operator<<(std::ostream& os, lVector3& v)
	{
		os << v.i << ", " << v.j << ", " << v.k;
		return os;
	}

	lVector3 operator+(lVector3& v1, lVector3& v2)
	{
		return lVector3(v1.i + v2.i, v1.j + v2.j, v1.k + v2.k);
	}

	void operator+=(lVector3& v1, lVector3 v2)
	{
		v1.i += v2.i;
		v1.j += v2.j;
		v1.k += v2.k;
	}

	lVector3 operator-(lVector3& v1, lVector3& v2)
	{
		return lVector3(v1.i - v2.i, v1.j - v2.j, v1.k - v2.k);
	}

	void operator-=(lVector3& v1, lVector3& v2)
	{
		v1.i -= v2.i;
		v1.j -= v2.j;
		v1.k -= v2.k;
	}

	lVector3 operator*(lVector3 v, long double scalar)
	{
		return lVector3(v.i * scalar, v.j * scalar, v.k * scalar);
	}

	void operator*=(lVector3& v, long double scalar)
	{
		v.i *= scalar;
		v.j *= scalar;
		v.k *= scalar;
	}

	/* ############################## SPHERICAL COORDINATE ############################## */

	class Spherical
	{
	public:
		double theta, phi, r;

		Spherical() {};

		Spherical(double theta, double phi, double r, HMATH_MAKE construct = RADIANS)
		{
			if (construct == RADIANS)
			{
				this->theta = theta;
				this->phi = phi;
				this->r = r;
			}
			else if (construct == DEGREES)
			{
				this->theta = theta * (H_PI / 180);
				this->phi = phi * (H_PI / 180);
				this->r = r;
			}
		}

		Spherical(Vector3 v)
		{
			this->theta = atan2(v.normed().j, v.normed().i);
			this->phi = asin(v.normed().k);
			this->r = v.norm();
		}

		Vector3 R3()
		{
			Vector3 v;
			v.i = this->r * cos(this->phi) * cos(this->theta);
			v.j = this->r * cos(this->phi) * sin(this->theta);
			v.k = this->r * sin(this->phi);
			return v;
		}

		std::string degreeString()
		{
			std::string return_string = std::to_string(toDegree(this->theta)) + ", " + std::to_string(toDegree(this->phi)) + ", " + std::to_string(this->r);
			return return_string;
		}
	};

	std::ostream& operator<<(std::ostream& os, Spherical& s)
	{
		os << s.theta << ", " << s.phi << ", " << s.r;
		return os;
	}

	/* ############################## DUAL VECTOR3 ############################## */

	class DualVector3
	{
	public:
		double i, j, k, ei, ej, ek;

		DualVector3() {}

		DualVector3(Vector3& A, Vector3& B, HMATH_MAKE construct = VECTOR_WISE)
		{
			if (construct == VECTOR_WISE)
			{
				this->i = A.i;
				this->j = A.j;
				this->k = A.k;
				this->ei = B.i;
				this->ej = B.j;
				this->ek = B.k;
			}
		}
	};

	std::ostream& operator<<(std::ostream& os, DualVector3& dv)
	{
		os << "( " << dv.i << " , " << dv.j << " , " << dv.k << " , " << dv.ei << " , " << dv.ej << " , " << dv.ek << " )";
		return os;
	}

	/* ############################## QUATERNION ############################## */

	class Quaternion
	{
	public:
		double w, i, j, k;

		Quaternion()
		{
			this->w;
			this->i;
			this->j;
			this->k;
		}

		Quaternion(double w, double i, double j, double k, HMATH_MAKE construct = ELEMENT_WISE)
		{
			if (construct == ELEMENT_WISE)
			{
				this->w = w;
				this->i = i;
				this->j = j;
				this->k = k;
			}
			else if (construct == NORMED)
			{
				double mag = sqrt(w * w + i * i + j * j + k * k);
				this->w = w / mag;
				this->i = i / mag;
				this->j = j / mag;
				this->k = k / mag;
			}
		}

		Quaternion(hmath::Vector3 axis, double theta)
		{
			double thetaOn2 = theta / 2;
			axis.normalize();

			this->w = cos(thetaOn2);
			this->i = sin(thetaOn2) * axis.i;
			this->j = sin(thetaOn2) * axis.j;
			this->k = sin(thetaOn2) * axis.k;
		}

        Vector3 getVectorComponent()
        {
            return Vector3(this->i, this->j, this->k);
        }

        double norm()
        {
            return sqrt(this->w*this->w + this->i*this->i + this->j*this->j + this->k*this->k);
        }

        Quaternion normalized()
        {
            double norm = this->norm();
            Quaternion normed(this->w / norm, this->i / norm, this->j / norm, this->k / norm);
            return normed;
        }

        void normalize()
        {
            double norm = this->norm();
            this->w /= norm;
            this->i /= norm;
            this->j /= norm;
            this->k /= norm;
        }

        Quaternion getConjugate()
        {
            Quaternion conjugate(this->w, -1 * this->i, -1 * this->j, -1 * this->k);
            return conjugate;
        }

        void makeConjugated()
        {
            this->i *= -1;
            this->j *= -1;
            this->k *= -1;
        }

        Quaternion getInverse()
        {
            double norm = this->norm();
            Quaternion inverse(this->w / norm, (this->i * -1) / norm, (this->j * -1) / norm, (this->k * -1) / norm);
            return inverse;
        }

        void makeInverted()
        {
            double norm = this->norm();
            this->w = (this->w * -1) / norm;
            this->i = (this->i * -1) / norm;
            this->j = (this->j * -1) / norm;
            this->k = (this->k * -1) / norm;
        }

		bool isIdentity(int power = 8)
		{
			double threshold = pow(10, power);

			bool scalar = (round(this->w * threshold) == threshold) ? true : false;
			bool i_comp = (round(this->i * threshold) == 0) ? true : false;
			bool j_comp = (round(this->j * threshold) == 0) ? true : false;
			bool k_comp = (round(this->k * threshold) == 0) ? true : false;
			
			if (scalar && i_comp && j_comp && k_comp)
			{
				return true;
			}
			else
			{
				return false;
			}
		}
	};

    Quaternion operator*(Quaternion q1, Quaternion q2)
	{
		Quaternion qp(0,0,0,0);
		qp.w = (q1.w * q2.w) - (q1.i * q2.i) - (q1.j * q2.j) - (q1.k * q2.k);
		qp.i = (q1.w * q2.i) + (q1.i * q2.w) + (q1.j * q2.k) - (q1.k * q2.j);
		qp.j = (q1.w * q2.j) - (q1.i * q2.k) + (q1.j * q2.w) + (q1.k * q2.i);
		qp.k = (q1.w * q2.k) + (q1.i * q2.j) - (q1.j * q2.i) + (q1.k * q2.w);
			
		return qp;
	}

	std::ostream& operator<<(std::ostream& os, Quaternion q)
	{
		os << std::to_string(q.w) + ", " << std::to_string(q.i) << ", " << std::to_string(q.j) << ", " << std::to_string(q.k);
		return os;
	}


	/* ############################## DUAL QUATERNION ############################## */

	class DualQuaternion
	{
	public:
		double w, i, j, k, e, ei, ej, ek;

		DualQuaternion() {};

		DualQuaternion(double w, double i, double j, double k, double e, double ei, double ej, double ek)
		{
			this->w = w;
			this->i = i;
			this->j = j;
			this->k = k;
			this->e = e;
			this->ei = ei;
			this->ej = ej;
			this->ek = ek;
		}
		
		DualQuaternion(Quaternion A, Quaternion B, HMATH_MAKE construct = LINEAR_COMBINATION)
		{
			this->w = A.w;
			this->i = A.i;
			this->j = A.j;
			this->k = A.k;
			this->e = B.w;
			this->ei = B.i;
			this->ej = B.j;
			this->ek = B.k;
		}

		DualQuaternion(Quaternion R, Vector3 T)
		{
			this->w = R.w;
			this->i = R.j;
			this->j = R.i;
			this->k = R.k;
			
			Quaternion Ton2(1, T.i * 0.5, T.j * 0.5, T.k * 0.5);

			Quaternion Tq = Ton2 * R;

			this->e = Tq.w;
			this->ei = Tq.i;
			this->ej = Tq.j;
			this->ek = Tq.k;
		}
	};

	Vector3 rotate(Vector3 point, Quaternion rotation)
	{
		Quaternion pure_qt(0, point.i, point.j, point.k);
		Quaternion pure_qt_prime = rotation * pure_qt * rotation.getConjugate();

		return Vector3(pure_qt_prime.i, pure_qt_prime.j, pure_qt_prime.k);
	}

	//Vector3::Vector3(hmath::Spherical s)
	//{
	//	this->i = cos(s.theta) * s.r;
	//	this->j = sin(s.theta) * s.r;
	//	this->k = sin(s.phi) * s.r;
	//}

	//Vector3:: Vector3(hmath::Spherical s, hmath::Vector3 local, hmath::Vector3 north_vector)
	//{
	//	hmath::Vector3 current(s);
	//	current.normalize();
	//	north_vector.normalize();
	//	hmath::Vector3 due_north(current, north_vector, VECTOR_TO);
	//	hmath::Vector3 new_x(due_north, current, VECTOR3D_CROSS);
	//	hmath::Vector3 new_y(current, new_x, VECTOR3D_CROSS);
	//	new_x.normalize();
	//	new_y.normalize();
	//}

	class VectorField2d
	{
	private:
		std::vector<std::vector<hmath::Vector2>> field;

	public:
		VectorField2d() {}
		~VectorField2d() {}

		std::vector<std::vector<hmath::Vector2>> getField()
		{
			return this->field;
		}
	};

	class VectorField3d
	{
	private:
		std::vector<std::vector<std::vector<hmath::Vector3>>> field;

	public:
		VectorField3d() {}
		~VectorField3d() {}

		std::vector<std::vector<std::vector<hmath::Vector3>>>& getField()
		{
			return this->field;
		}
	};	

	class Vector3Field4d
	{
	private:
		std::vector<std::vector<std::vector<std::vector<hmath::Vector3>>>> field;

	public:
		Vector3Field4d() {}
		~Vector3Field4d() {}

		std::vector<std::vector<std::vector<std::vector<hmath::Vector3>>>>& getField()
		{
			return this->field;
		}
	};


}