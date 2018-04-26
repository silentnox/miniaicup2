#include "json.hpp"
#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <map>
#include <float.h>
#include <deque>
#define _USE_MATH_DEFINES
#include <math.h>
#include <limits.h>

#pragma warning(disable:4244)
using namespace std;
using json = nlohmann::json;

double sqr(double x) { return x*x; };



struct Vec2 {
	double x, y;

	inline Vec2() :x(0), y(0) {};
	//inline Vec2(const Unit & v) :x(v.getX()), y(v.getY()) {};
	inline Vec2(double _x, double _y) :x(_x), y(_y) {};
	inline Vec2 operator-(Vec2 v) const {
		return Vec2(x - v.x, y - v.y);
	}
	inline Vec2 operator+(Vec2 v) const {
		return Vec2(x + v.x, y + v.y);
	}
	inline Vec2 operator*(double d) const {
		return Vec2(x*d, y*d);
	}
	inline bool operator==(Vec2 v) const {
		return (x == v.x && y == v.y);
	}
	inline Vec2 operator+=(Vec2 InV) {
		x += InV.x;
		y += InV.y;
		return *this;
	}
	inline double Dist(Vec2 in) const {
		return sqrt(sqr(in.x - x) + sqr(in.y - y));
	}
	inline double Dist2(Vec2 in) const {
		return sqr(in.x - x) + sqr(in.y - y);
	}
	inline double Angle(Vec2 in) const {
		double dot = x*in.x + y*in.y;
		double det = x*in.y - y*in.x;
		return atan2(det, dot);
	}
	double Dot(Vec2 In) const {
		return x*In.x + y*In.y;
	}
	inline double Len2() const {
		return sqr(x) + sqr(y);
	}
	inline double Len() const {
		return sqrt(sqr(x) + sqr(y));
	}
	double DistToRay(Vec2 Ray) const {

	}
	//void Normalise() {
	//	double l = Len();
	//	x /= l;
	//	y /= l;
	//}
	Vec2 Normalise() const {
		double vLen = Len();
		return Vec2(x / vLen, y / vLen);
	}
	void Rescale(double f) {
		Normalise();
		x *= f;
		y *= f;
	}
	void GetCellAt(int & OutX, int & OutY, double CellWidth) const {
		OutX = (int)ceil(x / CellWidth) - 1;
		OutY = (int)ceil(y / CellWidth) - 1;
	}
	bool IsZero() const {
		return x == 0 && y == 0;
	}
	bool IsEpsilon() const {
		return abs(x) <= FLT_EPSILON && abs(y) <= FLT_EPSILON;
	}
	static Vec2 CellCenter( int InX, int InY, double CellWidth ) {
		return Vec2( InX * CellWidth - CellWidth*0.5, InY * CellWidth - CellWidth*0.5);
	}
};

struct Seg {
	Vec2 From;
	Vec2 To;

	Seg(Vec2 InA, Vec2 InB) {
		From = InA;
		To = InB;
	}

	double DistToPoint(Vec2 InPoint) const {

	}
};

struct Line {
	Vec2	Normal;
	double	Dist;

	Line(Vec2 InNormal, double InDist) {
		Normal = InNormal;
		Dist = InDist;
	}

	Line(Vec2 InA, Vec2 InB) {
		Normal.x = InA.y - InB.y;
		Normal.y = InB.x - InA.x;
		Normal = Normal.Normalise();
		Dist = InA.x*InB.y - InB.x*InA.y;
	}

	double DistToPoint(Vec2 InPoint) const {
		return abs(Normal.x*InPoint.x + Normal.y*InPoint.y + Dist) / sqrt(sqr(Normal.x) + sqr(Normal.y));
	}
};

struct Rect {
	Vec2 Min, Max;

	Rect() {};
	Rect(Vec2 _min, Vec2 _max) :Min(_min), Max(_max) {};
	Rect(double minx, double miny, double maxx, double maxy) :Min(minx, miny), Max(maxx, maxy) {};
	Rect(Vec2 Center, double Radius) {
		Min.x = Center.x - Radius;
		Min.y = Center.y - Radius;
		Max.x = Center.x + Radius;
		Max.y = Center.y + Radius;
	}
	double Width() const {
		return Max.x - Min.x;
	}
	double Height() const {
		return Max.y - Min.y;
	}
	double Area() const {
		return Width() * Height();
	}

	Vec2 Center() const {
		return Vec2(Min.x + Width()*0.5, Min.y + Height()*0.5);
	}

	bool Contains(Vec2 P) const {
		return P.x > Min.x && P.x < Max.x && P.y > Min.y && P.y < Max.y;
	}

	bool Intersect(const Rect & r) const {
		return Contains(r.Min) || Contains(r.Max) || r.Contains(Min) || r.Contains(Max);
	}
	Vec2 Clamp(Vec2 v) const {
		return Vec2(min(Max.x, max(Min.x, v.x)), min(Max.y, max(Min.y, v.y)));
	}

	void AddPoint(const Vec2 & InPoint) {
		Min.x = min(Min.x, InPoint.x);
		Min.y = min(Min.y, InPoint.y);
		Max.x = max(Max.x, InPoint.x);
		Max.y = max(Max.y, InPoint.y);
	}

	void SetCenter(Vec2 InCenter) {

	}

	void Expand(double Factor) {
		Min.x -= Factor;
		Min.y -= Factor;
		Max.x += Factor;
		Max.y += Factor;
	}

	Rect Left() const {
		return Rect(Min, Vec2(Min.x + Width()*0.5, Max.y));
	}
	Rect Top() const {
		return Rect(Min, Vec2(Max.x, Min.y + Height()*0.5));
	}
	Rect Right() const {
		return Rect(Vec2(Min.x + Width()*0.5, Min.y), Max);
	}
	Rect Bottom() const {
		return Rect(Vec2(Min.x, Min.y + Height()*0.5), Max);
	}
};

template<typename T>
struct Field {
	struct Point {
		T		Value = 0;
		Vec2	Pos;

		Point() {
		}

		Point(const Vec2 & InPos, T InValue) {
			Pos = InPos;
			Value = InValue;
		}

		bool operator<(const Point & Right) const {
			return Value < Right.Value;
		}
		bool operator>(const Point & Right) const {
			return Value > Right.Value;
		}
	};

	Rect Bounds;

	vector<Point> Points;

	void InitSquare(Vec2 Center, int NumDivs, float InWidth) {
		Points.resize(NumDivs*NumDivs);

		float Step = InWidth / NumDivs;

		Bounds = Rect(Center, InWidth*0.5f);

		for (int Ind = 0; Ind < NumDivs*NumDivs; Ind++) {
			int Row = Ind / NumDivs;
			int Col = Ind % NumDivs;
			Points[Ind].Pos = Vec2(Center.x - InWidth*0.5f + Step*Col, Center.y - InWidth*0.5f + Step*Row);
		}
	}

	void InitCircle(Vec2 Center, float Radius, int NumDivs) {
		Points.resize(NumDivs);

		Bounds = Rect(Center, Radius);

		for (int Ind = 0; Ind < NumDivs; Ind++) {
			float Factor = (float)Ind / (float)NumDivs * 2.0f * M_PI;
			Points[Ind].Pos = Center + Vec2(cos(Factor), sin(Factor))*Radius;
		}
	}

	void AddPoint(Vec2 InPos) {
		Points.push_back(Point(InPos, 0));
	}

	void AddField(const Field & InField) {
		for (Point P : InField) {
			Points.push_back(P);
		}
	}

	void AddAttractor(Vec2 Pos, float Power) {

	}

	void AddDeflector(Vec2 Pos, float Power) {

	}

	Point Min() const {
		Point MinP = Points[0];
		for (const Point & P : Points) {
			if (MinP.Value > P.Value) MinP = P;
		}
		return MinP;
	}

	Point Max() const {
		Point MaxP = Points[0];
		for (const Point & P : Points) {
			if (MaxP.Value < P.Value) MaxP = P;
		}
		return MaxP;
	}

	Point MinLocal(Vec2 Center, float Radius) const {
		Point MinP;
		MinP.Pos = Center;
		MinP.Value = (T)INT_MAX;
		for (const Point & P : Points) {
			if (Center.Dist2(P.Pos) > sqr(Radius)) continue;
			if (MinP.Value > P.Value) MinP = P;
		}
		return MinP;
	}

	Point MaxLocal(Vec2 Center, float Radius) const {
		Point MaxP;
		MaxP.Pos = Center;
		MaxP.Value = -(T)INT_MAX;
		for (const Point & P : Points) {
			if (Center.Dist2(P.Pos) > sqr(Radius)) continue;
			if (MaxP.Value < P.Value) MaxP = P;
		}
		return MaxP;
	}

	void Normalise() {

	}
};
typedef Field<float> FltField;
struct Strategy {

	//struct FieldPoint {
	//	Vec2 Pos;
	//	double Value = 0.0;

	//	bool operator<(const FieldPoint & Other) const {
	//		return Value < Other.Value;
	//	}
	//};

	enum {
		UNKNOWN,
		FOOD,
		MINE,
		PLAYER,
		VIRUS,
		EJECT
	};

	struct Object {
		string Id;
		Vec2 Pos;
		Vec2 Vel;
		double Radius = 0.0;
		double Mass = 0.0;
		int Type = UNKNOWN;
		int TTF = -1;
		int Tick = 0; // last tick the object was visible
	};

	struct Command {
		enum {
			NONE,
			MOVE,
			EJECT,
			SPLIT
		};
		int Type = NONE;
		Vec2 Pos;

		string Debug;
	};

	struct Memory {
		map<string, Object> Objects;
		map<Vec2, int> MapPosToID;

		void AddObject(const Object & InObj) {

		}

		void ClearObjects(Vec2 Center, double Radius) {

		}

		void UpdateObjects() {

		}
	};

	Memory Mem;

	int GameWidth, GameHeight;
	int GameTicks;
	double FoodMass;
	int MaxFragments;
	int TicksUntilFusion;
	double VirusRadius;
	double VirusSplitMass;
	double Viscosity;
	double InertionFactor;
	double SpeedFactor;

	vector<Object> Objects;

	int TickNum = 0;

	int LastTickEnemy = 0;


	Field<int> VisitMap;

	ofstream Dbg;

	double GetStopDist( Vec2 Velocity, double Mass ) const {

	}

	Command Tick() {
		double MaxDist = Vec2(GameWidth, GameHeight).Len();
		Vec2 MapCenter = Vec2(GameWidth*0.5, GameHeight*0.5);
		Rect MapRect(0.0f, 0.0f, GameWidth, GameHeight);

		vector<Field<double>::Point> Final;

		double MinSelf = 1000, MaxSelf = 0;
		double MinPlayer = 1000, MaxPlayer = 0;

		int NumVisFood = 0;
		int NumVisPlayers = 0;
		int SelfFrags = 0;

		double TotalMass = 0.0;

		for (auto Obj : Objects) {

			if (Obj.Type == FOOD) NumVisFood++;
			if (Obj.Type == PLAYER) {
				NumVisPlayers++;
				LastTickEnemy = TickNum;

				MinPlayer = min(MinPlayer, Obj.Mass);
				MaxPlayer = max(MaxPlayer, Obj.Mass);
			}
			if (Obj.Type == MINE) {
				SelfFrags++;
				MinSelf = min(MinSelf, Obj.Mass);
				MaxSelf = max(MaxSelf, Obj.Mass);

				TotalMass += Obj.Mass;
			}
		}

		int NoEnemyTicks = TickNum - LastTickEnemy;

		for ( const auto & Self : Objects) {

			if (Self.Type != MINE) {
				continue;
			}

			Vec2 VelocityDir = Self.Vel.Normalise();
			Line VelocityLine = Line(Self.Pos, Self.Pos + Self.Vel);

			Vec2 VisCenter = Self.Pos + VelocityDir * 10;
			double VisRadius;

			if (SelfFrags < 2) {
				VisRadius = Self.Radius * 4.0;
			}
			else {
				VisRadius = Self.Radius * 2.5 * sqrt(SelfFrags);
			}

			for (auto & P : VisitMap.Points) {
				if (Rect(P.Pos, GameWidth / 16).Contains(Self.Pos)) {
					P.Value = TickNum;
				}
			}

			Field<double> PField;

			PField.InitSquare(Self.Pos, 16, 64.0);

			for (const Object & Obj : Objects) {
				if (Obj.Id == Self.Id) continue;
				if (!PField.Bounds.Contains(Obj.Pos)) continue;
				PField.AddPoint(Obj.Pos);
			}

			Object ClosestPlayer;
			Object ClosestVirus;

			Rect SafeRect = MapRect;

			//SafeRect.Expand(-Self.Radius*2.0f);
			//SafeRect.Expand(-64.0);

			Field<int>::Point VisitPoint = VisitMap.Min();

			for (Field<double>::Point & Point : PField.Points) {

				for (const auto & Obj2 : Objects) {

					if (Self.Id == Obj2.Id) continue;

					Vec2 TargetDir = (Obj2.Pos - Self.Pos).Normalise();
					double TargetDist = Point.Pos.Dist(Obj2.Pos);

					double AngFactor = max(0.5,(VelocityDir.Dot(TargetDir)+1)*0.5);
					double Factor = 1.0f / max(1.0,TargetDist);

					//Factor *= AngFactor;

					if (Obj2.Type == FOOD || Obj2.Type == EJECT) {
						Point.Value += 10.0 * Factor * AngFactor;
						continue;
					}
					if (Obj2.Type == PLAYER) {
						if (Self.Mass / Obj2.Mass > 1.2) Point.Value += 500.0 * Factor;
						if (Obj2.Mass / Self.Mass > 1.2) Point.Value += -1500.0 * Factor;
						continue;
					}
					if (Obj2.Type == VIRUS) {
						bool AvoidVirus = true;
						AvoidVirus &= Self.Mass > 120 || (TotalMass > 120 && Self.TTF < 15);
						AvoidVirus &= TargetDist < Self.Radius*2.0;
						//AvoidVirus &= Self.Radius > VirusRadius;
						if (AvoidVirus) {
							Point.Value += -700.0 * Factor;
						}
						continue;
					}
				}

				if (!NumVisPlayers) {
					Point.Value += (1.0 - 1.0 / sqr(MaxDist*0.5 - Point.Pos.Dist(MapCenter))) * 0.5;
					Point.Value += (1.0 / Point.Pos.Dist2(VisitPoint.Pos)) *  ((double)(TickNum - VisitPoint.Value) / (double)(TickNum + 1));
				}
			}

			if (PField.Min().Value < 0.0) {

				Vec2 WallPoint[4];

				WallPoint[0] = Vec2(0.0f, Self.Pos.y);
				WallPoint[1] = Vec2(Self.Pos.x, 0);
				WallPoint[2] = Vec2(GameWidth, Self.Pos.y);
				WallPoint[3] = Vec2(Self.Pos.x, GameHeight);


				for (int Ind = 0; Ind < 4; Ind++) {
					for (Field<double>::Point & Point : PField.Points) {
						double Factor = 1.0 / max(1.0, Point.Pos.Dist(WallPoint[Ind]));

						Point.Value += -1000.0 * Factor;
					}
				}

			}

			Field<double>::Point Max = PField.Max();

			Final.push_back(Max);
		}

		std::sort(Final.begin(), Final.end());

		Command Cmd;
		if (!Final.empty()) {
			Cmd.Pos = Final.back().Pos;
			Cmd.Type = Command::MOVE;
		}

		bool Split = true;

		Split &= NoEnemyTicks > 70 || MinSelf/(MaxPlayer+5) > 1.2;
		Split &= MinSelf > 80;
		//Split &= MaxSelf > 10;

		if (Split) {
			Cmd.Type = Command::SPLIT;
		}

		return Cmd;

	}

	void Parse( const json & InJson ) {
		for (json JsonObj : InJson) {

			Object Obj;

			Obj.Type = MINE;
			Obj.Tick = TickNum;

			for (json::iterator It = JsonObj.begin(); It != JsonObj.end(); ++It) {

				if (It.key() == "Id") Obj.Id = It.value().get<string>();
				if (It.key() == "X") Obj.Pos.x = It.value(); else 
				if (It.key() == "Y") Obj.Pos.y = It.value(); else
				if (It.key() == "SX") Obj.Vel.x = It.value(); else
				if (It.key() == "SY") Obj.Vel.y = It.value(); else
				if (It.key() == "R") Obj.Radius = It.value(); else
				if (It.key() == "M") Obj.Mass = It.value(); else
				if (It.key() == "TTF") Obj.TTF = It.value(); else
				if (It.key() == "T") {
					if (It.value() == "F") Obj.Type = FOOD; else
					if (It.value() == "E") Obj.Type = EJECT; else
					if (It.value() == "V") Obj.Type = VIRUS; else
					if (It.value() == "P") Obj.Type = PLAYER; else
					Obj.Type = MINE;
				}

			}

			Objects.push_back(Obj);
		}
	}

	void ParseConfig( const json & InJson ) {
		GameWidth = InJson["GAME_WIDTH"];
		GameHeight = InJson["GAME_HEIGHT"];
		FoodMass = InJson["FOOD_MASS"];
		MaxFragments = InJson["MAX_FRAGS_CNT"];
		GameWidth = InJson["GAME_WIDTH"];
		TicksUntilFusion = InJson["TICKS_TIL_FUSION"];
		VirusRadius = InJson["VIRUS_RADIUS"];
		VirusSplitMass = InJson["VIRUS_SPLIT_MASS"];
		Viscosity = InJson["VISCOSITY"];
		InertionFactor = InJson["INERTION_FACTOR"];
		SpeedFactor = InJson["SPEED_FACTOR"];
	}

	void Run() {
		Dbg.open("Debug.txt");

		string Raw;
		cin >> Raw;
		json Config = json::parse(Raw);
		ParseConfig(Config);

		VisitMap.InitSquare(Vec2(GameWidth, GameHeight)*0.5, 8, GameWidth);

		while (true) {
			cin >> Raw;
			json JsonData = json::parse(Raw);
			
			Objects.clear();
			Objects.reserve(10);

			Parse(JsonData["Mine"]);
			Parse(JsonData["Objects"]);

			Command Cmd = Tick();
			TickNum++;

			json Output;
			if (Cmd.Type != Command::NONE) {
				Output["X"] = Cmd.Pos.x;
				Output["Y"] = Cmd.Pos.y;
				if (Cmd.Type == Command::SPLIT) Output["Split"] = true; else
					if (Cmd.Type == Command::EJECT) Output["Eject"] = true;
			}

			//Output["Debug"] = "Test";
			//Output["Sprite"]["S"] = "Test";
			cout << Output.dump() << endl;
		}

		//Dbg.close();
	}

	void DebugDump( int NumTurns) {
		std::ofstream FileDump;
		FileDump.open("Dump.txt");
		string Raw;
		while (NumTurns) {
			cin >> Raw;
			FileDump << Raw << endl;
			json Cmd;
			Cmd["X"] = 0;
			Cmd["Y"] = 0;
			cout << Cmd << endl;
			NumTurns--;
		}
		FileDump.close();
	}
};

int main() {
	Strategy Strat;
	Strat.Run();
	return 0;
}