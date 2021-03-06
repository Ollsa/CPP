#include "parameter.h"

#define M_PI 3.14159265358979323846
#define LEN_INTERVAL 1

//Коэффициенты фильтрации проекций
#define FILTER_COOF_LEN		0.3		// коэффициент отклонения длины проекции точки на сегмент
#define FILTER_COOF_SEG_NUM	1		// коэффициент допустимого номера сегмента
#define FILTER_COOF_ANGLE	5		// коэффициент отклонения вектора от сегмента трассы в градусах

struct CoordinatsDecartDot
{
	double x;
	double y;
};

struct CoordinatsDecartSection
{
	CoordinatsDecartDot beg;
	CoordinatsDecartDot end;
};

struct ProjectionDot
{
	CoordinatsDecartDot xy;
	double l;
	double alpha;
	unsigned int numSegment;
};

class Projection
{
private:
	vector<ConfigTrace> configTraces;
	vector<Measurements> m;

public:
	Projection(Parameter *param);
	vector<ProjectionDot> allProjectionForMeasurement(vector<ConfigTrace>* trace, unsigned int currentIndex, Measurements* m);
	vector<ProjectionDot> filterProjection(vector<ProjectionDot>* prForM);
	double angle(CoordinatsDecartDot* beg, CoordinatsDecartDot* end);
	vector<ProjectionDot>getProjection();
};