#pragma once
#include <vector>
#include <algorithm>
#include "projection.h"
#include "parameter.h"
#include <iostream>


Projection::Projection(Parameter* param)
{
    configTraces = param->getVConfigTrace();
    if (configTraces.empty())
    {
        cout << "Error:: Configuration of the trace is void" << endl;
    }

    m = param->getVMeasurements();
    if (m.empty())
    {
        cout << "Error:: Mesurements are void or wrong" << endl;
    }
};

Projection::Projection()
{
};

// Сравнение вещественных чисел 
bool isEqual(double x, double y, double eps) {
    return abs(x - y) == eps;
}

// Вычисление координаты точки для построения перпендикуляра к прямой AB
CoordinatsDecartDot perpendicularDot(CoordinatsDecartDot *a, CoordinatsDecartDot* b, CoordinatsDecartDot* c)
{
    //x и y - координаты вектора, перпендикулярного к AB
    double x = b->y - a->y; 
    double y = a->x - b->x;
    double L = (a->x * b->y - b->x * a->y + a->y * c->x - b->y * c->x + b->x * c->y - a->x * c->y) / (x * (b->y - a->y) + y * (a->x - b->x));
    CoordinatsDecartDot h;
    h.x = c->x + x * L;
    h.y = c->y + y * L;
    return h;
}

//Расстояние между двумя точками
double r(CoordinatsDecartDot *a, CoordinatsDecartDot* b)
{
    return sqrt(pow((a->x - b->x), 2) + pow((a->y - b->y), 2));
}

//Определить, лежит ли точка C на отрезке AB
bool thc(CoordinatsDecartDot* A, CoordinatsDecartDot* B, CoordinatsDecartDot* C)
{
    CoordinatsDecartDot a = { B->x - A->x, B->y - A->y };
    CoordinatsDecartDot b = { C->x - A->x, C->y - A->y};
        double sa = a.x * b.y - b.x * a.y;
        if (sa > 0.0)
            return false;
        if (sa < 0.0)
            return false;
        if ((a.x * b.x < 0.0) || (a.y * b.y < 0.0))
            return false;
        if (sqrt(a.x * a.x + a.y * a.y) < sqrt(b.x * b.x + b.y * b.y))
            return false;
        if ((isEqual(A->x, C->x, EPSILON))&&(isEqual(A->y, C->y, EPSILON)))
            return true;
        if ((isEqual(B->x, C->x, EPSILON)) && (isEqual(B->y, C->y, EPSILON)))
            return true;
        return true;      
}

//Определение угла наклона отрезка относительно оси OX в градусах
double angle(CoordinatsDecartDot* beg, CoordinatsDecartDot* end)
{
    double at2 = (atan2(end->y - beg->y, end->x - beg->x)) * 180 / M_PI;
    return (at2 < 0) ? 360 + at2 : at2;
}

//Метод -компаратор для оределения наиболее подходящего направления
bool dirComp(const ProjectionDot& a, const ProjectionDot& b)
{
    return (a.alpha) < (b.alpha);
}

//Метод -компаратор для оределения минимальной длины проекции
bool lenComp(const ProjectionDot& a, const ProjectionDot& b)
{
    return (a.l) < (b.l);
}

//Метод -компаратор для оределения минимального номера сегмента
bool numSegComp(const ProjectionDot& a, const ProjectionDot& b)
{
    return (a.numSegment) < (b.numSegment);
}

// Отфильтровать приближения для точки
// Можно задавать кооэффичиенты фильтрации по
//    - наиболее подходящее по номеру сегмента.
//        Можно отбрасывать проекции удалённые от текущего сегмента трассы больше чем FILTER_COOF_ANGLE
//    - наиболее подходящего направления.
//        Задаётся угол допустимого отклонения от направления вектора скорости в FILTER_COOF_ANGLE
//    - наиболее близкое по длине проекции к отрезку трассы.
//        Можно отбрасывать проекции, удалённые от отрезка дальше, чем FILTER_COOF_LEN
//
vector<ProjectionDot> Projection::filterProjection(vector<ProjectionDot>* prForDot)
{
    vector<ProjectionDot> prDirect;
    if (!(prForDot->empty()))
    {
        if (prForDot->size() == 1)
        {
            return *prForDot;
        }

        // Взять ближайший сегмент
        std::sort(prForDot->begin(), prForDot->end(), numSegComp);
        unsigned int v = (*prForDot)[0].numSegment;
        vector<ProjectionDot>::iterator it = std::find_if(prForDot->begin(), prForDot->end(), [&](const ProjectionDot& s)->bool { return (s.numSegment > v + FILTER_COOF_SEG_NUM); });
        prForDot->erase(it, prForDot->end());

        //2 Для всех подходящих направлений выбрать проекцию с подходящей длиной
         std::sort(prForDot->begin(), prForDot->end(), lenComp);
         double val = (*prForDot)[0].l;
         it = std::find_if(prForDot->begin(), prForDot->end(), [&](const ProjectionDot& s)->bool { return (s.l > val + FILTER_COOF_LEN); });
         prForDot->erase(it, prForDot->end());
         
        //Отфильтровать проекции по направлению
         std::sort(prForDot->begin(), prForDot->end(), dirComp);
         val = (*prForDot)[0].alpha;
         it = std::find_if(prForDot->begin(), prForDot->end(), [&](const ProjectionDot& s)->bool { return (s.alpha > val + FILTER_COOF_ANGLE); });
         prForDot->erase(it, prForDot->end());


}
    
    return *prForDot;
};

// Построить уравнение прямой AB
Straight create_straight(CoordinatsDecartDot A, CoordinatsDecartDot B)
{
    Straight s;
    s.A = A.y - B.y;
    s.B = B.x - A.x;
    s.C = A.x * B.y - B.x * A.y;
    return s;
};

// Найти точку пересечения прямых a и b
vector<CoordinatsDecartDot> intersection(Straight a, Straight b) 
{
    CoordinatsDecartDot ans;
    vector<CoordinatsDecartDot> res;
    ans.x = (b.C * a.B - a.C * b.B) / (a.A * b.B - b.A * a.B);
    ans.y = (a.C * b.A - b.C * a.A) / (b.B * a.A - a.B * b.A);
    if (!(isnan(ans.x)) && !(isnan(ans.y)))
    {
        res.push_back(ans);
    }
    return res;
};

// Определение всех проекций измерений к каждому сегменту трассы
// Для измерения создаётся вектор перемещения за время дельта Т
// Дельта Т делится на несколько отрезков длиной, задаваемой параметром LEN_INTERVAL
// Считаю, что вектор перемещения сонаправлен с вектором скорости, если считать, что движение прямолинейное
// Для каждой точки вычисляется проекция на каждый сегмент трассы
vector<ProjectionDot> Projection::allProjectionForMeasurement(vector<ConfigTrace>*trace, unsigned int currentIndex, Measurements* m)
{
    vector<ProjectionDot> res;
    double deltaR = 0.0;        // длина вектора перемещения
    double deltaX = 0.0;
    double deltaY = 0.0;
    double lenProj;             // длина проекции
    
    unsigned long deltaT = m->t;
    CoordinatsDecartSection sec;

    for (unsigned int i = currentIndex; i < trace->size()-1; i++)
    {
        sec.beg = { (*trace)[i].x, (*trace)[i].y };
        sec.end = { (*trace)[i + 1].x, (*trace)[i + 1].y };

        double curX = m->x;
        double curY = m->y;

        //Для каждого интервала
        for (unsigned int interval = 0; interval < (deltaT / LEN_INTERVAL)+1; interval++)
        {
            CoordinatsDecartDot currentXY = { curX, curY };
            CoordinatsDecartDot dot = perpendicularDot(&sec.beg, &sec.end, &currentXY);
            Straight s = create_straight(sec.beg, sec.end);
            Straight sp = create_straight(currentXY, dot);

            //Проверям, попадает ли сама точка на отрезок
            if (thc(&sec.beg, &sec.end, &currentXY))
            {
                res.push_back({ currentXY , 0, abs(angle(&sec.beg,&sec.end) - m->phi),i });
            }
            else {
                //Находим точку пересечения отрезка трассы и перпендикуляра к нему
                vector<CoordinatsDecartDot> vProjDot = intersection(s, sp);
                if (!vProjDot.empty())
                {
                    lenProj = r(&(vProjDot[0]), &currentXY);

                    //Проверяем, попадает ли точка пересечения на отрезок
                    if (thc(&sec.beg, &sec.end, &vProjDot[0]))
                    {
                        res.push_back({ vProjDot[0] , lenProj, abs(angle(&sec.beg,&sec.end) - m->phi),i });
                    }
                }
            }
            
            //Определить длину перемещения  (v/t) и координаты конца
            deltaR = m->v * (LEN_INTERVAL);
            deltaX = cos(m->phi * M_PI / 180) * deltaR;
            deltaY = sin(m->phi * M_PI / 180) * deltaR;
            curX = curX + deltaX;
            curY = curY + deltaY;
        }
        sec.beg = { (*trace)[i].x, (*trace)[i].y };
        sec.end = { (*trace)[i + 1].x, (*trace)[i + 1].y };
    }
    return res;
};

//Получение проекций измерения
vector<ProjectionDot> Projection::getProjection()
{
    CoordinatsDecartSection sec;

    vector<ProjectionDot> vBest;
    unsigned int currentSegInd = 0;

    if (!configTraces.empty())
    {

        sec.beg = { configTraces[0].x , configTraces[0].y };
        sec.end = { configTraces[1].x, configTraces[1].y };

        unsigned int cInd = 0;
        if (!m.empty()) {
            for (unsigned int i = 0; i < m.size(); i++)
            {
                if (cInd < configTraces.size() - 1)
                {
                    vector<ProjectionDot> vPr;
                    unsigned int indPr = 0;
                    vector<ProjectionDot> pD = allProjectionForMeasurement(&configTraces, cInd, &m[i]);
                    if (!pD.empty())
                    {
                        for (unsigned a = 0; a < pD.size(); a++)
                        {
                            vPr.push_back(pD[a]);
                        }
                        vector<ProjectionDot> bPr = filterProjection(&vPr);

                        if (!bPr.empty())
                        {

                            for (unsigned int k = 0; k < bPr.size(); k++) {
                                vBest.push_back(bPr[k]);
                            }
                            cInd = bPr[bPr.size() - 1].numSegment;


                        }
                    }
                }

            }
        }
    }
    return vBest;
};
