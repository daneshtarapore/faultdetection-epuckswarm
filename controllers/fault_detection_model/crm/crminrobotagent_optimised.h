#ifndef CRMINROBOTAGENTOPTIMISED_H
#define CRMINROBOTAGENTOPTIMISED_H


/******************************************************************************/
/******************************************************************************/
// FAULT DETECTION MODEL TYPE
enum faultdetectionmodeltype {CRM, CRM_TCELLSINEXCESS, CTRNN, LINEQ, THRESHOLDONFVDIST};

#define FDMODELTYPE  CRM_TCELLSINEXCESS

//#define DEBUGCROSSREGULATIONMODELFLAG

#define MODELSTARTTIME 450.0 //100.0

/******************************************************************************/
/******************************************************************************/
#include <assert.h>
#include <limits.h>
#include <math.h>
#include <list>
#include <iostream>

/******************************************************************************/
/******************************************************************************/

#include "celldatacontainers.h"
#include "featurevectorsinrobotagent.h"
#include "propriofeaturevector.h"

/******************************************************************************/
/******************************************************************************/

using namespace std;

/******************************************************************************/
/******************************************************************************/

struct structTcell; struct structAPC;
enum ConjugationIntegrationPhase : unsigned {CONJ_K0, CONJ_K1, CONJ};
enum TcellIntegrationPhase : unsigned {K0, K1};

/******************************************************************************/
/******************************************************************************/

class CRMinRobotAgentOptimised
{
public:
    CRMinRobotAgentOptimised(unsigned m_uRobotId, unsigned NUMBER_OF_FEATURES);

    virtual ~CRMinRobotAgentOptimised();

    virtual double GetCurrE(unsigned int thtype);
    virtual double GetCurrR(unsigned int thtype);

    virtual void SetCurrE(unsigned int thtype, double f_currE);
    virtual void SetCurrR(unsigned int thtype, double f_currR);

    virtual double GetAPC(unsigned int apctype);

    virtual void FreeTcellsAndAvailableAPCSites(TcellIntegrationPhase TK, ConjugationIntegrationPhase CONJK);

    virtual void ConjugatesQSS(bool bResetConjugates, TcellIntegrationPhase TK);
    virtual void Derivative(TcellIntegrationPhase TK);

    virtual void ConjugatesQSS_ExcessTcells(bool bClearDeadConjugates, TcellIntegrationPhase TK);
    virtual void Derivative_ExcessTcells(TcellIntegrationPhase TK);
    virtual void ComputeNewDerivative(TcellIntegrationPhase TK);

    virtual inline double GetFVtoApcScaling() {return m_fFVtoApcscaling;}

    virtual inline double GetConvergenceError() {return m_dconvergence_error;}
    virtual inline double GetConvergenceError_Perc() {return m_dpercconvergence_error;}

    virtual void PrintCRMDetails(unsigned int id);
    virtual void PrintFeatureVectorDistribution(unsigned int id);
    virtual void PrintAPCList(unsigned int id);
    virtual void PrintTcellResponseToAPCList(unsigned int id);
    virtual void PrintTcellList(unsigned int id);
    virtual void PrintConjugatestoAPCList(unsigned int id, ConjugationIntegrationPhase CONJK);
    virtual void PrintConjugatestoTcellList(unsigned int id, ConjugationIntegrationPhase CONJK);

    virtual void TcellNumericalIntegration_RK2();
    virtual void SimulationStepUpdatePosition(double m_fInternalRobotTimer, t_listFVsSensed *listFVsSensed);
    virtual void DiffuseTcells();

    void ScaleDownConjugates(ConjugationIntegrationPhase CONJK);


    virtual inline void IncNumberFloatingPtOperations() {++m_uNumberFloatingPtOperations;}
    virtual inline void IncNumberFloatingPtOperations(unsigned int count)
        {for(unsigned int i=0;i<count;++i) IncNumberFloatingPtOperations();}


    inline list<structAPC>*    GetListAPCs() {return &listAPCs;}
    inline list<structTcell>*  GetListTcells() {return &listTcells;}

    virtual inline void SetMostWantedList(t_listFVsSensed::iterator* it, unsigned int state)
    {
        t_listFVsSensed::iterator it_fvsensed = (*it);
        (*it_fvsensed).uMostWantedState = state;
    }

    virtual inline unsigned GetIdentification() {return m_uRobotId;}

    static double NegExpDistAffinity(unsigned int v1, unsigned int v2, double k);
    static unsigned int GetNumberOfSetBits(unsigned int x);

protected:

    virtual void UpdateState();
    virtual void UpdateAPCList(); //Sense()
    virtual void UpdateTcellList(unsigned int hammingdistance); //unsigned hammingdistance
    virtual void UpdateConjugatesToAPCList();
    virtual void UpdateConjugatesToTcellList();
    virtual void MarkConjugatesOfDeadTcell(list<structTcell>::iterator* ptrit_tcells);

    virtual inline double GetWeight() {return m_fWeight;}

    unsigned long long m_uNumberFloatingPtOperations;

    t_listFVsSensed*              ptr_listFVsSensed;


    double step_h; double conjstep_h; // internal step count of the CRM instance
    double seedE; // : Density of effector cells at the start
    double seedR; // : Density of regulatory cells at the start
    double kon;   // : Conjugation rate
    double koff;  // : Dissociation rate
    double kpe;   // : Proliferation rate for effector cells
    double kde;   // : Death rate for effector cells
    double kpr;   // : Proliferation rate for regulatory cells
    double kdr;   //  Death rate for regulatory cells
    double se;    // Density of new effector cells added at each simulation step
    double sr;    // Density of new regulatory cells added at each simulation step
    double se_rate, sr_rate; // Rate of influx of new T-cells
    unsigned int sites; // Number of binding sites on each APC
    double m_fIntegrationTime;

    double m_fTCELL_UPPERLIMIT_STEPSIZE, m_fTCELL_LOWERLIMIT_STEPSIZE;
    double m_fERRORALLOWED_TCELL_STEPSIZE, m_fERRORALLOWED_CONJ_STEPSIZE;
    double m_fTCELL_CONVERGENCE, m_fCONJ_CONVERGENCE; //-3//todo: set as percentage instead of absolute value

    // For communication of cells between robots
    double m_fTryExchangeProbability; // Probability of trying to exchange cells with other robots
    //double m_fExchangeRange;


    unsigned m_uHistoryTcells; // a history of t-cell populations (at each of the previous m_uHistoryTcells simulation time-steps)
    float m_fSuspicionThreshold; // Threshold above which a FV is to be tolerated - but deemed suspicious [0,1]
    float m_fNewFVSuspicionIncr;


    list<structTcell> listTcells, listTcells_cpy; // list of non-zero t-cell clonaltypes and a copy in case integration has to be run again
    list <list<structTcell> > listlistTcells; // a record of T-cells over previous X simulation steps

    list<structAPC>   listAPCs;
    virtual inline void IncIt(list<structTcell>::iterator *it_tcell, list<structTcell>* list)
    { (*it_tcell) == list->end() ? (*it_tcell):++(*it_tcell); }
    virtual inline void IncIt(list<structAPC>::iterator *it_apc, list<structAPC>* list)
    { (*it_apc)   == list->end() ? (*it_apc):++(*it_apc); }

    unsigned int    m_unNumberOfReceptors;

    double          m_fcross_affinity; /* the level of cross affinity*/

    double          m_fWeight;
    double          m_fFVtoApcscaling; //linear scaling - multiplicative factor

    bool            m_bConvergenceFlag;
    double          m_dconvergence_error;
    double          m_dpercconvergence_error;


    double      m_fInternalRobotTimer;
    unsigned    m_uRobotId;
};


/******************************************************************************/
/******************************************************************************/

#endif // CRMINROBOTAGENTOPTIMISED_H
