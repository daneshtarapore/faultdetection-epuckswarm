#ifndef CELLDATACONTAINERS_H
#define CELLDATACONTAINERS_H

/******************************************************************************/
/******************************************************************************/
#include <list>
//#include "robotagent_optimised.h"
#include "crminrobotagent_optimised.h"
/******************************************************************************/
/******************************************************************************/

using namespace std;

/******************************************************************************/
/******************************************************************************/

class CRMinRobotAgentOptimised;

/******************************************************************************/
/******************************************************************************/
struct structTcell; struct structAPC;
enum ConjugationIntegrationPhase : unsigned;
enum TcellIntegrationPhase : unsigned;
/******************************************************************************/
/******************************************************************************/

struct structConj
{
    bool deadconjugate;
    unsigned int utcellFV, uapcFV;

    double affinity;
    double fConjugates, fEffectorConjugates, fRegulatorConjugates;

    double fConjugates_k0, fConjugates_k1; // for the computation of conjugates in QSS
    double fDelta_k0, fDelta_k1;             // for the computation of conjugates in QSS

    structTcell* ptrTcell;
    structAPC*   ptrAPC;

    structConj(structTcell* tc, structAPC* apc, double cross_affinity);

    double GetConjugate(ConjugationIntegrationPhase CONJK);
    void SetConjugate(ConjugationIntegrationPhase CONJK, double conj);
    double GetConjugateDelta(ConjugationIntegrationPhase CONJK);
    void SetConjugateDelta(ConjugationIntegrationPhase CONJK, double conjdelta);
};

/******************************************************************************/
/******************************************************************************/

struct structTcell
{
    unsigned int uFV;

    unsigned long int uHistory;

    double  fE, fR;
    double  fE_prev, fR_prev;

    double fFreeTcells;

    // predicted number of cells at time t+step with Euler and Huen methods
    double  fE_Eu, fR_Eu, fE_Hu, fR_Hu;

    // the slopes at time = t and time = t+step
    double  fDeltaE_k0, fDeltaR_k0, fDeltaE_k1, fDeltaR_k1;
    structAPC*   ptrAPCWithAffinity1; // maintain a pointer to apc with highest affinity - for fast accesss when logging data

    list<structConj*> listPtrstoConjugatesofTcell;

    structTcell(unsigned int fv, double seedE, double seedR, unsigned long history, structAPC* ptrAPC);
    ~structTcell();

    double GetE(TcellIntegrationPhase K);
    void   SetE(TcellIntegrationPhase K, double e);

    double GetR(TcellIntegrationPhase K);
    void   SetR(TcellIntegrationPhase K, double r);

    void GetDelta(TcellIntegrationPhase K, double *deltaE, double *deltaR);
    void SetDelta(TcellIntegrationPhase K, double deltaE, double deltaR);
};

/******************************************************************************/
/******************************************************************************/

struct structAPC
{
    unsigned int uFV;
    double       fAPC;
    double       fAvailableSites, fTotalSites;
    double       fTotalConjugates; // total number of conjugates on apc. used to speed up rescaling of conjugates (upon overflow); and in its computation when assuming tcells in excess of apcs sites

    double       fEffectorConjugatesPerAPC, fRegulatorConjugatesPerAPC;
    double       fE_weightedbyaffinity, fR_weightedbyaffinity;

    // used in the computation of conjugates when assuming excess of t-cells
    double       f_tcellsweightedaffinity_tmp, f_ecellsweightedaffinity_tmp, f_rcellsweightedaffinity_tmp;

    list<structConj> listConjugatesonAPC;
//    unsigned int uMostWantedListState;

    structAPC(unsigned int fv, double apc, double sites);
    ~structAPC();

    // updates the list of conjugates for a given apc
    // returns the number of fp operations
    unsigned UpdateConjugateList(list<structTcell>* tcells, double cross_affinity);
};

/******************************************************************************/
/******************************************************************************/

#endif // CELLDATACONTAINERS_H
