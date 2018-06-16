#include "celldatacontainers.h"

/******************************************************************************/
/******************************************************************************/

structConj::structConj(structTcell* tc, structAPC* apc, double cross_affinity)
{
    ptrTcell = tc; ptrAPC = apc;
    utcellFV = ptrTcell->uFV; uapcFV = ptrAPC->uFV;
    affinity = CRMinRobotAgentOptimised::NegExpDistAffinity(utcellFV, uapcFV, cross_affinity);

    fConjugates    = 0.0;
    deadconjugate  = false;
};

/******************************************************************************/
/******************************************************************************/

double structConj::GetConjugate(ConjugationIntegrationPhase phase)
{
    if(phase == CONJ_K0)
        return fConjugates_k0;
    else if(phase == CONJ_K1)
        return fConjugates_k1;
    else
        return fConjugates;
}

/******************************************************************************/
/******************************************************************************/

void structConj::SetConjugate(ConjugationIntegrationPhase phase, double conj)
{
    if(phase == CONJ_K0)
        fConjugates_k0 = conj;
    else if(phase == CONJ_K1)
        fConjugates_k1 = conj;
    else
        fConjugates = conj;
}

/******************************************************************************/
/******************************************************************************/

double structConj::GetConjugateDelta(ConjugationIntegrationPhase phase)
{
    return phase == CONJ_K0 ? fDelta_k0 : fDelta_k1;
}

/******************************************************************************/
/******************************************************************************/

void structConj::SetConjugateDelta(ConjugationIntegrationPhase phase, double conjdelta)
{
    phase == CONJ_K0 ? fDelta_k0=conjdelta : fDelta_k1=conjdelta;
}

/******************************************************************************/
/******************************************************************************/

structTcell::structTcell(unsigned int fv, double seedE, double seedR, unsigned long history, structAPC* ptrAPC)
{
    uFV      = fv;
    uHistory = history;
    fE       = seedE;
    fR       = seedR;
    ptrAPCWithAffinity1 = ptrAPC;
}

/******************************************************************************/
/******************************************************************************/

structTcell::~structTcell()
{
    listPtrstoConjugatesofTcell.clear();
}

/******************************************************************************/
/******************************************************************************/

double structTcell::GetE(TcellIntegrationPhase K)
{
    return K == K0 ? fE : fE_Eu;
}

/******************************************************************************/
/******************************************************************************/

void structTcell::SetE(TcellIntegrationPhase K, double e)
{
    K == K0 ? fE = e : fE_Eu = e;
}

/******************************************************************************/
/******************************************************************************/

double structTcell::GetR(TcellIntegrationPhase K)
{
    return K == K0 ? fR : fR_Eu;
}

/******************************************************************************/
/******************************************************************************/

void structTcell::SetR(TcellIntegrationPhase K, double r)
{
    K == K0 ? fR = r : fR_Eu = r;
}

/******************************************************************************/
/******************************************************************************/

void structTcell::GetDelta(TcellIntegrationPhase K, double *deltaE, double *deltaR)
{
    K == K0 ? (*deltaE = fDeltaE_k0) : (*deltaE = fDeltaE_k1);
    K == K0 ? (*deltaR = fDeltaR_k0) : (*deltaR = fDeltaR_k1);
}

/******************************************************************************/
/******************************************************************************/

void structTcell::SetDelta(TcellIntegrationPhase K, double deltaE, double deltaR)
{
    K == K0 ? (fDeltaE_k0 = deltaE) : (fDeltaE_k1 = deltaE);
    K == K0 ? (fDeltaR_k0 = deltaR) : (fDeltaR_k1 = deltaR);
}

/******************************************************************************/
/******************************************************************************/

structAPC::structAPC(unsigned int fv, double apc, double sites)
{
    uFV  = fv; fAPC = apc;
    fTotalSites = apc * sites;
}

/******************************************************************************/
/******************************************************************************/

structAPC::~structAPC()
{
    listConjugatesonAPC.clear();
}

/******************************************************************************/
/******************************************************************************/

// updates the list of conjugates for a given apc
unsigned structAPC::UpdateConjugateList(list<structTcell>* tcells, double cross_affinity)
{
    list<structTcell>::iterator it_tcells; list<structConj>::iterator  it_conjs;
    it_tcells = tcells->begin(); it_conjs = listConjugatesonAPC.begin();

    unsigned u_NumFpOperations = 0;
    while(it_conjs != listConjugatesonAPC.end() && it_tcells != tcells->end())
    {
        if((*it_conjs).utcellFV == (*it_tcells).uFV)
        {   if((*it_conjs).deadconjugate)
            {
                (*it_conjs).deadconjugate = false; (*it_conjs).ptrTcell = &(*it_tcells);
            }
            ++it_conjs; ++it_tcells; continue;
        }

        if((*it_conjs).utcellFV > (*it_tcells).uFV)
        {
            //! TODO: ONLY ADD CONJUGATES IF AFFINITY > 0. THHIS WILL SAVE ON COMPUTATION IN WALKING SHORTER CONJUGATE LISTS, AND IN MEMORY USED TO STORE THESE LISTS.
            //if(CRMinRobotAgentOptimised::NegExpDistAffinity((*it_tcells).uFV, this->uFV, cross_affinity) > 0.0)
            {
                listConjugatesonAPC.insert(it_conjs, structConj(&(*it_tcells), this, cross_affinity));
#ifdef FLOATINGPOINTOPERATIONS
                u_NumFpOperations += 3;
#endif
            }
            ++it_tcells;
        }
        else
            it_conjs = listConjugatesonAPC.erase(it_conjs);
    }

    while(it_conjs != listConjugatesonAPC.end())
        it_conjs = listConjugatesonAPC.erase(it_conjs);

    while(it_tcells != tcells->end())
    {
        //! TODO: ONLY ADD CONJUGATES IF AFFINITY > 0. THHIS WILL SAVE ON COMPUTATION IN WALKING SHORTER CONJUGATE LISTS, AND IN MEMORY USED TO STORE THESE LISTS.
        //if(CRMinRobotAgentOptimised::NegExpDistAffinity((*it_tcells).uFV, this->uFV, cross_affinity) > 0.0)
        {
            listConjugatesonAPC.push_back(structConj(&(*it_tcells), this, cross_affinity));
#ifdef FLOATINGPOINTOPERATIONS
            u_NumFpOperations += 3;
#endif
        }
        ++it_tcells;
    }
    return u_NumFpOperations;
}

/******************************************************************************/
/******************************************************************************/
