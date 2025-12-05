#ifndef _MY_SOLVER_EXTENSION_H
#define _MY_SOLVER_EXTENSION_H

#include <VxSim/IDynamics.h>
#include <VxSim/IExtension.h>
#include <VxData/Container.h>
#include <VxData/Field.h>
#include <Vx/VxTimer.h>

namespace Vx
{
    class VxUniverse;
    class VxFrameStepperOption;
}

namespace VxSim
{
    class VxPluginExtension;
}

class ExternalKeaConstraintSolver;
struct DebugData;


/// This class encapsulates several custom solver implementations.
///
class MySolverExtension : public VxSim::IDynamics, public VxSim::IExtension
{
public:
    /// Constructor
    MySolverExtension(VxSim::VxExtension *iProxy);
    
    /// Destructor
    virtual ~MySolverExtension();

    virtual void onAddToUniverse( Vx::VxUniverse * universe );

    virtual void onRemoveFromUniverse( Vx::VxUniverse * universe );

    virtual void preStep();
    virtual void postStep();

public:

    VxData::Field<bool> pWriteDebugDevelData;        ///< If true, saves debug and development data.
    VxData::Field<std::string> pDebugOutputFolder;  

    VxData::Container extendedSolverParams;
    VxData::Container directSolverParams;
    VxData::Container iterativeSolverParams;

    // Solver specific parameters
    VxData::Field<double> pIterativeSolverTol;  
    VxData::Field<double> pStandardSolverTol;
    VxData::Field<unsigned int> pIterativeSolverMaxIter;
    VxData::Field<unsigned int> pStandardSolverMaxIter;
    VxData::Field<unsigned int> pScaledBoxIter;
    VxData::Field<bool> pUseScaledBox;
    VxData::Field<unsigned int> pPivotingStrategy;

    // Judice parameters
    VxData::Field<bool> pPGSFriction;     ///< Compute friction using PGS.
    VxData::Field<bool> pCycleDetect;

    // Projected Gauss-Siedel parameters
    VxData::Field<double> pPGSEpsilon;          ///< regularization parameter.
    VxData::Field<double> pPGSSor;              ///< successive over relaxation parameter.
    VxData::Field<bool> pPGSNonlinearCP;        ///< nonlinear complementarity problem
    VxData::Field<bool> pPGSUsePreconditioner;

    // PGS-SM parameters
    VxData::Field<int> pKpgs;
    VxData::Field<int> pKsm;

    VxData::Field<int> pSolverIndex;
    VxData::Field<double> pTimestep;

    VxData::Field<std::string> oSolverName;

    DebugData* m_debugData;    
    int m_debugFrame;
    unsigned int numOfStrategies; // number of different pivoting strategies
};

#endif // _FRAMESTEPPEROPTION_DYNAMICS_EXTENSION_H
