#pragma once

#include <VxGraphics/IGraphic.h>

#include <VxSim/IDynamics.h>
#include <VxSim/IExtension.h>
#include <VxSim/IApplicationListener.h>

class ExportScreenshots : public VxSim::IExtension
    , public VxGraphics::IGraphic
    , public VxSim::IDynamics
    , public VxSim::IApplicationListener
{
public:
    /// Constructor
    ///
    ExportScreenshots(VxSim::VxExtension* proxy);

    /// Destructor
    ///
    virtual ~ExportScreenshots();

    /// @internal
    virtual void preStep() override;
    /// @internal
    virtual void onAddToUniverse(Vx::VxUniverse* universe) override;
    /// @internal
    void onAddToView(VxGraphics::Viewport* viewport) override;
    /// @internal
    void onApplicationModeChange(VxSim::eApplicationMode previous, VxSim::eApplicationMode next) override;

    VxData::Field<Vx::VxFilename> pOutputPath;
    VxData::Field<bool> pSaveScreenshots;
    
private:
    VxGraphics::Viewport* mViewport;
    int mFrameCount;
};
