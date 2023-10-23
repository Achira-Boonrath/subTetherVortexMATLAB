#include "ExportScreenshots.h"

#include <VxGraphics/WindowingSystem.h>
#include <VxGraphics/Window.h>
#include <VxGraphics/Viewport.h>

#include <VxPlatform/Directory.h>

ExportScreenshots::ExportScreenshots(VxSim::VxExtension* proxy)
    : IExtension(proxy)
    , VxGraphics::IGraphic(proxy)
    , VxSim::IDynamics(proxy)
    , mViewport(nullptr)
    , mFrameCount(0)
    , pSaveScreenshots(false, "Save screenshots", &getProxy()->getParameterContainer())
    , pOutputPath("","Output path", &getProxy()->getParameterContainer(), true)
{
    const std::string tempDir = VxPlatform::Directory(VxPlatform::Directory::kTempDirectory).toUniversalPath();
    pOutputPath = tempDir;
}

ExportScreenshots::~ExportScreenshots()
{
}

void ExportScreenshots::preStep()
{
    if (!mViewport || !pSaveScreenshots.getValue() )
    {
        return;
    }
    // else:

    VxGraphics::Window* window = VxGraphics::findFirstWindowForViewport(mViewport);
    if (window)
    { 
        std::string filename = pOutputPath.getValue();
        filename += "/";
        char buf[64];
        sprintf(buf, "sshot_%06d.png", mFrameCount);
        filename += buf;

        window->takeScreenshot(filename);
        window->swapBuffers();
        ++mFrameCount;
    }
}

void ExportScreenshots::onApplicationModeChange(VxSim::eApplicationMode previous, VxSim::eApplicationMode next)
{
    if (previous == VxSim::kModeEditing && next == VxSim::kModeSimulating)
    {
        // reset frame count when starting simulation
        mFrameCount = 0;
    }
}

void ExportScreenshots::onAddToUniverse(Vx::VxUniverse* universe)
{
    mFrameCount = 0;
}

void ExportScreenshots::onAddToView(VxGraphics::Viewport* viewport)
{
    mViewport = viewport;
}
